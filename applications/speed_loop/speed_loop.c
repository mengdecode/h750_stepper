/*
 * 速度环控制器实现 —— 纯速度单环, 位置式 PID, 加速度斜坡, 多实例
 *
 * 控制周期:
 *   1. 读编码器 32-bit 位置 → delta (counts/周期)
 *   2. 换算为实测速度: actual_hz = delta * freq_hz * (steps_per_rev / enc_counts_per_rev)
 *      freq_hz = 1/period_us * 1e6, 即控制频率
 *   3. 位置式 PID(目标速度 Hz, 实测速度 Hz) → 输出频率 Hz
 *   4. 加速度斜坡限幅
 *   5. 死区判断 → 停止或输出
 *   6. 设置方向 + 频率输出
 *
 * 多实例:
 *   全局链表 speed_loop_list, ISR 根据 hwtimer_dev 分发 semaphore.
 *   每个实例独立线程、独立 PID.
 */

#include "speed_loop.h"
#include <math.h>
#include <rtdevice.h>

#define LOG_TAG  "speed_loop"
#include <ulog.h>

static void speed_loop_thread_entry(void *param);

/* ---- 多实例链表 (ISR 根据 timer_dev 分发) ---- */
static speed_loop_t *g_sl_list = RT_NULL;

static rt_err_t sl_timer_isr(rt_device_t dev, rt_size_t size)
{
    speed_loop_t *sl = g_sl_list;
    while (sl) {
        if (sl->timer_dev == dev) {
            rt_sem_release(sl->sem);
            break;
        }
        sl = sl->next;
    }
    return RT_EOK;
}

/* ---- 初始化 ---- */
rt_err_t speed_loop_init(speed_loop_t            *sl,
                         stepper_motor_driver_t   *motor,
                         stepper_encoder_driver_t *encoder,
                         stepper_pid_pos_t        *pid,
                         const char               *timer_device,
                         rt_uint32_t               period_us,
                         rt_uint32_t               steps_per_rev,
                         rt_uint32_t               enc_counts_per_rev,
                         int                       enc_invert,
                         float                     speed_limit,
                         float                     move_threshold,
                         float                     max_accel)
{
    rt_err_t ret;
    rt_hwtimerval_t  tv;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq_val;

    RT_ASSERT(sl && motor && encoder && pid);

    sl->motor     = motor;
    sl->encoder   = encoder;
    sl->pid       = pid;
    sl->period_us = period_us;
    sl->freq_hz   = 1000000UL / period_us;

    sl->speed_limit    = speed_limit;
    sl->move_threshold = move_threshold;
    sl->max_accel      = max_accel;

    /* 机械参数: 编码器 counts → 电机 Hz 换算系数 */
    sl->speed_ratio = (float)steps_per_rev / (float)enc_counts_per_rev;
    sl->enc_invert  = enc_invert;

    sl->running       = 0;
    sl->last_enc      = 0;
    sl->enc_delta     = 0;
    sl->current_speed = 0;
    sl->output_freq   = 0;
    sl->ramp_target   = 0;
    sl->ramp_current  = 0;

    /* 信号量 */
    sl->sem = rt_sem_create("sl_sem", 0, RT_IPC_FLAG_FIFO);
    if (!sl->sem) {
        LOG_E("sem create failed");
        return -RT_ENOMEM;
    }

    /* 硬件定时器 (必须在建线程前完成——线程优先级更高，会立即抢占) */
    sl->timer_dev = rt_device_find(timer_device);
    if (!sl->timer_dev) {
        LOG_E("timer '%s' not found", timer_device);
        rt_sem_delete(sl->sem);
        return -RT_ERROR;
    }
    ret = rt_device_open(sl->timer_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret != RT_EOK) {
        LOG_E("timer '%s' open failed %d", timer_device, ret);
        rt_sem_delete(sl->sem);
        return ret;
    }

    /* 注册到全局链表 */
    sl->next = g_sl_list;
    g_sl_list = sl;

    rt_device_set_rx_indicate(sl->timer_dev, sl_timer_isr);

    /* 配置 hwtimer: 1MHz tick, 周期模式 */
    freq_val = 1000000;
    rt_device_control(sl->timer_dev, HWTIMER_CTRL_FREQ_SET, &freq_val);
    mode = HWTIMER_MODE_PERIOD;
    ret = rt_device_control(sl->timer_dev, HWTIMER_CTRL_MODE_SET, &mode);
    if (ret != RT_EOK) {
        LOG_E("set period mode failed %d", ret);
        rt_sem_delete(sl->sem);
        return ret;
    }

    tv.sec  = period_us / 1000000UL;
    tv.usec = period_us % 1000000UL;
    {
        rt_ssize_t n = rt_device_write(sl->timer_dev, 0, &tv, sizeof(tv));
        if (n != (rt_ssize_t)sizeof(tv)) {
            LOG_E("set period failed %d", (int)n);
            rt_sem_delete(sl->sem);
            return -RT_ERROR;
        }
    }

    /* 线程 */
    sl->thread = rt_thread_create("sl_ctrl", speed_loop_thread_entry, sl,
                                   2048, 5, 10);
    if (!sl->thread) {
        LOG_E("thread create failed");
        rt_sem_delete(sl->sem);
        return -RT_ENOMEM;
    }
    rt_thread_startup(sl->thread);

    LOG_I("init ok  timer=%s %luus/%luHz  ratio=%.3f(step/enc)  "
          "inv=%d  limit=%.0fHz  thr=%.0f  accel=%.0f",
          timer_device,
          (unsigned long)period_us, (unsigned long)sl->freq_hz,
          (double)sl->speed_ratio, enc_invert,
          (double)speed_limit, (double)move_threshold,
          (double)max_accel);
    return RT_EOK;
}

/* ---- 控制线程 ---- */
static void speed_loop_thread_entry(void *param)
{
    speed_loop_t *sl = (speed_loop_t *)param;
    rt_int32_t    enc_pos;
    rt_int32_t    delta;
    float         pid_out;
    float         freq_out;
    int           dir;
    int           tick = 0;

    LOG_D("thread running");

    while (1) {
        if (rt_sem_take(sl->sem, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        if (!sl->running) {
            sl->ramp_target  = 0;
            sl->ramp_current = 0;
            sl->output_freq  = 0;
            continue;
        }

        /* 1. 读编码器 → delta (counts/周期) */
        enc_pos = stepper_encoder_driver_read(sl->encoder);
        if (sl->enc_invert) enc_pos = -enc_pos;
        delta   = enc_pos - sl->last_enc;
        sl->last_enc    = enc_pos;
        sl->enc_delta   = delta;

        /* 2. 换算为实测速度 (Hz) + 轻度低通滤波 */
        {
            float raw_speed = (float)delta * (float)sl->freq_hz * sl->speed_ratio;
            /* alpha=0.5: 响应快, 同时抑制高频抖动 */
            sl->current_speed = sl->current_speed * 0.5f + raw_speed * 0.5f;
        }

        /* 3. 位置式 PID: 目标=Hz, 反馈=实测Hz */
        pid_out = stepper_pid_pos_calculate(sl->pid, sl->current_speed);

        sl->ramp_target = pid_out;

        /* 4. 加速度斜坡: ramp_current → ramp_target */
        {
            float diff = sl->ramp_target - sl->ramp_current;
            if (diff >  sl->max_accel)      sl->ramp_current += sl->max_accel;
            else if (diff < -sl->max_accel) sl->ramp_current -= sl->max_accel;
            else                            sl->ramp_current  = sl->ramp_target;
        }
        freq_out = sl->ramp_current;

        /* 符号限幅 */
        if (freq_out >  sl->speed_limit) freq_out =  sl->speed_limit;
        if (freq_out < -sl->speed_limit) freq_out = -sl->speed_limit;

        sl->output_freq = freq_out;

        /* 5. 死区判断 */
        if (fabsf(freq_out) < sl->move_threshold) {
            stepper_motor_driver_set_frequency(sl->motor, 0);
            goto log_tick;
        }

        /* 6. 方向 & 频率 */
        dir = (freq_out > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
        stepper_motor_driver_set_direction(sl->motor, dir);

        {
            rt_uint32_t abs_freq = (rt_uint32_t)fabsf(freq_out);
            if (abs_freq < (rt_uint32_t)MOTOR_MIN_FREQ_HZ)
                abs_freq = MOTOR_MIN_FREQ_HZ;
            stepper_motor_driver_set_frequency(sl->motor, abs_freq);
        }

log_tick:
        /* 每 500ms 打一次状态 */
        if (++tick >= sl->freq_hz / 2) {
            tick = 0;
            LOG_I("enc=%ld  delta=%ld  speed=%.0fHz  target=%.0fHz  "
                  "out=%.1fHz  ramp=%.1f/%.1f",
                  (long)enc_pos, (long)delta,
                  (double)sl->current_speed,
                  (double)stepper_pid_pos_get_target(sl->pid),
                  (double)sl->output_freq,
                  (double)sl->ramp_current, (double)sl->ramp_target);
        }
    }
}

/* ---- Public API ---- */

void speed_loop_start(speed_loop_t *sl)
{
    if (sl->running) return;

    /* 重置 PID 和斜坡状态 */
    stepper_pid_pos_reset(sl->pid);
    sl->ramp_target  = 0;
    sl->ramp_current = 0;
    sl->output_freq  = 0;

    /* 从当前编码器位置起步 */
    sl->last_enc = stepper_encoder_driver_read(sl->encoder);
    stepper_pid_pos_set_target(sl->pid, 0);  /* 初始目标=0 */

    stepper_motor_driver_start(sl->motor);
    sl->running = 1;
    LOG_I("started  enc=%ld", (long)sl->last_enc);
}

void speed_loop_stop(speed_loop_t *sl)
{
    sl->running = 0;
    stepper_motor_driver_stop(sl->motor);

    /* 重置所有状态 */
    stepper_pid_pos_reset(sl->pid);
    sl->ramp_target  = 0;
    sl->ramp_current = 0;
    sl->output_freq  = 0;
    sl->last_enc     = 0;
    sl->enc_delta    = 0;
    sl->current_speed = 0;
    LOG_I("stopped");
}

void speed_loop_set_target(speed_loop_t *sl, float target_hz)
{
    float old_target;

    /* 限幅 */
    if (target_hz >  sl->speed_limit) target_hz =  sl->speed_limit;
    if (target_hz < -sl->speed_limit) target_hz = -sl->speed_limit;

    old_target = stepper_pid_pos_get_target(sl->pid);

    /* 目标变更 > 20% 或换向: 重置积分，避免飞车 */
    if (fabsf(target_hz - old_target) > fabsf(old_target) * 0.2f ||
        (old_target > 0 && target_hz < 0) ||
        (old_target < 0 && target_hz > 0)) {
        stepper_pid_pos_reset(sl->pid);
        sl->ramp_target  = 0;
        sl->ramp_current = 0;
    }

    stepper_pid_pos_set_target(sl->pid, target_hz);
}

float speed_loop_get_current_speed(speed_loop_t *sl)
{
    return sl->current_speed;
}

float speed_loop_get_output_freq(speed_loop_t *sl)
{
    return sl->output_freq;
}

rt_int32_t speed_loop_get_encoder_position(speed_loop_t *sl)
{
    return sl->last_enc;
}
