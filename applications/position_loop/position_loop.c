/*
 * 位置环控制器实现 —— 纯位置单环, 位置式 PID, 梯形速度斜坡
 *
 * 控制周期:
 *   1. 读编码器 32-bit 位置
 *   2. 目标位置梯形斜坡: ramp_pos → target_pos
 *   3. 位置式 PID(ramp_pos, current_pos) → 频率 Hz
 *   4. 死区判断 → 到位停止
 *   5. 方向 + 频率输出
 *
 * 多实例:
 *   全局链表, ISR 根据 hwtimer_dev 分发 semaphore.
 */

#include "position_loop.h"
#include <math.h>
#include <rtdevice.h>

#define LOG_TAG  "pos_loop"
#include <ulog.h>

static void pl_thread_entry(void *param);

/* ---- 多实例链表 ---- */
static position_loop_t *g_pl_list = RT_NULL;

static rt_err_t pl_timer_isr(rt_device_t dev, rt_size_t size)
{
    position_loop_t *pl = g_pl_list;
    while (pl) {
        if (pl->timer_dev == dev) {
            rt_sem_release(pl->sem);
            break;
        }
        pl = pl->next;
    }
    return RT_EOK;
}

/* ---- 初始化 ---- */
rt_err_t position_loop_init(position_loop_t         *pl,
                            stepper_motor_driver_t   *motor,
                            stepper_encoder_driver_t *encoder,
                            stepper_pid_pos_t        *pid,
                            const char               *timer_device,
                            rt_uint32_t               period_us,
                            int                       enc_invert,
                            float                     speed_limit,
                            float                     move_threshold,
                            float                     max_accel,
                            float                     ramp_max_speed)
{
    rt_err_t ret;
    rt_hwtimerval_t  tv;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq_val;

    RT_ASSERT(pl && motor && encoder && pid);

    pl->motor     = motor;
    pl->encoder   = encoder;
    pl->pid       = pid;
    pl->period_us = period_us;
    pl->freq_hz   = 1000000UL / period_us;

    pl->speed_limit    = speed_limit;
    pl->move_threshold = move_threshold;
    pl->max_accel      = max_accel;
    pl->ramp_max_speed = ramp_max_speed;
    pl->enc_invert     = enc_invert;

    pl->running        = 0;
    pl->arrived_locked = 0;
    pl->target_pos   = 0;
    pl->ramp_pos     = 0;
    pl->ramp_speed   = 0;
    pl->last_enc     = 0;
    pl->current_pos  = 0;
    pl->prev_pos_err = 0;
    pl->output_freq  = 0;

    /* 信号量 */
    pl->sem = rt_sem_create("pl_sem", 0, RT_IPC_FLAG_FIFO);
    if (!pl->sem) { LOG_E("sem failed"); return -RT_ENOMEM; }

    /* 硬件定时器 */
    pl->timer_dev = rt_device_find(timer_device);
    if (!pl->timer_dev) {
        LOG_E("timer '%s' not found", timer_device);
        rt_sem_delete(pl->sem);
        return -RT_ERROR;
    }
    ret = rt_device_open(pl->timer_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret) { rt_sem_delete(pl->sem); return ret; }

    pl->next = g_pl_list;
    g_pl_list = pl;
    rt_device_set_rx_indicate(pl->timer_dev, pl_timer_isr);

    freq_val = 1000000;
    rt_device_control(pl->timer_dev, HWTIMER_CTRL_FREQ_SET, &freq_val);
    mode = HWTIMER_MODE_PERIOD;
    rt_device_control(pl->timer_dev, HWTIMER_CTRL_MODE_SET, &mode);

    tv.sec  = period_us / 1000000UL;
    tv.usec = period_us % 1000000UL;
    {
        rt_ssize_t n = rt_device_write(pl->timer_dev, 0, &tv, sizeof(tv));
        if (n != (rt_ssize_t)sizeof(tv)) {
            rt_sem_delete(pl->sem);
            return -RT_ERROR;
        }
    }

    pl->thread = rt_thread_create("pl_ctrl", pl_thread_entry, pl,
                                   2048, 5, 10);
    if (!pl->thread) { rt_sem_delete(pl->sem); return -RT_ENOMEM; }
    rt_thread_startup(pl->thread);

    LOG_I("init ok  timer=%s %luus/%luHz  limit=%.0fHz  "
          "thr=%.0f  accel=%.0f  ramp_max=%.0fHz",
          timer_device, (unsigned long)period_us, (unsigned long)pl->freq_hz,
          (double)speed_limit, (double)move_threshold,
          (double)max_accel, (double)ramp_max_speed);
    return RT_EOK;
}

/* ---- 控制线程 ---- */
static void pl_thread_entry(void *param)
{
    position_loop_t *pl = (position_loop_t *)param;
    rt_int32_t       enc_pos;
    float            freq_out;
    int              dir;
    int              tick = 0;

    LOG_D("thread running");

    while (1) {
        if (rt_sem_take(pl->sem, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        if (!pl->running) {
            pl->ramp_speed = 0;
            pl->output_freq = 0;
            continue;
        }

        /* 1. 读编码器位置 */
        enc_pos = stepper_encoder_driver_read(pl->encoder);
        if (pl->enc_invert) enc_pos = -enc_pos;
        pl->current_pos = enc_pos;
        pl->last_enc    = enc_pos;

        /* 2. 目标位置梯形斜坡 */
        {
            rt_int32_t ramp_err = pl->target_pos - pl->ramp_pos;
            int        sign     = (ramp_err > 0) ? 1 : ((ramp_err < 0) ? -1 : 0);
            float      dist     = fabsf((float)ramp_err);

            if (sign != 0) {
                /* 计算减速距离: v^2 / (2a) */
                float decel_dist = (pl->ramp_speed * pl->ramp_speed)
                                 / (2.0f * pl->max_accel * (float)pl->freq_hz);

                if (dist <= decel_dist + 1.0f) {
                    /* 减速阶段 */
                    pl->ramp_speed -= pl->max_accel;
                    if (pl->ramp_speed < pl->max_accel)
                        pl->ramp_speed = pl->max_accel;
                } else if (pl->ramp_speed < pl->ramp_max_speed) {
                    /* 加速阶段 */
                    pl->ramp_speed += pl->max_accel;
                    if (pl->ramp_speed > pl->ramp_max_speed)
                        pl->ramp_speed = pl->ramp_max_speed;
                }

                /* ramp_pos 按速度移动: speed(Hz) / freq_hz(Hz) = counts/周期 */
                pl->ramp_pos += (rt_int32_t)((float)sign * pl->ramp_speed
                                             / (float)pl->freq_hz);
                /* 防止超调 */
                if ((sign > 0 && pl->ramp_pos > pl->target_pos) ||
                    (sign < 0 && pl->ramp_pos < pl->target_pos))
                    pl->ramp_pos = pl->target_pos;

                /* 防止斜坡超前电机太多: 限幅当前误差 ≤ 200 counts */
                if (pl->ramp_pos - pl->current_pos > 200)
                    pl->ramp_pos = pl->current_pos + 200;
                else if (pl->current_pos - pl->ramp_pos > 200)
                    pl->ramp_pos = pl->current_pos - 200;
            }
        }

        /* 3. 位置式 PID: 目标=ramp_pos, 反馈=当前位置 */
        {
            rt_int32_t pos_err = pl->ramp_pos - pl->current_pos;

            /* 过零时只清积分, 让前馈+PID自然减速过零再反向 */
            if ((pos_err > 0 && pl->prev_pos_err < 0) ||
                (pos_err < 0 && pl->prev_pos_err > 0)) {
                stepper_pid_pos_reset(pl->pid);
            }
            pl->prev_pos_err = pos_err;

            freq_out = 0;

            /* ---- 近目标 (|err| ≤ 20): 不用 PID, 每周期只走一步, 到位即停 ---- */
            if (fabsf((float)(pl->target_pos - pl->current_pos)) <= 20.0f)
            {
                rt_int32_t final_err = pl->target_pos - pl->current_pos;

                if (abs(final_err) <= 1)
                {
                    freq_out = 0;
                }
                else
                {
                    /* 比例速度: err*27 Hz → 3个周期(60ms)内到位
                       每步=0.625counts, 控制周期=20ms
                       N周期移动量 = F * N * 0.02 * 0.625 = F*N/80 counts
                       要移 E counts: F = E * 80/N,  N=3 → F = E*26.67 ≈ E*27 */
                    float f = fabsf((float)final_err) * 27.0f;
                    if (f < 50.0f) f = 50.0f;
                    if (f > 500.0f) f = 500.0f;
                    freq_out = (final_err > 0) ? f : -f;
                }
                pl->ramp_speed = 0;
            }
            else
            {
                /* ---- 远离目标: PID + 前馈 (原有逻辑) ---- */
                int   ramp_sign = (pl->target_pos > pl->ramp_pos) ? 1
                                : ((pl->target_pos < pl->ramp_pos) ? -1 : 0);
                float ff_speed  = (float)ramp_sign * pl->ramp_speed;
                float pid_corr;
                float corr_limit = pl->ramp_max_speed * 0.25f;

                stepper_pid_pos_set_target(pl->pid, (float)pl->ramp_pos);
                pid_corr = stepper_pid_pos_calculate(pl->pid, (float)pl->current_pos);

                if (pid_corr >  corr_limit) pid_corr =  corr_limit;
                if (pid_corr < -corr_limit) pid_corr = -corr_limit;

                freq_out = ff_speed + pid_corr;

                /* 输出限幅 */
                if (freq_out >  pl->speed_limit) freq_out =  pl->speed_limit;
                if (freq_out < -pl->speed_limit) freq_out = -pl->speed_limit;
            }

            pl->output_freq = freq_out;

            /* 4. 方向 & 频率 */
            dir = (freq_out > 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
            stepper_motor_driver_set_direction(pl->motor, dir);

            {
                rt_uint32_t abs_freq = (rt_uint32_t)fabsf(freq_out);
                /* 远离目标: 保证最低速度. 近目标低速信号已由回差逻辑保证 */
                rt_int32_t abs_err = (rt_int32_t)fabsf((float)(pl->target_pos - pl->current_pos));
                if (abs_err >= 100 || abs_freq >= (rt_uint32_t)MOTOR_MIN_FREQ_HZ)
                {
                    if (abs_freq > 0 && abs_freq < (rt_uint32_t)MOTOR_MIN_FREQ_HZ)
                        abs_freq = MOTOR_MIN_FREQ_HZ;
                }
                stepper_motor_driver_set_frequency(pl->motor, abs_freq);
            }
        }

log_tick:
        if (++tick >= pl->freq_hz / 2) {
            tick = 0;
            LOG_I("pos=%ld  ramp=%ld  target=%ld  err=%.0f  out=%.1fHz  dir=%s",
                  (long)pl->current_pos, (long)pl->ramp_pos,
                  (long)pl->target_pos,
                  (double)(pl->target_pos - pl->current_pos),
                  (double)pl->output_freq,
                  pl->output_freq > 0 ? "CW" : (pl->output_freq < 0 ? "CCW" : "STOP"));
        }
    }
}

/* ---- Public API ---- */

void position_loop_start(position_loop_t *pl)
{
    if (pl->running) return;

    stepper_pid_pos_reset(pl->pid);
    pl->current_pos = stepper_encoder_driver_read(pl->encoder);
    if (pl->enc_invert) pl->current_pos = -pl->current_pos;

    pl->target_pos = pl->current_pos;
    pl->ramp_pos   = pl->current_pos;
    pl->ramp_speed = 0;
    pl->output_freq = 0;

    stepper_motor_driver_start(pl->motor);
    pl->running = 1;
    LOG_I("started  pos=%ld", (long)pl->current_pos);
}

void position_loop_stop(position_loop_t *pl)
{
    pl->running = 0;
    stepper_motor_driver_stop(pl->motor);

    stepper_pid_pos_reset(pl->pid);
    pl->ramp_speed   = 0;
    pl->output_freq  = 0;
    pl->ramp_pos     = 0;
    pl->target_pos   = 0;
    LOG_I("stopped");
}

void position_loop_set_target(position_loop_t *pl, rt_int32_t target)
{
    /* 如果目标变更较大, 重置斜坡从当前位置开始 */
    if (pl->running) {
        pl->ramp_pos   = pl->current_pos;
        pl->ramp_speed = 0;
    }
    pl->target_pos      = target;
    pl->arrived_locked  = 0;  /* 新目标, 解锁 */
    LOG_I("target -> %ld  (from %ld)", (long)target, (long)pl->current_pos);
}

rt_int32_t position_loop_get_position(position_loop_t *pl)
{
    return pl->current_pos;
}

int position_loop_is_arrived(position_loop_t *pl)
{
    return abs(pl->target_pos - pl->current_pos) < (rt_int32_t)pl->move_threshold;
}
