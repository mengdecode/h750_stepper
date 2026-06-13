/**
 * @file position_loop.c
 * @brief 位置环控制器实现 —— S 形加减速 + 位置式 PID + 到位锁定
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          S-curve, jerk-limit, blocking API, fault/idle tick
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
                            float                     s_curve_jerk,
                            float                     ramp_max_speed,
                            float                     counts_per_unit,
                            const char               *unit,
                            float                     pos_min,
                            float                     pos_max)
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
    pl->s_curve_jerk   = s_curve_jerk;
    pl->ramp_max_speed = ramp_max_speed;
    pl->enc_invert     = enc_invert;
    position_scale_init(&pl->scale, counts_per_unit, unit, pos_min, pos_max);

    pl->running        = 0;
    pl->arrived_locked = 0;
    pl->target_pos   = 0;
    pl->ramp_pos     = 0;
    pl->ramp_speed   = 0;
    pl->ramp_accel   = 0;
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
          "thr=%.0f  accel=%.0f  jerk=%.0f  ramp=%.0fHz  "
          "scale=%.0fcnt/%s  range[%.0f, %.0f]%s",
          timer_device, (unsigned long)period_us, (unsigned long)pl->freq_hz,
          (double)speed_limit, (double)move_threshold,
          (double)max_accel, (double)s_curve_jerk, (double)ramp_max_speed,
          (double)counts_per_unit, unit,
          (double)pos_min, (double)pos_max, unit);
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
            pl->ramp_accel = 0;
            pl->output_freq = 0;
            continue;
        }

        /* 1. 读编码器位置 */
        enc_pos = stepper_encoder_driver_read(pl->encoder);
        if (pl->enc_invert) enc_pos = -enc_pos;
        pl->current_pos = enc_pos;
        pl->last_enc    = enc_pos;

        /* 1.5 故障检测 & 空闲电流 */
        if (stepper_motor_driver_check_fault(pl->motor)) {
            LOG_E("motor fault detected, stopping");
            position_loop_stop(pl);
            continue;
        }
        stepper_motor_driver_idle_tick(pl->motor);

        /* 2. 目标位置 S 形斜坡
         *    加速度的变化率受 jerk 限制, 速度曲线自然形成 S 形
         *    对比梯形: 梯形加速度突变 → 电机抖动;  S 形平滑 → 运行安静 */
        {
            rt_int32_t ramp_err = pl->target_pos - pl->ramp_pos;
            int        sign     = (ramp_err > 0) ? 1 : ((ramp_err < 0) ? -1 : 0);
            float      remain   = fabsf((float)ramp_err);

            if (sign != 0 && remain > 0.5f) {
                /* S 形制动速度: v² = 2*a*(d/freq)*freq² → v = sqrt(2*a*freq*d)
                 *   0.75 安全系数补偿 S 形平滑多用的制动距离 */
                float v_stop_limit = sqrtf(2.0f * pl->max_accel
                                           * (float)pl->freq_hz * remain * 0.75f);
                float v_target;

                /* 目标速度: 取减速限制 与 最大巡航速度 的较小值 */
                v_target = (v_stop_limit < pl->ramp_max_speed)
                         ? v_stop_limit : pl->ramp_max_speed;
                v_target *= (float)sign;

                /* 期望加速度 = 消除速度差所需的加速度 */
                float a_desired = v_target - pl->ramp_speed;

                /* 硬限幅到 ±max_accel */
                if (a_desired >  pl->max_accel) a_desired =  pl->max_accel;
                if (a_desired < -pl->max_accel) a_desired = -pl->max_accel;

                /* ---- S 形核心: 加加速度 (jerk) 平滑 ----
                 *   加速度不跳变, 而是每周期最多变化 ±jerk */
                float jerk = pl->s_curve_jerk;
                if (a_desired > pl->ramp_accel + jerk) {
                    pl->ramp_accel += jerk;
                } else if (a_desired < pl->ramp_accel - jerk) {
                    pl->ramp_accel -= jerk;
                } else {
                    pl->ramp_accel = a_desired;
                }

                /* 积分: 速度 += 加速度 */
                pl->ramp_speed += pl->ramp_accel;
                if (pl->ramp_speed >  pl->ramp_max_speed) pl->ramp_speed =  pl->ramp_max_speed;
                if (pl->ramp_speed < -pl->ramp_max_speed) pl->ramp_speed = -pl->ramp_max_speed;

                /* 积分: 位置 += 速度 / 控制频率 */
                pl->ramp_pos += (rt_int32_t)(pl->ramp_speed / (float)pl->freq_hz);

                /* 防止超调 */
                if ((sign > 0 && pl->ramp_pos > pl->target_pos) ||
                    (sign < 0 && pl->ramp_pos < pl->target_pos))
                    pl->ramp_pos = pl->target_pos;

                /* 防止斜坡超前/滞后电机太多: 限幅 = max(200, 2周期行程) */
                {
                    rt_int32_t lead = (rt_int32_t)(fabsf(pl->ramp_speed)
                                     / (float)pl->freq_hz * 2.0f);
                    if (lead < 200) lead = 200;
                    if (lead > 2000) lead = 2000;
                    if (pl->ramp_pos - pl->current_pos > lead)
                        pl->ramp_pos = pl->current_pos + lead;
                    else if (pl->current_pos - pl->ramp_pos > lead)
                        pl->ramp_pos = pl->current_pos - lead;

                    /* 电机冲过目标后, ramp 必须停在目标, 不能跟着冲 */
                    if (pl->ramp_pos > pl->target_pos
                        && pl->current_pos <= pl->target_pos)
                        pl->ramp_pos = pl->target_pos;
                    else if (pl->ramp_pos < pl->target_pos
                             && pl->current_pos >= pl->target_pos)
                        pl->ramp_pos = pl->target_pos;
                }
            } else {
                /* 到位: 清零加速度和速度 */
                pl->ramp_accel = 0;
                pl->ramp_speed = 0;
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

                if (final_err == 0)
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
                /* ---- 远离目标: PID + 前馈 ---- */
                float ff_speed  = pl->ramp_speed;  /* ramp_speed 已带符号 */
                float pid_corr;
                float corr_limit = pl->ramp_max_speed * 0.05f;

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
            LOG_I("pos=%.2f%s(%ld)  ramp=%.2f%s(%ld)  "
                  "tgt=%.2f%s(%ld)  err=%.2f%s  out=%.0fHz  dir=%s",
                  (double)position_scale_to_physical(&pl->scale, pl->current_pos),
                  pl->scale.unit, (long)pl->current_pos,
                  (double)position_scale_to_physical(&pl->scale, pl->ramp_pos),
                  pl->scale.unit, (long)pl->ramp_pos,
                  (double)position_scale_to_physical(&pl->scale, pl->target_pos),
                  pl->scale.unit, (long)pl->target_pos,
                  (double)(pl->target_pos - pl->current_pos)
                    / (double)pl->scale.counts_per_unit,
                  pl->scale.unit,
                  (double)pl->output_freq,
                  pl->output_freq > 0 ? "CW" : (pl->output_freq < 0 ? "CCW" : "STOP"));
        }
    }
}

/* ---- Public API ---- */

void position_loop_start(position_loop_t *pl)
{
    if (pl->running) return;

    /* 清除上一次故障的残留状态 */
    stepper_motor_driver_clear_fault(pl->motor);

    stepper_pid_pos_reset(pl->pid);
    pl->current_pos = stepper_encoder_driver_read(pl->encoder);
    if (pl->enc_invert) pl->current_pos = -pl->current_pos;

    pl->target_pos = pl->current_pos;
    pl->ramp_pos   = pl->current_pos;
    pl->ramp_speed = 0;
    pl->ramp_accel = 0;
    pl->output_freq = 0;

    stepper_motor_driver_start(pl->motor);
    pl->running = 1;
    LOG_D("started  pos=%ld", (long)pl->current_pos);
}

void position_loop_stop(position_loop_t *pl)
{
    pl->running = 0;
    stepper_motor_driver_stop(pl->motor);

    stepper_pid_pos_reset(pl->pid);
    pl->ramp_speed   = 0;
    pl->ramp_accel   = 0;
    pl->output_freq  = 0;
    pl->ramp_pos     = 0;
    pl->target_pos   = 0;
    LOG_D("stopped");
}

void position_loop_set_target(position_loop_t *pl, rt_int32_t target)
{
    /* 软限位: 超出范围不响应 */
    if (target < pl->scale.pos_min_counts || target > pl->scale.pos_max_counts) {
        LOG_W("target %ld out of range [%ld, %ld] counts",
              (long)target, (long)pl->scale.pos_min_counts,
              (long)pl->scale.pos_max_counts);
        return;
    }
    /* 如果目标变更较大, 重置斜坡从当前位置开始 */
    if (pl->running) {
        pl->ramp_pos   = pl->current_pos;
        pl->ramp_speed = 0;
        pl->ramp_accel = 0;
        stepper_pid_pos_reset(pl->pid);
    }
    pl->target_pos      = target;
    pl->arrived_locked  = 0;  /* 新目标, 解锁 */
    LOG_D("target -> %ld  (from %ld)", (long)target, (long)pl->current_pos);
}

rt_int32_t position_loop_get_position(position_loop_t *pl)
{
    return pl->current_pos;
}

float position_loop_get_position_physical(position_loop_t *pl)
{
    return position_scale_to_physical(&pl->scale, pl->current_pos);
}

int position_loop_move_to_physical(position_loop_t *pl,
                                    float target, rt_uint32_t timeout_ms)
{
    if (!position_scale_check_range(&pl->scale, target))
        return MOTOR_E_OUT_OF_RANGE;
    rt_int32_t counts = position_scale_to_counts(&pl->scale, target);
    return position_loop_move_to(pl, counts, timeout_ms);
}

int position_loop_is_arrived(position_loop_t *pl)
{
    return abs(pl->target_pos - pl->current_pos) < (rt_int32_t)pl->move_threshold;
}

int position_loop_move_to(position_loop_t *pl,
                          rt_int32_t target, rt_uint32_t timeout_ms)
{
    rt_tick_t deadline = 0;
    int ret = MOTOR_E_OK;

    if (!pl->running) return MOTOR_E_NOT_RUNNING;
    if (pl->motor->fault_state != MOTOR_FAULT_NONE) return MOTOR_E_FAULT;
    if (target < pl->scale.pos_min_counts || target > pl->scale.pos_max_counts)
        return MOTOR_E_OUT_OF_RANGE;

    position_loop_set_target(pl, target);

    if (timeout_ms > 0)
        deadline = rt_tick_get() + rt_tick_from_millisecond(timeout_ms);

    while (!position_loop_is_arrived(pl)) {
        /* 检查故障 */
        if (pl->motor->fault_state != MOTOR_FAULT_NONE) {
            ret = MOTOR_E_FAULT;
            break;
        }
        /* 检查超时 */
        if (timeout_ms > 0 && rt_tick_get() >= deadline) {
            ret = MOTOR_E_HOME_TIMEOUT;
            break;
        }
        rt_thread_mdelay(10);
    }

    return ret;
}

int position_loop_home(position_loop_t *pl, stepper_encoder_driver_t *enc,
                       rt_base_t home_pin, int home_dir,
                       rt_uint32_t speed_hz, rt_uint32_t timeout_ms)
{
    int ret;

    if (!pl->running) return MOTOR_E_NOT_RUNNING;

    ret = stepper_motor_driver_home(pl->motor, home_pin, home_dir,
                                    speed_hz, 0, timeout_ms);
    if (ret != MOTOR_E_OK) return ret;

    /* 零点: 编码器清零, 位置环复位, 记录 home_offset */
    stepper_encoder_driver_clear(enc);
    position_scale_set_home(&pl->scale, 0);
    pl->current_pos = 0;
    pl->target_pos  = 0;
    pl->ramp_pos    = 0;

    return MOTOR_E_OK;
}
