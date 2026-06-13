/**
 * @file stepper_openloop.c
 * @brief 开环速度控制器实现 —— 无编码器, 斜坡限速 + PWM 频率输出
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#include "stepper_openloop.h"
#include <math.h>
#include <rtdevice.h>

#define LOG_TAG  "openloop"
#include <ulog.h>

static void ol_thread_entry(void *param);

static stepper_openloop_t *g_ol_list = RT_NULL;

static rt_err_t ol_timer_isr(rt_device_t dev, rt_size_t size)
{
    stepper_openloop_t *ol = g_ol_list;
    while (ol) {
        if (ol->timer_dev == dev) {
            rt_sem_release(ol->sem);
            break;
        }
        ol = ol->next;
    }
    return RT_EOK;
}

rt_err_t stepper_openloop_init(stepper_openloop_t      *ol,
                               stepper_motor_driver_t *motor,
                               const char         *timer_device,
                               rt_uint32_t         period_us,
                               float               max_accel)
{
    rt_err_t ret;
    rt_hwtimerval_t  tv;
    rt_hwtimer_mode_t mode;
    rt_uint32_t freq_val;

    RT_ASSERT(ol && motor);

    ol->motor        = motor;
    ol->period_us    = period_us;
    ol->freq_hz      = 1000000UL / period_us;
    ol->max_accel    = max_accel;
    ol->running      = 0;
    ol->target_freq  = 0;
    ol->current_freq = 0;
    ol->direction    = MOTOR_DIR_CW;

    ol->sem = rt_sem_create("ol_sem", 0, RT_IPC_FLAG_FIFO);
    if (!ol->sem) { LOG_E("sem failed"); return -RT_ENOMEM; }

    ol->timer_dev = rt_device_find(timer_device);
    if (!ol->timer_dev) { LOG_E("timer '%s' not found", timer_device); rt_sem_delete(ol->sem); return -RT_ERROR; }
    ret = rt_device_open(ol->timer_dev, RT_DEVICE_OFLAG_RDWR);
    if (ret) { rt_sem_delete(ol->sem); return ret; }

    ol->next = g_ol_list;
    g_ol_list = ol;
    rt_device_set_rx_indicate(ol->timer_dev, ol_timer_isr);

    freq_val = 1000000;
    rt_device_control(ol->timer_dev, HWTIMER_CTRL_FREQ_SET, &freq_val);
    mode = HWTIMER_MODE_PERIOD;
    rt_device_control(ol->timer_dev, HWTIMER_CTRL_MODE_SET, &mode);

    tv.sec  = period_us / 1000000UL;
    tv.usec = period_us % 1000000UL;
    {
        rt_ssize_t n = rt_device_write(ol->timer_dev, 0, &tv, sizeof(tv));
        if (n != (rt_ssize_t)sizeof(tv)) {
            rt_sem_delete(ol->sem);
            return -RT_ERROR;
        }
    }

    ol->thread = rt_thread_create("ol_ctrl", ol_thread_entry, ol, 1024, 5, 10);
    if (!ol->thread) { rt_sem_delete(ol->sem); return -RT_ENOMEM; }
    rt_thread_startup(ol->thread);

    LOG_I("init ok  timer=%s %luus/%luHz  accel=%.1f",
          timer_device, period_us, ol->freq_hz, (double)max_accel);
    return RT_EOK;
}

static void ol_thread_entry(void *param)
{
    stepper_openloop_t *ol = (stepper_openloop_t *)param;
    float step_inc;

    LOG_D("thread running");

    while (1) {
        if (rt_sem_take(ol->sem, RT_WAITING_FOREVER) != RT_EOK)
            continue;

        if (!ol->running) {
            ol->current_freq = 0;
            ol->pos_mode     = 0;
            continue;
        }

        /* 位置模式: 检测减速点 */
        if (ol->pos_mode) {
            float remaining = ol->total_steps - ol->acc_steps;
            float a = ol->max_accel * (float)ol->freq_hz;

            if (remaining <= (ol->current_freq * ol->current_freq) / (2.0f * a))
                ol->target_freq = 0;

            if (remaining <= 0.5f) {
                ol->current_freq = 0;
                ol->target_freq  = 0;
                ol->pos_mode     = 0;
                stepper_motor_driver_set_frequency(ol->motor, 0);
                LOG_I("done  steps=%.0f", (double)ol->acc_steps);
                continue;
            }
        }

        /* 加速度斜坡 */
        {
            float diff = ol->target_freq - ol->current_freq;
            if (diff >  ol->max_accel)      ol->current_freq += ol->max_accel;
            else if (diff < -ol->max_accel) ol->current_freq -= ol->max_accel;
            else                            ol->current_freq  = ol->target_freq;
        }

        /* 累计步数 */
        step_inc = ol->current_freq / (float)ol->freq_hz;
        ol->acc_steps += step_inc;

        stepper_motor_driver_set_direction(ol->motor, ol->direction);
        stepper_motor_driver_set_frequency(ol->motor,
            ol->current_freq > 0.5f ? (rt_uint32_t)ol->current_freq : 0);
    }
}

void stepper_openloop_start(stepper_openloop_t *ol)
{
    if (ol->running) return;
    ol->current_freq = ol->max_accel;  /* 跳过 0Hz, 直接出脉冲 */
    ol->target_freq  = ol->max_accel;
    ol->pos_mode     = 0;
    ol->acc_steps    = 0;
    stepper_motor_driver_start(ol->motor);
    ol->running = 1;
    LOG_I("started");
}

void stepper_openloop_stop(stepper_openloop_t *ol)
{
    ol->running = 0;
    stepper_motor_driver_stop(ol->motor);
    ol->target_freq  = 0;
    ol->current_freq = 0;
    ol->pos_mode     = 0;
    ol->acc_steps    = 0;
    LOG_I("stopped");
}

void stepper_openloop_move(stepper_openloop_t *ol, float steps, float speed_hz)
{
    if (!ol->running) stepper_openloop_start(ol);

    ol->pos_mode    = 1;
    ol->move_speed  = speed_hz;
    ol->total_steps = fabsf(steps);
    ol->acc_steps   = 0;
    ol->direction   = (steps >= 0) ? MOTOR_DIR_CW : MOTOR_DIR_CCW;
    ol->target_freq = speed_hz;

    LOG_I("move  steps=%.0f  speed=%.0f Hz  dir=%s",
          (double)ol->total_steps, (double)speed_hz,
          ol->direction == MOTOR_DIR_CW ? "CW" : "CCW");
}

void stepper_openloop_set_freq(stepper_openloop_t *ol, float freq_hz)
{
    ol->target_freq = freq_hz;
}

void stepper_openloop_set_direction(stepper_openloop_t *ol, int dir)
{
    ol->direction = dir;
}
