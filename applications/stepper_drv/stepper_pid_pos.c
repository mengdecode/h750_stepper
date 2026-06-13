/**
 * @file stepper_pid_pos.c
 * @brief 位置式 PID 算法实现 —— 抗饱和 (back-calculation)
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          抗饱和 PID, 纯算法模块
 */

#include "stepper_pid_pos.h"

void stepper_pid_pos_init(stepper_pid_pos_t *pid,
                          float kp, float ki, float kd,
                          float integral_limit,
                          float output_limit,
                          float error_limit)
{
    pid->target         = 0;
    pid->actual         = 0;
    pid->err            = 0;
    pid->err_last       = 0;
    pid->kp             = kp;
    pid->ki             = ki;
    pid->kd             = kd;
    pid->integral_acc   = 0;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->error_limit    = error_limit;
    pid->output         = 0;
}

void stepper_pid_pos_reset(stepper_pid_pos_t *pid)
{
    pid->actual       = 0;
    pid->err          = 0;
    pid->err_last     = 0;
    pid->integral_acc = 0;
    pid->output       = 0;
}

void stepper_pid_pos_set_target(stepper_pid_pos_t *pid, float target)
{
    pid->target = target;
}

float stepper_pid_pos_get_target(const stepper_pid_pos_t *pid)
{
    return pid->target;
}

float stepper_pid_pos_get_output(const stepper_pid_pos_t *pid)
{
    return pid->output;
}

void stepper_pid_pos_set_gains(stepper_pid_pos_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void stepper_pid_pos_set_integral_limit(stepper_pid_pos_t *pid, float limit)
{
    pid->integral_limit = limit;
}

void stepper_pid_pos_set_output_limit(stepper_pid_pos_t *pid, float limit)
{
    pid->output_limit = limit;
}

float stepper_pid_pos_calculate(stepper_pid_pos_t *pid, float actual)
{
    float p_term, i_term, d_term, output;
    int   pos_saturated = 0, neg_saturated = 0;

    pid->actual = actual;
    pid->err    = pid->target - pid->actual;

    /* 误差限幅: 防止目标跳变时 P 项爆炸 */
    if (pid->error_limit > 0) {
        if (pid->err >  pid->error_limit) pid->err =  pid->error_limit;
        if (pid->err < -pid->error_limit) pid->err = -pid->error_limit;
    }

    /* 分别计算三项 */
    p_term = pid->kp * pid->err;
    i_term = pid->ki * pid->integral_acc;
    d_term = pid->kd * (pid->err - pid->err_last);
    output = p_term + i_term + d_term;

    /* ── 输出限幅 + 反算抗饱和 ── */
    if (pid->output_limit > 0) {
        if (output > pid->output_limit) {
            /* 反算: I 项只允许占 output_limit - P - D */
            i_term = pid->output_limit - p_term - d_term;
            if (i_term < 0) i_term = 0;
            if (pid->ki != 0)
                pid->integral_acc = i_term / pid->ki;
            output = pid->output_limit;
            pos_saturated = 1;
        } else if (output < -pid->output_limit) {
            i_term = -pid->output_limit - p_term - d_term;
            if (i_term > 0) i_term = 0;
            if (pid->ki != 0)
                pid->integral_acc = i_term / pid->ki;
            output = -pid->output_limit;
            neg_saturated = 1;
        }
    }

    pid->output = output;

    /* ── 积分累加 (未饱和时) ── */
    if (pid->ki != 0 && !pos_saturated && !neg_saturated) {
        pid->integral_acc += pid->err;

        /* 积分限幅 */
        if (pid->integral_limit > 0) {
            if (pid->integral_acc >  pid->integral_limit)
                pid->integral_acc =  pid->integral_limit;
            if (pid->integral_acc < -pid->integral_limit)
                pid->integral_acc = -pid->integral_limit;
        }
    }

    pid->err_last = pid->err;
    return output;
}
