/*
 * 步进电机开环速度控制器 (带加速度斜坡, 多实例支持)
 *
 * 无编码器反馈, 仅按设定频率 + 斜坡输出 PWM 脉冲。
 */

#ifndef STEPPER_OPENLOOP_H__
#define STEPPER_OPENLOOP_H__

#include "../stepper_drv/stepper_motor_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct stepper_openloop {
    stepper_motor_driver_t *motor;

    rt_device_t   timer_dev;
    rt_uint32_t   period_us;
    rt_uint32_t   freq_hz;

    rt_thread_t   thread;
    rt_sem_t      sem;
    volatile int  running;

    float  max_accel;          /* 频率变化上限 / 控制周期 */
    float  target_freq;        /* 目标步进频率 (Hz)        */
    float  current_freq;       /* 斜坡平滑后的当前频率      */
    int    direction;          /* MOTOR_DIR_CW / CCW        */

    /* 位置模式 */
    int    pos_mode;           /* 1=梯形加减速到位自动停    */
    float  move_speed;         /* 匀速段目标速度 (Hz)       */
    float  total_steps;        /* 本次移动总步数             */
    float  acc_steps;          /* 已累计步数                 */

    struct stepper_openloop *next;
} stepper_openloop_t;

rt_err_t stepper_openloop_init(stepper_openloop_t      *ol,
                               stepper_motor_driver_t *motor,
                               const char         *timer_device,
                               rt_uint32_t         period_us,
                               float               max_accel);

void stepper_openloop_start(stepper_openloop_t *ol);
void stepper_openloop_stop(stepper_openloop_t *ol);
void stepper_openloop_set_freq(stepper_openloop_t *ol, float freq_hz);
void stepper_openloop_set_direction(stepper_openloop_t *ol, int dir);
void stepper_openloop_move(stepper_openloop_t *ol, float steps, float speed_hz);

static __inline int stepper_openloop_is_running(const stepper_openloop_t *ol) {
    return ol->running;
}
static __inline float stepper_openloop_get_current_freq(const stepper_openloop_t *ol) {
    return ol->current_freq;
}
static __inline float stepper_openloop_get_acc_steps(const stepper_openloop_t *ol) {
    return ol->acc_steps;
}

#ifdef __cplusplus
}
#endif
#endif
