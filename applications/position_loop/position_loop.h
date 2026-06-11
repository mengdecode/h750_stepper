/*
 * 位置环控制器 —— 纯位置单环, 位置式 PID
 *
 * 控制架构:
 *   target(counts) → [梯形速度斜坡] → [位置式PID] → DIR + PWM频率
 *                         ↑
 *                  encoder ←── 位置反馈 (32-bit counts)
 *
 * 与 speed_loop 的区别:
 *   1. 控制目标是编码器位置 (counts), 不是速度 (Hz)
 *   2. 反馈是编码器当前位置, 无需速度换算
 *   3. PID 输出 = 频率 (Hz), 直接驱动电机
 *   4. 目标位置通过梯形速度曲线平滑过渡
 *
 * 多实例支持:
 *   全局链表, ISR 根据 hwtimer 设备分发 semaphore.
 */

#ifndef POSITION_LOOP_H__
#define POSITION_LOOP_H__

#include "../stepper_drv/stepper_motor_driver.h"
#include "../stepper_drv/stepper_encoder_driver.h"
#include "../stepper_drv/stepper_pid_pos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct position_loop {
    /* 子对象引用 (依赖注入) */
    stepper_motor_driver_t   *motor;
    stepper_encoder_driver_t *encoder;
    stepper_pid_pos_t        *pid;

    /* 控制定时器 */
    rt_device_t   timer_dev;
    rt_uint32_t   period_us;
    rt_uint32_t   freq_hz;

    /* 线程 & 同步 */
    rt_thread_t   thread;
    rt_sem_t      sem;
    volatile int  running;

    /* 位置参数 */
    float  speed_limit;           /* 定位速度上限 (Hz)       */
    float  move_threshold;        /* 到位死区 (counts)        */
    float  max_accel;             /* 加减速 (Hz/period)       */
    int    enc_invert;            /* 编码器方向反转           */

    /* 目标位置斜坡 (梯形速度曲线) */
    rt_int32_t target_pos;        /* 最终目标位置 (counts)     */
    rt_int32_t ramp_pos;          /* 斜坡平滑后的即时目标      */
    float      ramp_speed;        /* 当前斜坡速度 (Hz)         */
    float      ramp_max_speed;    /* 最大斜坡速度 (Hz)         */

    /* 运行时状态 */
    rt_int32_t last_enc;          /* 上次 32-bit 编码器值     */
    rt_int32_t current_pos;       /* 当前编码器位置            */
    rt_int32_t prev_pos_err;      /* 上次位置误差 (过零检测)    */
    float     output_freq;        /* 最近一次输出频率          */

    /* 到位锁定 (防回弹反复) */
    volatile int  arrived_locked;

    /* 链表 (多实例 ISR 分发) */
    struct position_loop *next;
} position_loop_t;

/**
 * 初始化位置环控制器
 *
 * @param pl              实例指针
 * @param motor           已初始化的电机驱动
 * @param encoder         已初始化的编码器
 * @param pid             已初始化的位置式 PID
 * @param timer_device    hwtimer 设备名
 * @param period_us       控制周期 (us)
 * @param enc_invert      编码器方向反转 (1=反转)
 * @param speed_limit     定位速度上限 (Hz)
 * @param move_threshold  到位死区 (counts)
 * @param max_accel       加减速 (Hz/period)
 * @param ramp_max_speed  梯形速度曲线最大速度 (Hz)
 */
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
                            float                     ramp_max_speed);

/** 启动位置环 */
void position_loop_start(position_loop_t *pl);

/** 停止位置环 */
void position_loop_stop(position_loop_t *pl);

/** 设置目标位置 (counts) */
void position_loop_set_target(position_loop_t *pl, rt_int32_t target);

/** 获取当前位置 */
rt_int32_t position_loop_get_position(position_loop_t *pl);

/** 是否已到位 */
int position_loop_is_arrived(position_loop_t *pl);

static __inline int position_loop_is_running(const position_loop_t *pl)
{
    return pl->running;
}

#ifdef __cplusplus
}
#endif
#endif
