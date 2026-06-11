/*
 * 速度环控制器 —— 纯速度单环, 位置式 PID
 *
 * 控制架构:
 *   target_freq(Hz) → [位置式PID] → 加速度斜坡 → DIR + PWM频率
 *                         ↑
 *                  encoder ←── 实测速度(Hz) = delta * freq_hz * (steps/enc_per_rev)
 *
 * 单位统一: target 和 feedback 都是 Hz, PID 输出也是 Hz
 *
 * 多实例支持:
 *   全局链表, ISR 根据 hwtimer 设备分发 semaphore 到对应控制器.
 *   每个控制器独立线程、独立 PID、独立电机/编码器.
 */

#ifndef SPEED_LOOP_H__
#define SPEED_LOOP_H__

#include "../stepper_drv/stepper_motor_driver.h"
#include "../stepper_drv/stepper_encoder_driver.h"
#include "../stepper_drv/stepper_pid_pos.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct speed_loop {
    /* 子对象引用 (依赖注入) */
    stepper_motor_driver_t   *motor;
    stepper_encoder_driver_t *encoder;
    stepper_pid_pos_t        *pid;

    /* 控制定时器 */
    rt_device_t   timer_dev;
    rt_uint32_t   period_us;      /* 控制周期 (us)          */
    rt_uint32_t   freq_hz;        /* 控制频率 (Hz)          */

    /* 线程 & 同步 */
    rt_thread_t   thread;
    rt_sem_t      sem;
    volatile int  running;

    /* 控制参数 */
    float  speed_limit;           /* 速度上限 (Hz)           */
    float  move_threshold;        /* 死区阈值 (Hz)           */
    float  max_accel;             /* 最大加速度 (Hz/period)  */

    /* 机械参数（编码器→电机频率换算） */
    float  speed_ratio;           /* (steps_per_rev / enc_counts_per_rev) */
    int    enc_invert;            /* 1 = 编码器方向反转                  */

    /* 斜坡状态 */
    float  ramp_target;           /* 斜坡目标速度            */
    float  ramp_current;          /* 斜坡平滑后速度          */

    /* 运行时状态 */
    rt_int32_t last_enc;          /* 上次 32-bit 编码器值   */
    rt_int32_t enc_delta;         /* 本周期编码器增量        */
    float     current_speed;      /* 当前实测速度 (Hz)       */
    float     output_freq;        /* 最近一次输出频率        */

    /* 链表 (多实例 ISR 分发) */
    struct speed_loop *next;
} speed_loop_t;

/**
 * 初始化速度环控制器
 *
 * @param sl              实例指针（调用者分配）
 * @param motor           已初始化的电机驱动实例
 * @param encoder         已初始化的编码器实例
 * @param pid             已初始化的位置式 PID 实例
 * @param timer_device    hwtimer 设备名 (如 "timer4")
 * @param period_us       控制周期 (如 20000 → 50Hz)
 * @param steps_per_rev   电机每转步数 (如 6400 = 360/1.8*32)
 * @param enc_counts_per_rev  编码器每转计数 (如 4000 = 1000*4)
 * @param enc_invert      编码器方向反转 (1=反转)
 * @param speed_limit     速度上限 (Hz), 同时作为 PID 输出限幅
 * @param move_threshold  死区阈值 (Hz), 低于此值停止脉冲
 * @param max_accel       最大加速度 (Hz/period), 斜坡限速
 * @return                RT_EOK 成功
 */
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
                         float                     max_accel);

/** 启动速度环 (重置 PID, 从当前编码器位置起步) */
void speed_loop_start(speed_loop_t *sl);

/** 停止速度环 (停止脉冲, 重置状态) */
void speed_loop_stop(speed_loop_t *sl);

/** 设置目标速度 (Hz), 正=CW, 负=CCW */
void speed_loop_set_target(speed_loop_t *sl, float target_hz);

/** 获取当前实测速度 (编码器 delta) */
float speed_loop_get_current_speed(speed_loop_t *sl);

/** 获取当前输出频率 */
float speed_loop_get_output_freq(speed_loop_t *sl);

/** 获取编码器累计位置 */
rt_int32_t speed_loop_get_encoder_position(speed_loop_t *sl);

static __inline int speed_loop_is_running(const speed_loop_t *sl)
{
    return sl->running;
}

#ifdef __cplusplus
}
#endif
#endif
