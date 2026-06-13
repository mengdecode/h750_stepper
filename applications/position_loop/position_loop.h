/**
 * @file position_loop.h
 * @brief 位置环控制器 —— S 形加减速 + PID + 到位锁定
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 控制架构:
 *   target → [S 形速度斜坡] → [PID + 前馈] → DIR + 脉冲频率
 *                       ↑                          ↓
 *                encoder ←────────────── 位置反馈 (32-bit counts)
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          S-curve, position_scale, blocking API
 */

#ifndef POSITION_LOOP_H__
#define POSITION_LOOP_H__

#include "../stepper_drv/stepper_motor_driver.h"
#include "../stepper_drv/stepper_encoder_driver.h"
#include "../stepper_drv/stepper_pid_pos.h"
#include "../stepper_drv/position_scale.h"

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
    position_scale_t scale;        /* 物理位置 ↔ 计数值 拟合    */

    /* 目标位置斜坡 (S 形速度曲线) */
    rt_int32_t target_pos;        /* 最终目标位置 (counts)     */
    rt_int32_t ramp_pos;          /* 斜坡平滑后的即时目标      */
    float      ramp_speed;        /* 当前斜坡速度 (Hz)         */
    float      ramp_accel;        /* 当前加速度 (Hz/period)    */
    float      s_curve_jerk;      /* S 形加加速度 (Hz/period²) */
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
 * @param max_accel       最大加速度 (Hz/period)
 * @param s_curve_jerk    S 形加加速度 (Hz/period²), 越小越平滑
 * @param ramp_max_speed  最大斜坡速度 (Hz)
 * @param counts_per_unit 每个物理单位的编码器计数值
 * @param unit            单位名 ("mm", "°" 等)
 * @param pos_min / pos_max 物理软限位
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
                            float                     s_curve_jerk,
                            float                     ramp_max_speed,
                            float                     counts_per_unit,
                            const char               *unit,
                            float                     pos_min,
                            float                     pos_max);

/** 启动位置环 */
void position_loop_start(position_loop_t *pl);

/** 停止位置环 */
void position_loop_stop(position_loop_t *pl);

/** 设置目标位置 (counts) */
void position_loop_set_target(position_loop_t *pl, rt_int32_t target);

/** 获取当前位置 (counts) */
rt_int32_t position_loop_get_position(position_loop_t *pl);

/** 获取当前位置 (物理单位) */
float position_loop_get_position_physical(position_loop_t *pl);

/** 阻塞式定位 — 物理单位版 */
int position_loop_move_to_physical(position_loop_t *pl,
                                    float target, rt_uint32_t timeout_ms);

/** 是否已到位 */
int position_loop_is_arrived(position_loop_t *pl);

/**
 * 阻塞式定位: 设目标, 等到位/故障/超时
 * @param timeout_ms  超时 (ms), 0=不限
 * @return MOTOR_E_OK(到位) / MOTOR_E_FAULT / MOTOR_E_HOME_TIMEOUT(超时)
 */
int position_loop_move_to(position_loop_t *pl,
                          rt_int32_t target, rt_uint32_t timeout_ms);

/**
 * 阻塞式找零
 * @param home_pin    原点引脚
 * @param home_dir    找零方向
 * @param speed_hz    找零速度
 * @param timeout_ms  超时 (ms)
 * @return MOTOR_E_OK 或错误码
 */
int position_loop_home(position_loop_t *pl, stepper_encoder_driver_t *enc,
                       rt_base_t home_pin, int home_dir,
                       rt_uint32_t speed_hz, rt_uint32_t timeout_ms);

static __inline int position_loop_is_running(const position_loop_t *pl)
{
    return pl->running;
}

#ifdef __cplusplus
}
#endif
#endif
