/**
 * @file stepper_motor_driver.h
 * @brief 步进电机驱动 —— HAL OC Toggle 模式
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 输出频率 = tim_clk / (2 * (PSC+1) * (ARR+1))
 * PSC 自动选择使 ARR 在合理范围 [0, 0xFFFF].
 * 改频率只更新 ARR/PSC/CCR, 边沿无毛刺.
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          S-curve, fault, homing, scale, production features
 */

#ifndef STEPPER_MOTOR_DRIVER_H__
#define STEPPER_MOTOR_DRIVER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 方向 ---- */
#define MOTOR_DIR_CW   1
#define MOTOR_DIR_CCW  0

/* ---- 硬件频率限制 ---- */
#define MOTOR_MAX_FREQ_HZ  100000
#define MOTOR_MIN_FREQ_HZ  50

/* ---- 错误码 ---- */
#define MOTOR_E_OK            0
#define MOTOR_E_FREQ_TOO_LOW -1
#define MOTOR_E_FREQ_TOO_HIGH -2
#define MOTOR_E_FAULT        -3
#define MOTOR_E_NOT_RUNNING  -4
#define MOTOR_E_HOME_TIMEOUT -5
#define MOTOR_E_LOCKED       -6
#define MOTOR_E_OUT_OF_RANGE -7

/* ---- 驱动器状态 ---- */
#define MOTOR_STATE_IDLE     0   /* 停机 */
#define MOTOR_STATE_RUNNING  1   /* 运行中 */
#define MOTOR_STATE_BRAKING  2   /* 减速停机中 */
#define MOTOR_STATE_HOMING   3   /* 找零中 */
#define MOTOR_STATE_FAULT    4   /* 故障 */

/* ---- 故障标志 ---- */
#define MOTOR_FAULT_NONE     0
#define MOTOR_FAULT_DRIVER   (1 << 0)  /* 驱动 IC 故障 */

typedef struct {
    /* 定时器 */
    TIM_HandleTypeDef  htim;
    rt_uint32_t        channel;
    rt_uint32_t        tim_clk;

    /* 引脚 */
    GPIO_TypeDef      *pulse_port;
    rt_uint16_t        pulse_pin;
    rt_uint8_t         pulse_af;
    rt_base_t          dir_pin;
    rt_base_t          en_pin;
    rt_base_t          fault_pin;      /* 故障检测引脚, -1=未使用 */
    rt_uint8_t         dir_invert : 1;
    rt_uint8_t         enabled    : 1;
    rt_uint8_t         running    : 1;

    /* 空闲电流 */
    rt_uint8_t         idle_reduce : 1;  /* 停转时释放 EN 降低电流 */
    rt_tick_t          idle_since;       /* 停转时刻 (tick) */
    rt_uint32_t        idle_timeout_ms;  /* 空闲多久后降电流, 0=立即 */

    /* 步数统计 */
    rt_uint32_t        current_freq;     /* 当前频率 Hz */
    rt_int8_t          current_dir;      /* 当前方向: 1=CW 0=CCW -1=未设置 */
    volatile rt_int64_t total_steps;     /* 累计脉冲数 (带符号) */
    rt_tick_t          last_tick;        /* 上次频率更新时刻 */

    /* 故障 */
    volatile rt_uint8_t fault_state;     /* 故障状态字 */

    /* 状态 */
    volatile rt_uint8_t state;           /* MOTOR_STATE_xxx */

    /* 线程安全 */
    rt_mutex_t         lock;

    /* 制动参数 */
    rt_uint32_t        brake_decel;      /* 减速停机减速率 (Hz/周期), 0=立即停 */
} stepper_motor_driver_t;

/* ============ 初始化和生命周期 ============ */

/**
 * 初始化电机驱动实例
 * @param motor        实例指针
 * @param tim          TIM 外设基地址
 * @param channel      TIM 通道
 * @param tim_clk      定时器时钟 Hz
 * @param pulse_port   脉冲 GPIO 端口
 * @param pulse_pin    脉冲 GPIO 引脚
 * @param pulse_af     脉冲 AF 编号
 * @param dir_pin      方向引脚 (GET_PIN)
 * @param en_pin       使能引脚 (GET_PIN)
 * @param fault_pin    故障检测引脚 (GET_PIN), -1=不使用
 * @param dir_invert   方向反转
 */
rt_err_t stepper_motor_driver_init(stepper_motor_driver_t *motor,
                                   TIM_TypeDef            *tim,
                                   rt_uint32_t             channel,
                                   rt_uint32_t             tim_clk,
                                   GPIO_TypeDef           *pulse_port,
                                   rt_uint16_t             pulse_pin,
                                   rt_uint8_t              pulse_af,
                                   rt_base_t               dir_pin,
                                   rt_base_t               en_pin,
                                   rt_base_t               fault_pin,
                                   int                     dir_invert);

/* ============ 启停控制 ============ */

void stepper_motor_driver_enable(stepper_motor_driver_t *motor);
void stepper_motor_driver_disable(stepper_motor_driver_t *motor);
void stepper_motor_driver_start(stepper_motor_driver_t *motor);
void stepper_motor_driver_stop(stepper_motor_driver_t *motor);

/**
 * 减速停机: 以 decel_rate (Hz/周期) 斜坡减速到 0 再停
 * @param decel_rate  减速率, 0=立即急停
 * @note  阻塞调用, 在调用者线程中执行
 */
void stepper_motor_driver_brake(stepper_motor_driver_t *motor,
                                rt_uint32_t decel_rate);

/**
 * 启动非阻塞减速停机, 调用者需轮询 is_running()
 * @param decel_rate  减速率 (Hz/周期)
 */
void stepper_motor_driver_brake_async(stepper_motor_driver_t *motor,
                                      rt_uint32_t decel_rate);

/* ============ 方向 & 频率 ============ */

void stepper_motor_driver_set_direction(stepper_motor_driver_t *motor,
                                        int direction);

/**
 * 设置脉冲频率
 * @return  MOTOR_E_OK, 或错误码 (频率被钳位时也返回非零)
 */
int stepper_motor_driver_set_frequency(stepper_motor_driver_t *motor,
                                       rt_uint32_t freq_hz);

/* ============ 找零 ============ */

/**
 * 阻塞式找零: 低速向 home_pin 方向移动, 触发后立即停止
 * @param home_pin      原点传感器引脚 (GET_PIN), 低电平=触发
 * @param home_dir      找零方向 (MOTOR_DIR_CW / MOTOR_DIR_CCW)
 * @param speed_hz      找零速度
 * @param backoff_steps 到位后反向退多少步, 0=不退
 * @param timeout_ms    超时 (ms), 0=不限时
 * @return MOTOR_E_OK 或错误码
 */
int stepper_motor_driver_home(stepper_motor_driver_t *motor,
                              rt_base_t  home_pin,
                              int        home_dir,
                              rt_uint32_t speed_hz,
                              rt_int32_t backoff_steps,
                              rt_uint32_t timeout_ms);

/* ============ 故障检测 ============ */

/** 轮询故障引脚, 返回非零=有故障 */
int stepper_motor_driver_check_fault(stepper_motor_driver_t *motor);

/** 清除故障状态 */
void stepper_motor_driver_clear_fault(stepper_motor_driver_t *motor);

/* ============ 空闲电流 ============ */

/**
 * 配置空闲降电流
 * @param enable      1=启用 (停转后释放 EN)
 * @param timeout_ms  空闲多久后降电流, 0=立即
 */
void stepper_motor_driver_set_idle_reduce(stepper_motor_driver_t *motor,
                                          int enable,
                                          rt_uint32_t timeout_ms);

/** 每周期调用, 处理空闲降电流定时 */
void stepper_motor_driver_idle_tick(stepper_motor_driver_t *motor);

/* ============ 步数统计 ============ */

/** 获取累计脉冲数 (带符号: CW 正, CCW 负) */
rt_int64_t stepper_motor_driver_get_steps(stepper_motor_driver_t *motor);

/** 清零步数计数器 */
void stepper_motor_driver_clear_steps(stepper_motor_driver_t *motor);

/* ============ 状态查询 ============ */

/** 获取当前频率 (Hz), 0=停转 */
rt_uint32_t stepper_motor_driver_get_frequency(stepper_motor_driver_t *motor);

/** 获取当前方向 */
int stepper_motor_driver_get_direction(stepper_motor_driver_t *motor);

/** 获取驱动器状态 (MOTOR_STATE_xxx) */
int stepper_motor_driver_get_state(stepper_motor_driver_t *motor);

/** 获取故障状态字 */
int stepper_motor_driver_get_fault_state(stepper_motor_driver_t *motor);

static __inline int stepper_motor_driver_is_running(const stepper_motor_driver_t *motor)
{
    return motor->running;
}

static __inline int stepper_motor_driver_is_enabled(const stepper_motor_driver_t *motor)
{
    return motor->enabled;
}

#ifdef __cplusplus
}
#endif
#endif
