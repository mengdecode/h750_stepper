/*
 * 步进电机驱动 —— HAL OC Toggle 模式, 无毛刺脉冲
 *
 * TIMx CHx 输出比较翻转: 每次 CNT==CCR 引脚翻转, 自动 50% 占空比.
 * 改变频率只更新 ARR/PSC, 边沿始终干净.
 *
 * 输出频率 = tim_clk / (2 * (PSC+1) * (ARR+1))
 *
 * 设计原则:
 *   - 不硬编码任何外设, 所有引脚/定时器通过 init() 参数传入
 *   - 多实例安全, 每个实例独立的 TIM handle
 *   - 驱动与控制器分离, 只负责脉冲输出
 *
 * 使用方式:
 *   stepper_motor_driver_t motor;
 *   stepper_motor_driver_init(&motor, TIM2, TIM_CHANNEL_1, 200000000,
 *                              GPIOA, GPIO_PIN_0, GPIO_AF1_TIM2,
 *                              GET_PIN(C,1), GET_PIN(C,3));
 *   stepper_motor_driver_enable(&motor);
 *   stepper_motor_driver_start(&motor);
 *   stepper_motor_driver_set_direction(&motor, MOTOR_DIR_CW);
 *   stepper_motor_driver_set_frequency(&motor, 5000);
 */

#ifndef STEPPER_MOTOR_DRIVER_H__
#define STEPPER_MOTOR_DRIVER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_DIR_CW   1
#define MOTOR_DIR_CCW  0

/* 硬件上限 */
#define MOTOR_MAX_FREQ_HZ  100000
#define MOTOR_MIN_FREQ_HZ  50

typedef struct {
    TIM_HandleTypeDef  htim;         /* HAL 定时器句柄               */
    rt_uint32_t        channel;      /* TIM_CHANNEL_x                */
    rt_base_t          dir_pin;      /* 方向引脚 (GET_PIN)           */
    rt_base_t          en_pin;       /* 使能引脚 (GET_PIN)           */
    rt_uint32_t        tim_clk;      /* 定时器时钟 Hz                */
    GPIO_TypeDef      *pulse_port;   /* 脉冲 GPIO 端口               */
    rt_uint16_t        pulse_pin;    /* 脉冲 GPIO 引脚               */
    rt_uint8_t         pulse_af;     /* 脉冲 复用功能                */
    rt_uint8_t         enabled : 1;  /* 使能状态                     */
    rt_uint8_t         running : 1;  /* 运行状态                     */
    rt_uint8_t         dir_invert:1; /* DIR 极性反转: 1=HW方向与逻辑相反 */
} stepper_motor_driver_t;

/**
 * 初始化电机驱动实例
 * @param motor       实例指针（调用者分配）
 * @param tim         TIM 外设基地址 (TIM2, TIM3, ...)
 * @param channel     TIM 通道 (TIM_CHANNEL_1 ~ TIM_CHANNEL_4)
 * @param tim_clk     定时器时钟频率 Hz (APB1*2 或 APB2*2)
 * @param pulse_port  脉冲 GPIO 端口 (GPIOA, GPIOB, ...)
 * @param pulse_pin   脉冲 GPIO 引脚 (GPIO_PIN_0 ~ GPIO_PIN_15)
 * @param pulse_af    脉冲 复用功能编号
 * @param dir_pin     方向引脚编号 (GET_PIN(port, pin))
 * @param en_pin      使能引脚编号 (GET_PIN(port, pin))
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
                                   int                     dir_invert);

/** 使能电机（EN 引脚拉高） */
void stepper_motor_driver_enable(stepper_motor_driver_t *motor);

/** 禁用电机（EN 引脚拉低, 并停止脉冲） */
void stepper_motor_driver_disable(stepper_motor_driver_t *motor);

/** 启动脉冲输出 */
void stepper_motor_driver_start(stepper_motor_driver_t *motor);

/** 停止脉冲输出（不改变 EN 引脚） */
void stepper_motor_driver_stop(stepper_motor_driver_t *motor);

/** 设置方向 */
void stepper_motor_driver_set_direction(stepper_motor_driver_t *motor, int direction);

/** 设置输出频率 (Hz), 0 = 停止脉冲 */
void stepper_motor_driver_set_frequency(stepper_motor_driver_t *motor, rt_uint32_t freq_hz);

/** 查询运行状态 */
static __inline int stepper_motor_driver_is_running(const stepper_motor_driver_t *motor)
{
    return motor->running;
}

/** 查询使能状态 */
static __inline int stepper_motor_driver_is_enabled(const stepper_motor_driver_t *motor)
{
    return motor->enabled;
}

#ifdef __cplusplus
}
#endif
#endif
