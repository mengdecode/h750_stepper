/**
 * @file stepper_encoder_driver.h
 * @brief 编码器驱动 —— 基于 TIM Encoder 模式, 多实例安全
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 设计原则: 不硬编码外设, TIM/GPIO 全部参数传入
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#ifndef STEPPER_ENCODER_DRIVER_H__
#define STEPPER_ENCODER_DRIVER_H__

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    TIM_HandleTypeDef  htim;         /* HAL 定时器句柄                    */
    GPIO_TypeDef      *ch1_port;     /* CH1 GPIO 端口                     */
    rt_uint16_t        ch1_pin;      /* CH1 GPIO 引脚                     */
    rt_uint8_t         ch1_af;       /* CH1 复用功能                      */
    GPIO_TypeDef      *ch2_port;     /* CH2 GPIO 端口                     */
    rt_uint16_t        ch2_pin;      /* CH2 GPIO 引脚                     */
    rt_uint8_t         ch2_af;       /* CH2 复用功能                      */
    volatile rt_int32_t overflow;    /* 溢出计数器 (32位)                  */
    rt_uint16_t        last_raw;     /* 上一次 CNT 原始值 (用于 delta)    */
} stepper_encoder_driver_t;

/**
 * 初始化编码器驱动实例
 * @param enc         实例指针（调用者分配）
 * @param tim         TIM 外设基地址, 必须是支持 Encoder 模式的 TIM
 * @param ch1_port    CH1 GPIO 端口 (如 GPIOC)
 * @param ch1_pin     CH1 GPIO 引脚 (如 GPIO_PIN_6)
 * @param ch1_af      CH1 复用功能   (如 GPIO_AF2_TIM3)
 * @param ch2_port    CH2 GPIO 端口
 * @param ch2_pin     CH2 GPIO 引脚
 * @param ch2_af      CH2 复用功能
 * @return            RT_EOK 成功
 */
rt_err_t stepper_encoder_driver_init(stepper_encoder_driver_t *enc,
                                     TIM_TypeDef              *tim,
                                     GPIO_TypeDef             *ch1_port,
                                     rt_uint16_t               ch1_pin,
                                     rt_uint8_t                ch1_af,
                                     GPIO_TypeDef             *ch2_port,
                                     rt_uint16_t               ch2_pin,
                                     rt_uint8_t                ch2_af);

/**
 * 读取当前 32 位计数值 (含溢出处理)
 * @return  32 位累加计数值
 *
 * 在控制周期中调用, 内部处理 16 位溢出.
 * 临界区保护 (关中断) 保证原子性.
 */
rt_int32_t stepper_encoder_driver_read(stepper_encoder_driver_t *enc);

/** 清零计数器 (CNT 和 overflow 都归零) */
void stepper_encoder_driver_clear(stepper_encoder_driver_t *enc);

/** 同步: 以当前 CNT 为基准清零, 保留硬件计数值作为新零点 */
void stepper_encoder_driver_sync(stepper_encoder_driver_t *enc);

#ifdef __cplusplus
}
#endif
#endif
