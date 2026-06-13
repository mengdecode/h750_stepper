/**
 * @file stepper_encoder_driver.c
 * @brief 编码器驱动实现 —— TIM Encoder 模式, 多实例安全
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 溢出处理 (16位差值法, 无 UIF 竞态)
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#include "stepper_encoder_driver.h"

#define LOG_TAG  "enc_drv"
#include <ulog.h>

/* ---- GPIO 时钟动态使能 ---- */
static void gpio_clk_enable(GPIO_TypeDef *port)
{
    if      (port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else if (port == GPIOE) __HAL_RCC_GPIOE_CLK_ENABLE();
    else if (port == GPIOF) __HAL_RCC_GPIOF_CLK_ENABLE();
    else if (port == GPIOG) __HAL_RCC_GPIOG_CLK_ENABLE();
    else if (port == GPIOH) __HAL_RCC_GPIOH_CLK_ENABLE();
    else if (port == GPIOI) __HAL_RCC_GPIOI_CLK_ENABLE();
    else if (port == GPIOJ) __HAL_RCC_GPIOJ_CLK_ENABLE();
    else if (port == GPIOK) __HAL_RCC_GPIOK_CLK_ENABLE();
}

/* ---- TIM 时钟动态使能 ---- */
static void tim_clk_enable(TIM_TypeDef *tim)
{
    if      (tim == TIM1)  __HAL_RCC_TIM1_CLK_ENABLE();
    else if (tim == TIM2)  __HAL_RCC_TIM2_CLK_ENABLE();
    else if (tim == TIM3)  __HAL_RCC_TIM3_CLK_ENABLE();
    else if (tim == TIM4)  __HAL_RCC_TIM4_CLK_ENABLE();
    else if (tim == TIM5)  __HAL_RCC_TIM5_CLK_ENABLE();
    else if (tim == TIM6)  __HAL_RCC_TIM6_CLK_ENABLE();
    else if (tim == TIM7)  __HAL_RCC_TIM7_CLK_ENABLE();
    else if (tim == TIM8)  __HAL_RCC_TIM8_CLK_ENABLE();
    else if (tim == TIM12) __HAL_RCC_TIM12_CLK_ENABLE();
    else if (tim == TIM13) __HAL_RCC_TIM13_CLK_ENABLE();
    else if (tim == TIM14) __HAL_RCC_TIM14_CLK_ENABLE();
    else if (tim == TIM15) __HAL_RCC_TIM15_CLK_ENABLE();
    else if (tim == TIM16) __HAL_RCC_TIM16_CLK_ENABLE();
    else if (tim == TIM17) __HAL_RCC_TIM17_CLK_ENABLE();
}

rt_err_t stepper_encoder_driver_init(stepper_encoder_driver_t *enc,
                                     TIM_TypeDef              *tim,
                                     GPIO_TypeDef             *ch1_port,
                                     rt_uint16_t               ch1_pin,
                                     rt_uint8_t                ch1_af,
                                     GPIO_TypeDef             *ch2_port,
                                     rt_uint16_t               ch2_pin,
                                     rt_uint8_t                ch2_af)
{
    TIM_Encoder_InitTypeDef  enc_cfg = {0};
    GPIO_InitTypeDef         gpio    = {0};

    RT_ASSERT(enc && tim);

    enc->ch1_port  = ch1_port;
    enc->ch1_pin   = ch1_pin;
    enc->ch1_af    = ch1_af;
    enc->ch2_port  = ch2_port;
    enc->ch2_pin   = ch2_pin;
    enc->ch2_af    = ch2_af;
    enc->overflow  = 0;
    enc->last_raw  = 0;

    /* CH1 GPIO */
    gpio_clk_enable(ch1_port);
    gpio.Pin       = ch1_pin;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = ch1_af;
    HAL_GPIO_Init(ch1_port, &gpio);

    /* CH2 GPIO (端口可能不同于 CH1) */
    gpio_clk_enable(ch2_port);
    gpio.Pin       = ch2_pin;
    gpio.Mode      = GPIO_MODE_AF_OD;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = ch2_af;
    HAL_GPIO_Init(ch2_port, &gpio);

    /* TIM 编码器模式 */
    tim_clk_enable(tim);

    enc->htim.Instance               = tim;
    enc->htim.Init.Prescaler         = 0;
    enc->htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    enc->htim.Init.Period            = 0xFFFF;
    enc->htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    enc->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    enc_cfg.EncoderMode   = TIM_ENCODERMODE_TI12;
    enc_cfg.IC1Polarity   = TIM_ICPOLARITY_RISING;
    enc_cfg.IC1Selection  = TIM_ICSELECTION_DIRECTTI;
    enc_cfg.IC1Prescaler  = TIM_ICPSC_DIV1;
    enc_cfg.IC1Filter     = 0;
    enc_cfg.IC2Polarity   = TIM_ICPOLARITY_RISING;
    enc_cfg.IC2Selection  = TIM_ICSELECTION_DIRECTTI;
    enc_cfg.IC2Prescaler  = TIM_ICPSC_DIV1;
    enc_cfg.IC2Filter     = 0;

    HAL_TIM_Encoder_Init(&enc->htim, &enc_cfg);
    HAL_TIM_Encoder_Start(&enc->htim, TIM_CHANNEL_ALL);

    /* 初始化 last_raw 为当前 CNT, 避免第一次读取产生虚假大跳变 */
    enc->last_raw = (rt_uint16_t)enc->htim.Instance->CNT;

    LOG_I("init ok  tim=0x%p  ch1=%u/%u  ch2=%u/%u",
          (void *)tim,
          (unsigned)ch1_pin, (unsigned)ch1_af,
          (unsigned)ch2_pin, (unsigned)ch2_af);
    return RT_EOK;
}

rt_int32_t stepper_encoder_driver_read(stepper_encoder_driver_t *enc)
{
    rt_uint16_t raw;
    rt_int16_t  diff;
    rt_base_t   level;

    RT_ASSERT(enc);

    level = rt_hw_interrupt_disable();

    /*
     * 16位差值法: (int16_t)(raw - last_raw) 自动处理 0xFFFF↔0x0000 溢出.
     *
     * 例: last_raw=0xFFF0(-16), raw=0x0010(+16)
     *     diff = (int16_t)(0x0010 - 0xFFF0) = (int16_t)(0x0020) = 32 ✓
     *
     * 例: last_raw=0x0010(+16), raw=0xFFF0(-16)
     *     diff = (int16_t)(0xFFF0 - 0x0010) = (int16_t)(0xFFE0) = -32 ✓
     */
    raw  = (rt_uint16_t)enc->htim.Instance->CNT;
    diff = (rt_int16_t)(raw - enc->last_raw);
    enc->last_raw = raw;
    enc->overflow += diff;

    rt_hw_interrupt_enable(level);
    return enc->overflow;
}

void stepper_encoder_driver_clear(stepper_encoder_driver_t *enc)
{
    rt_base_t level;
    RT_ASSERT(enc);

    level = rt_hw_interrupt_disable();
    enc->htim.Instance->CNT = 0;
    enc->overflow = 0;
    enc->last_raw = 0;
    rt_hw_interrupt_enable(level);
}

void stepper_encoder_driver_sync(stepper_encoder_driver_t *enc)
{
    rt_base_t level;
    RT_ASSERT(enc);

    /* 以当前 CNT 为基准, 累加器清零 */
    level = rt_hw_interrupt_disable();
    enc->last_raw = (rt_uint16_t)enc->htim.Instance->CNT;
    enc->overflow = 0;
    rt_hw_interrupt_enable(level);
}
