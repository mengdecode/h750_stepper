/*
 * 步进电机驱动实现 —— HAL OC Toggle 模式
 *
 * 输出频率 = tim_clk / (2 * (PSC+1) * (ARR+1))
 * PSC 自动选择使 ARR 在合理范围 [0, 0xFFFF].
 * 改频率只更新 ARR/PSC/CCR, 边沿无毛刺.
 *
 * 与 closed_loop/stepper_motor.c 的区别:
 *   1. 脉冲 GPIO 端口不硬编码 GPIOA
 *   2. GPIO 时钟根据 port 参数动态使能
 *   3. TIM 时钟使能更完整 (支持 TIM1~TIM8)
 */

#include "stepper_motor_driver.h"

#define LOG_TAG  "motor_drv"
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

/* 从 RT-Thread pin 号获取 GPIO 端口指针 (使用 CMSIS 宏, 地址正确) */
static GPIO_TypeDef* gpio_port_from_pin(rt_base_t pin)
{
    switch ((pin >> 4) & 0xFu) {
        case 0:  return GPIOA;
        case 1:  return GPIOB;
        case 2:  return GPIOC;
        case 3:  return GPIOD;
        case 4:  return GPIOE;
        case 5:  return GPIOF;
        case 6:  return GPIOG;
        case 7:  return GPIOH;
        case 8:  return GPIOI;
        case 9:  return GPIOJ;
        case 10: return GPIOK;
        default: return GPIOA;
    }
}

static rt_uint16_t gpio_pin_from_pin(rt_base_t pin)
{
    return (rt_uint16_t)(1u << (pin & 0xFu));
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

rt_err_t stepper_motor_driver_init(stepper_motor_driver_t *motor,
                                   TIM_TypeDef            *tim,
                                   rt_uint32_t             channel,
                                   rt_uint32_t             tim_clk,
                                   GPIO_TypeDef           *pulse_port,
                                   rt_uint16_t             pulse_pin,
                                   rt_uint8_t              pulse_af,
                                   rt_base_t               dir_pin,
                                   rt_base_t               en_pin,
                                   int                     dir_invert)
{
    TIM_OC_InitTypeDef  oc = {0};
    GPIO_InitTypeDef    gpio = {0};

    RT_ASSERT(motor && tim);

    motor->channel    = channel;
    motor->dir_pin    = dir_pin;
    motor->en_pin     = en_pin;
    motor->tim_clk    = tim_clk;
    motor->pulse_port = pulse_port;
    motor->pulse_pin  = pulse_pin;
    motor->pulse_af   = pulse_af;
    motor->dir_invert = dir_invert;
    motor->enabled    = 0;
    motor->running    = 0;

    /* GPIO: 脉冲引脚 AF 推挽 */
    gpio_clk_enable(pulse_port);
    gpio.Pin       = pulse_pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = pulse_af;
    HAL_GPIO_Init(pulse_port, &gpio);

    /* DIR & EN 引脚 — 直接用 HAL_GPIO_Init, 与脉冲引脚方式一致
     *   放弃 rt_pin_mode, 彻底避免 MODER 被其他代码覆盖为 AF */
    {
        GPIO_TypeDef *dir_port, *en_port;
        GPIO_InitTypeDef gpio_dir = {0}, gpio_en = {0};

        /* 从 RT-Thread pin 号解码 GPIO 端口和引脚
         * pin 号 = port_idx * 16 + pin_num  (port_idx: A=0 B=1 C=2 ...) */
        dir_port = gpio_port_from_pin(dir_pin);
        en_port  = gpio_port_from_pin(en_pin);

        gpio_clk_enable(dir_port);
        gpio_clk_enable(en_port);

        gpio_dir.Pin       = gpio_pin_from_pin(dir_pin);
        gpio_dir.Mode      = GPIO_MODE_OUTPUT_PP;
        gpio_dir.Pull      = GPIO_NOPULL;
        gpio_dir.Speed     = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(dir_port, &gpio_dir);

        gpio_en.Pin        = gpio_pin_from_pin(en_pin);
        gpio_en.Mode       = GPIO_MODE_OUTPUT_PP;
        gpio_en.Pull       = GPIO_NOPULL;
        gpio_en.Speed      = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(en_port, &gpio_en);
    }
    HAL_GPIO_WritePin(gpio_port_from_pin(en_pin),
                       gpio_pin_from_pin(en_pin), GPIO_PIN_RESET);
    HAL_GPIO_WritePin(gpio_port_from_pin(dir_pin),
                       gpio_pin_from_pin(dir_pin), GPIO_PIN_RESET);

    /* 定时器基座 */
    tim_clk_enable(tim);

    motor->htim.Instance               = tim;
    motor->htim.Init.Prescaler         = 0;
    motor->htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    motor->htim.Init.Period            = 0xFFFF;
    motor->htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    motor->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&motor->htim);

    /* OC Toggle */
    oc.OCMode      = TIM_OCMODE_TOGGLE;
    oc.Pulse       = 0x7FFF;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    HAL_TIM_OC_ConfigChannel(&motor->htim, &oc, channel);

    LOG_I("init ok  tim_clk=%luHz  pulse_port=0x%p pin=%u af=%u  dir=%d en=%d  dir_inv=%d",
          tim_clk, (void *)pulse_port, (unsigned)pulse_pin, (unsigned)pulse_af,
          (int)dir_pin, (int)en_pin, dir_invert);
    return RT_EOK;
}

void stepper_motor_driver_enable(stepper_motor_driver_t *motor)
{
    GPIO_TypeDef *port = gpio_port_from_pin(motor->en_pin);
    rt_uint16_t   pinmask = gpio_pin_from_pin(motor->en_pin);
    rt_uint8_t    pinnum = (rt_uint8_t)(motor->en_pin & 0xFu);

    /* 强制 MODER 为输出 */
    port->MODER = (port->MODER & ~(3u << (pinnum * 2))) | (1u << (pinnum * 2));
    HAL_GPIO_WritePin(port, pinmask, GPIO_PIN_SET);
    motor->enabled = 1;
}

void stepper_motor_driver_disable(stepper_motor_driver_t *motor)
{
    GPIO_TypeDef *port = gpio_port_from_pin(motor->en_pin);
    rt_uint16_t   pinmask = gpio_pin_from_pin(motor->en_pin);
    rt_uint8_t    pinnum = (rt_uint8_t)(motor->en_pin & 0xFu);

    stepper_motor_driver_stop(motor);
    /* 强制 MODER 为输出 */
    port->MODER = (port->MODER & ~(3u << (pinnum * 2))) | (1u << (pinnum * 2));
    HAL_GPIO_WritePin(port, pinmask, GPIO_PIN_RESET);
    motor->enabled = 0;
}

void stepper_motor_driver_start(stepper_motor_driver_t *motor)
{
    if (!motor->enabled) stepper_motor_driver_enable(motor);
    HAL_TIM_OC_Start(&motor->htim, motor->channel);
    motor->running = 1;
}

void stepper_motor_driver_stop(stepper_motor_driver_t *motor)
{
    HAL_TIM_OC_Stop(&motor->htim, motor->channel);
    motor->running = 0;
}

void stepper_motor_driver_set_direction(stepper_motor_driver_t *motor, int direction)
{
    GPIO_TypeDef *port;
    rt_uint16_t   pinmask;
    rt_uint8_t    pinnum;
    int level;

    port    = gpio_port_from_pin(motor->dir_pin);
    pinmask = gpio_pin_from_pin(motor->dir_pin);
    pinnum  = (rt_uint8_t)(motor->dir_pin & 0xFu);
    level   = (direction == MOTOR_DIR_CW) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    if (motor->dir_invert) level = (level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

    /* 暴力强制 MODER 为输出模式 — 每次调用都写, 防止被任何代码覆盖 */
    port->MODER = (port->MODER & ~(3u << (pinnum * 2))) | (1u << (pinnum * 2));
    /* 然后立即写方向电平 */
    HAL_GPIO_WritePin(port, pinmask, level);
}

void stepper_motor_driver_set_frequency(stepper_motor_driver_t *motor, rt_uint32_t freq_hz)
{
    rt_uint32_t arr, psc, tim_clk;
    float f = (float)freq_hz;

    if (f < (float)MOTOR_MIN_FREQ_HZ) f = 0;
    if (f > (float)MOTOR_MAX_FREQ_HZ) f = (float)MOTOR_MAX_FREQ_HZ;

    if (f == 0) {
        /* 真停机: 关闭 OC 通道, 不留残脉冲 */
        HAL_TIM_OC_Stop(&motor->htim, motor->channel);
        motor->running = 0;
        return;
    }

    /* 如果通道关着, 重新开启 */
    if (!motor->running) {
        HAL_TIM_OC_Start(&motor->htim, motor->channel);
        motor->running = 1;
    }

    tim_clk = motor->tim_clk;

    /* 选 PSC 使 ARR 在合理范围 */
    psc = 0;
    arr = tim_clk / (2 * (rt_uint32_t)f) - 1;
    while (arr > 0xFFFF && psc < 0xFFFF) {
        psc++;
        arr = tim_clk / (2 * (rt_uint32_t)f * (psc + 1)) - 1;
    }
    if (arr > 0xFFFF) arr = 0xFFFF;

    __HAL_TIM_SET_PRESCALER(&motor->htim, psc);
    __HAL_TIM_SET_AUTORELOAD(&motor->htim, arr);
    __HAL_TIM_SET_COMPARE(&motor->htim, motor->channel, arr / 2);

    /* 立即生效 */
    motor->htim.Instance->EGR |= TIM_EGR_UG;
    motor->htim.Instance->SR  &= ~TIM_SR_UIF;
}
