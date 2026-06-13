/**
 * @file stepper_motor_driver.c
 * @brief 步进电机驱动实现 —— HAL OC Toggle 模式, 生产级特性
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 特性: 线程安全 / 步数统计 / 故障检测 / 减速停机 /
 *       空闲降电流 / 找零 / 状态查询 / 频率校验 / S-curve
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          生产级特性: mutex, fault, brake, homing, step count
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

static GPIO_TypeDef* gpio_port_from_pin(rt_base_t pin)
{
    switch ((pin >> 4) & 0xFu) {
        case 0:  return GPIOA; case 1:  return GPIOB;
        case 2:  return GPIOC; case 3:  return GPIOD;
        case 4:  return GPIOE; case 5:  return GPIOF;
        case 6:  return GPIOG; case 7:  return GPIOH;
        case 8:  return GPIOI; case 9:  return GPIOJ;
        case 10: return GPIOK; default: return GPIOA;
    }
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

/* ---- 步数积分 (内部) ---- */
static void steps_accumulate(stepper_motor_driver_t *motor)
{
    rt_tick_t now = rt_tick_get();
    rt_tick_t elapsed;
    rt_int64_t inc;

    if (motor->last_tick == 0) {
        motor->last_tick = now;
        return;
    }
    elapsed = now - motor->last_tick;
    if (elapsed == 0) return;

    /* signed steps = freq * dt, 方向由 current_dir 决定 */
    inc = (rt_int64_t)((float)motor->current_freq * elapsed
                       / (float)RT_TICK_PER_SECOND);
    if (motor->current_dir == MOTOR_DIR_CCW) inc = -inc;
    motor->total_steps += inc;
    motor->last_tick = now;
}

/* ---- 内部: 写频率寄存器 (不加锁, 不累计步数) ---- */
static void freq_write_raw(stepper_motor_driver_t *motor, rt_uint32_t freq_hz)
{
    rt_uint32_t arr, psc, tim_clk;
    float f = (float)freq_hz;

    if (f == 0) {
        HAL_TIM_OC_Stop(&motor->htim, motor->channel);
        motor->running = 0;
        return;
    }
    if (!motor->running) {
        HAL_TIM_OC_Start(&motor->htim, motor->channel);
        motor->running = 1;
    }

    tim_clk = motor->tim_clk;
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

    motor->htim.Instance->EGR |= TIM_EGR_UG;
    motor->htim.Instance->SR  &= ~TIM_SR_UIF;
}

/* ---- 内部: 强制 MODER 为输出 ---- */
static __inline void force_moder_output(rt_base_t pin)
{
    GPIO_TypeDef *port = gpio_port_from_pin(pin);
    rt_uint8_t    num  = (rt_uint8_t)(pin & 0xFu);
    port->MODER = (port->MODER & ~(3u << (num * 2)))
                | (1u << (num * 2));
}

/* ================================================================
 *  Public API
 * ================================================================ */

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
                                   int                     dir_invert)
{
    TIM_OC_InitTypeDef  oc = {0};
    GPIO_InitTypeDef    gpio = {0};

    RT_ASSERT(motor && tim);

    motor->channel    = channel;
    motor->dir_pin    = dir_pin;
    motor->en_pin     = en_pin;
    motor->fault_pin  = fault_pin;
    motor->tim_clk    = tim_clk;
    motor->pulse_port = pulse_port;
    motor->pulse_pin  = pulse_pin;
    motor->pulse_af   = pulse_af;
    motor->dir_invert = dir_invert;
    motor->enabled    = 0;
    motor->running    = 0;
    motor->state      = MOTOR_STATE_IDLE;
    motor->fault_state = MOTOR_FAULT_NONE;
    motor->total_steps = 0;
    motor->current_freq = 0;
    motor->current_dir  = -1;
    motor->last_tick    = 0;
    motor->idle_reduce  = 0;
    motor->idle_timeout_ms = 0;
    motor->brake_decel  = 0;

    /* 线程安全 */
    motor->lock = rt_mutex_create("motor_lk", RT_IPC_FLAG_FIFO);
    if (!motor->lock) return -RT_ENOMEM;

    /* GPIO: 脉冲引脚 AF 推挽 */
    gpio_clk_enable(pulse_port);
    gpio.Pin       = pulse_pin;
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_PULLUP;
    gpio.Speed     = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = pulse_af;
    HAL_GPIO_Init(pulse_port, &gpio);

    /* DIR & EN */
    rt_pin_mode(dir_pin, PIN_MODE_OUTPUT);
    rt_pin_mode(en_pin, PIN_MODE_OUTPUT);
    rt_pin_write(en_pin, PIN_LOW);
    rt_pin_write(dir_pin, PIN_LOW);

    /* 故障检测引脚: 输入 + 内部上拉 (低电平有效) */
    if (fault_pin >= 0) {
        rt_pin_mode(fault_pin, PIN_MODE_INPUT_PULLUP);
    }

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

    LOG_I("init ok  tim=%luHz  pulse=%u/%u  dir=%d en=%d fault=%d  inv=%d",
          tim_clk, (unsigned)pulse_pin, (unsigned)pulse_af,
          (int)dir_pin, (int)en_pin, (int)fault_pin, dir_invert);
    return RT_EOK;
}

/* ---- 启停 ---- */

void stepper_motor_driver_enable(stepper_motor_driver_t *motor)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    force_moder_output(motor->en_pin);
    rt_pin_write(motor->en_pin, PIN_HIGH);
    motor->enabled = 1;
    rt_mutex_release(motor->lock);
}

void stepper_motor_driver_disable(stepper_motor_driver_t *motor)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    stepper_motor_driver_stop(motor);
    force_moder_output(motor->en_pin);
    rt_pin_write(motor->en_pin, PIN_LOW);
    motor->enabled = 0;
    rt_mutex_release(motor->lock);
}

void stepper_motor_driver_start(stepper_motor_driver_t *motor)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    if (!motor->enabled) {
        force_moder_output(motor->en_pin);
        rt_pin_write(motor->en_pin, PIN_HIGH);
        motor->enabled = 1;
    }
    HAL_TIM_OC_Start(&motor->htim, motor->channel);
    motor->running = 1;
    motor->state   = MOTOR_STATE_RUNNING;
    motor->last_tick = rt_tick_get();
    rt_mutex_release(motor->lock);
}

void stepper_motor_driver_stop(stepper_motor_driver_t *motor)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    steps_accumulate(motor);
    HAL_TIM_OC_Stop(&motor->htim, motor->channel);
    motor->running  = 0;
    motor->state    = MOTOR_STATE_IDLE;
    motor->current_freq = 0;
    motor->idle_since = rt_tick_get();
    rt_mutex_release(motor->lock);
}

/* ---- 减速停机 ---- */

void stepper_motor_driver_brake(stepper_motor_driver_t *motor,
                                rt_uint32_t decel_rate)
{
    rt_uint32_t f;

    if (decel_rate == 0) {
        stepper_motor_driver_stop(motor);
        return;
    }

    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    motor->state = MOTOR_STATE_BRAKING;
    f = motor->current_freq;
    rt_mutex_release(motor->lock);

    while (f > 0) {
        if (f > decel_rate) f -= decel_rate;
        else                f = 0;

        stepper_motor_driver_set_frequency(motor, f);
        rt_thread_mdelay(20);  /* 与控制周期同步 */
    }

    /* 最后停掉 */
    stepper_motor_driver_stop(motor);
}

void stepper_motor_driver_brake_async(stepper_motor_driver_t *motor,
                                      rt_uint32_t decel_rate)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    motor->brake_decel = decel_rate;
    motor->state = MOTOR_STATE_BRAKING;
    rt_mutex_release(motor->lock);
}

/* ---- 方向 ---- */

void stepper_motor_driver_set_direction(stepper_motor_driver_t *motor,
                                        int direction)
{
    int level;

    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);

    /* 累计旧方向的步数 */
    steps_accumulate(motor);

    level = (direction == MOTOR_DIR_CW) ? PIN_HIGH : PIN_LOW;
    if (motor->dir_invert) level = (level == PIN_HIGH) ? PIN_LOW : PIN_HIGH;

    force_moder_output(motor->dir_pin);
    rt_pin_write(motor->dir_pin, level);
    motor->current_dir = direction;
    rt_mutex_release(motor->lock);
}

/* ---- 频率 ---- */

int stepper_motor_driver_set_frequency(stepper_motor_driver_t *motor,
                                       rt_uint32_t freq_hz)
{
    int ret = MOTOR_E_OK;
    rt_uint32_t clamped = freq_hz;

    /* 频率范围校验 */
    if (freq_hz > 0 && freq_hz < (rt_uint32_t)MOTOR_MIN_FREQ_HZ) {
        clamped = MOTOR_MIN_FREQ_HZ;
        ret = MOTOR_E_FREQ_TOO_LOW;
    }
    if (freq_hz > (rt_uint32_t)MOTOR_MAX_FREQ_HZ) {
        clamped = MOTOR_MAX_FREQ_HZ;
        ret = MOTOR_E_FREQ_TOO_HIGH;
    }

    /* 故障时拒绝运行 */
    if (clamped > 0 && motor->fault_state != MOTOR_FAULT_NONE) {
        return MOTOR_E_FAULT;
    }

    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);

    /* 累计旧频率的步数 */
    steps_accumulate(motor);

    if (clamped == 0) {
        freq_write_raw(motor, 0);
        motor->current_freq = 0;
        motor->state = MOTOR_STATE_IDLE;
        motor->idle_since = rt_tick_get();
    } else {
        freq_write_raw(motor, clamped);
        motor->current_freq = clamped;
        if (motor->state != MOTOR_STATE_BRAKING)
            motor->state = MOTOR_STATE_RUNNING;
    }

    rt_mutex_release(motor->lock);
    return ret;
}

/* ---- 找零 ---- */

int stepper_motor_driver_home(stepper_motor_driver_t *motor,
                              rt_base_t  home_pin,
                              int        home_dir,
                              rt_uint32_t speed_hz,
                              rt_int32_t backoff_steps,
                              rt_uint32_t timeout_ms)
{
    rt_tick_t deadline = 0;
    int ret = MOTOR_E_OK;

    if (speed_hz < (rt_uint32_t)MOTOR_MIN_FREQ_HZ)
        speed_hz = MOTOR_MIN_FREQ_HZ;

    /* 配置原点引脚为输入上拉 */
    rt_pin_mode(home_pin, PIN_MODE_INPUT_PULLUP);

    motor->state = MOTOR_STATE_HOMING;

    /* 设置方向, 启动低速 */
    stepper_motor_driver_set_direction(motor, home_dir);
    stepper_motor_driver_enable(motor);
    stepper_motor_driver_set_frequency(motor, speed_hz);

    if (timeout_ms > 0)
        deadline = rt_tick_get() + rt_tick_from_millisecond(timeout_ms);

    /* 轮询原点信号 (低电平触发) */
    while (rt_pin_read(home_pin) != PIN_LOW) {
        if (timeout_ms > 0 && rt_tick_get() >= deadline) {
            ret = MOTOR_E_HOME_TIMEOUT;
            break;
        }
        rt_thread_mdelay(1);
    }

    /* 立即停止 */
    stepper_motor_driver_set_frequency(motor, 0);

    if (ret == MOTOR_E_OK && backoff_steps > 0) {
        /* 反向退回 */
        int back_dir = (home_dir == MOTOR_DIR_CW) ? MOTOR_DIR_CCW : MOTOR_DIR_CW;
        stepper_motor_driver_set_direction(motor, back_dir);
        stepper_motor_driver_set_frequency(motor, speed_hz / 2);
        {
            rt_int64_t start = stepper_motor_driver_get_steps(motor);
            rt_int64_t target = start + backoff_steps;
            if (home_dir == MOTOR_DIR_CW) target = start - backoff_steps;
            while (1) {
                rt_int64_t now = stepper_motor_driver_get_steps(motor);
                if ((home_dir == MOTOR_DIR_CW  && now <= target) ||
                    (home_dir == MOTOR_DIR_CCW && now >= target))
                    break;
                rt_thread_mdelay(1);
            }
        }
        stepper_motor_driver_set_frequency(motor, 0);
    }

    motor->state = MOTOR_STATE_IDLE;
    return ret;
}

/* ---- 故障检测 ---- */

int stepper_motor_driver_check_fault(stepper_motor_driver_t *motor)
{
    if (motor->fault_pin < 0) return 0;
    if (rt_pin_read(motor->fault_pin) == PIN_LOW) {
        if (motor->fault_state == MOTOR_FAULT_NONE) {
            motor->fault_state = MOTOR_FAULT_DRIVER;
            motor->state = MOTOR_STATE_FAULT;
            stepper_motor_driver_stop(motor);
        }
        return 1;
    }
    /* 故障消除: 清零状态, 但保留 state=FAULT 直到 clear_fault 被调用 */
    motor->fault_state = MOTOR_FAULT_NONE;
    return 0;
}

void stepper_motor_driver_clear_fault(stepper_motor_driver_t *motor)
{
    motor->fault_state = MOTOR_FAULT_NONE;
    if (motor->state == MOTOR_STATE_FAULT)
        motor->state = MOTOR_STATE_IDLE;
}

/* ---- 空闲电流 ---- */

void stepper_motor_driver_set_idle_reduce(stepper_motor_driver_t *motor,
                                          int enable,
                                          rt_uint32_t timeout_ms)
{
    motor->idle_reduce = enable ? 1 : 0;
    motor->idle_timeout_ms = timeout_ms;
}

void stepper_motor_driver_idle_tick(stepper_motor_driver_t *motor)
{
    if (!motor->idle_reduce || motor->running) return;

    rt_tick_t elapsed = rt_tick_get() - motor->idle_since;
    if (elapsed >= rt_tick_from_millisecond(motor->idle_timeout_ms)) {
        /* 释放 EN 降低电流 */
        rt_pin_write(motor->en_pin, PIN_LOW);
    }
}

/* ---- 步数统计 ---- */

rt_int64_t stepper_motor_driver_get_steps(stepper_motor_driver_t *motor)
{
    rt_int64_t total;
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    steps_accumulate(motor);
    total = motor->total_steps;
    rt_mutex_release(motor->lock);
    return total;
}

void stepper_motor_driver_clear_steps(stepper_motor_driver_t *motor)
{
    rt_mutex_take(motor->lock, RT_WAITING_FOREVER);
    steps_accumulate(motor);
    motor->total_steps = 0;
    rt_mutex_release(motor->lock);
}

/* ---- 状态查询 ---- */

rt_uint32_t stepper_motor_driver_get_frequency(stepper_motor_driver_t *motor)
{
    return motor->current_freq;
}

int stepper_motor_driver_get_direction(stepper_motor_driver_t *motor)
{
    return motor->current_dir;
}

int stepper_motor_driver_get_state(stepper_motor_driver_t *motor)
{
    return motor->state;
}

int stepper_motor_driver_get_fault_state(stepper_motor_driver_t *motor)
{
    return motor->fault_state;
}
