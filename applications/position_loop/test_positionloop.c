/**
 * @file test_positionloop.c
 * @brief 位置环电机控制 —— 测试例程 & 配置
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * 命令:
 *   posloop <id> [target]          → 定位 (整数=counts, 小数=物理单位)
 *   posloop_home <id> [cw|ccw]    → 找零
 *   posloop_tune <id> <kp> <ki> <kd> → 在线调参
 *   posloop_start/stop <id>        → 启停
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          S-curve, scale, blocking API, homing, fault
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <string.h>
#include <board.h>

#include "../stepper_drv/stepper_pid_pos.h"
#include "../stepper_drv/stepper_motor_driver.h"
#include "../stepper_drv/stepper_encoder_driver.h"
#include "position_loop.h"

#define LOG_TAG  "posloop"
#include <ulog.h>

/* ---- 硬件映射表 ---- */
static TIM_TypeDef* tim_from_index(int idx)
{
    switch (idx) {
        case 1: return TIM1; case 2: return TIM2; case 3: return TIM3;
        case 4: return TIM4; case 5: return TIM5; case 8: return TIM8;
        case 12: return TIM12; case 15: return TIM15;
        default: return TIM2;
    }
}

static GPIO_TypeDef* gpio_from_index(int idx)
{
    switch (idx) {
        case 0: return GPIOA; case 1: return GPIOB; case 2: return GPIOC;
        case 3: return GPIOD; case 4: return GPIOE; case 5: return GPIOF;
        case 6: return GPIOG; case 7: return GPIOH;
        case 8: return GPIOI; case 9: return GPIOJ; case 10: return GPIOK;
        default: return GPIOA;
    }
}

static rt_uint32_t tim_ch_from_index(int ch)
{
    switch (ch) {
        case 1: return TIM_CHANNEL_1; case 2: return TIM_CHANNEL_2;
        case 3: return TIM_CHANNEL_3; case 4: return TIM_CHANNEL_4;
        default: return TIM_CHANNEL_1;
    }
}

/* ---- Kconfig 配置 fallback (仅当 rtconfig.h 未定义时生效) ---- */
#ifndef POS_LOOP_MOTOR_COUNT
#define POS_LOOP_MOTOR_COUNT  1
#endif

#ifndef POS_LOOP_CTRL_PERIOD_US
#define POS_LOOP_CTRL_PERIOD_US  20000
#endif

/* 电机 0 默认 */
#ifndef POS_LOOP_M0_TIM_INDEX
#define POS_LOOP_M0_TIM_INDEX  2
#endif
#ifndef POS_LOOP_M0_TIM_CH
#define POS_LOOP_M0_TIM_CH  1
#endif
#ifndef POS_LOOP_M0_TIM_CLK_HZ
#define POS_LOOP_M0_TIM_CLK_HZ  200000000UL
#endif
#ifndef POS_LOOP_M0_PULSE_PORT_INDEX
#define POS_LOOP_M0_PULSE_PORT_INDEX  0
#endif
#ifndef POS_LOOP_M0_PULSE_PIN
#define POS_LOOP_M0_PULSE_PIN  0
#endif
#ifndef POS_LOOP_M0_PULSE_AF
#define POS_LOOP_M0_PULSE_AF  1
#endif
#ifndef POS_LOOP_M0_DIR_PIN
#define POS_LOOP_M0_DIR_PIN  33
#endif
#ifndef POS_LOOP_M0_EN_PIN
#define POS_LOOP_M0_EN_PIN  35    /* GET_PIN(C,3) = PC3 */
#endif
#ifndef POS_LOOP_M0_ENC_TIM_INDEX
#define POS_LOOP_M0_ENC_TIM_INDEX  3
#endif
#ifndef POS_LOOP_M0_ENC_CH1_PORT_INDEX
#define POS_LOOP_M0_ENC_CH1_PORT_INDEX  2
#endif
#ifndef POS_LOOP_M0_ENC_CH1_PIN
#define POS_LOOP_M0_ENC_CH1_PIN  6
#endif
#ifndef POS_LOOP_M0_ENC_CH1_AF
#define POS_LOOP_M0_ENC_CH1_AF  2
#endif
#ifndef POS_LOOP_M0_ENC_CH2_PORT_INDEX
#define POS_LOOP_M0_ENC_CH2_PORT_INDEX  2
#endif
#ifndef POS_LOOP_M0_ENC_CH2_PIN
#define POS_LOOP_M0_ENC_CH2_PIN  7
#endif
#ifndef POS_LOOP_M0_ENC_CH2_AF
#define POS_LOOP_M0_ENC_CH2_AF  2
#endif
#ifndef POS_LOOP_M0_CTRL_TIMER
#define POS_LOOP_M0_CTRL_TIMER  "timer4"
#endif
#ifndef POS_LOOP_M0_ENC_INVERT
#define POS_LOOP_M0_ENC_INVERT  1
#endif
#ifndef POS_LOOP_M0_DIR_INVERT
#define POS_LOOP_M0_DIR_INVERT  0    /* MODER已修复, 标准极性即可 */
#endif
#ifndef POS_LOOP_M0_SPEED_LIMIT
#define POS_LOOP_M0_SPEED_LIMIT  100000
#endif
#ifndef POS_LOOP_M0_MOVE_THRESHOLD
#define POS_LOOP_M0_MOVE_THRESHOLD  20
#endif
#ifndef POS_LOOP_M0_MAX_ACCEL
#define POS_LOOP_M0_MAX_ACCEL  500
#endif
#ifndef POS_LOOP_M0_RAMP_MAX_SPEED
#define POS_LOOP_M0_RAMP_MAX_SPEED  20000
#endif
#ifndef POS_LOOP_M0_KP
#define POS_LOOP_M0_KP  2000   /* scaled x1000: 2.0 (前馈+微调) */
#endif
#ifndef POS_LOOP_M0_KI
#define POS_LOOP_M0_KI  200    /* scaled x1000: 0.2 */
#endif
#ifndef POS_LOOP_M0_KD
#define POS_LOOP_M0_KD  200    /* scaled x1000: 0.2 */
#endif
#ifndef POS_LOOP_M0_S_CURVE_JERK
#define POS_LOOP_M0_S_CURVE_JERK  (POS_LOOP_M0_MAX_ACCEL / 8)
#endif
#ifndef POS_LOOP_M0_FAULT_PIN
#define POS_LOOP_M0_FAULT_PIN  GET_PIN(E,0)  /* PE0, 低电平有效, 内部上拉 */
#endif
#ifndef POS_LOOP_M0_HOME_PIN
#define POS_LOOP_M0_HOME_PIN  GET_PIN(E,1)   /* PE1, 原点传感器 */
#endif
#ifndef POS_LOOP_M0_BRAKE_DECEL
#define POS_LOOP_M0_BRAKE_DECEL  1000  /* 急停减速率 (Hz/周期) */
#endif
#ifndef POS_LOOP_M0_COUNTS_PER_UNIT
/* 100转=1mm: PPR×4×减速比/导程 = 1000×4×100/1 = 400000 counts/mm */
#define POS_LOOP_M0_COUNTS_PER_UNIT  400000.0f
#endif
#ifndef POS_LOOP_M0_UNIT_NAME
#define POS_LOOP_M0_UNIT_NAME  "mm"
#endif
#ifndef POS_LOOP_M0_POS_MIN
#define POS_LOOP_M0_POS_MIN  0.0f
#endif
#ifndef POS_LOOP_M0_POS_MAX
#define POS_LOOP_M0_POS_MAX  100.0f
#endif

/* M1~3 默认复制 M0 */
#ifndef POS_LOOP_M1_TIM_INDEX
#define POS_LOOP_M1_TIM_INDEX  POS_LOOP_M0_TIM_INDEX
#define POS_LOOP_M1_TIM_CH  POS_LOOP_M0_TIM_CH
#define POS_LOOP_M1_TIM_CLK_HZ  POS_LOOP_M0_TIM_CLK_HZ
#define POS_LOOP_M1_PULSE_PORT_INDEX  POS_LOOP_M0_PULSE_PORT_INDEX
#define POS_LOOP_M1_PULSE_PIN  POS_LOOP_M0_PULSE_PIN
#define POS_LOOP_M1_PULSE_AF  POS_LOOP_M0_PULSE_AF
#define POS_LOOP_M1_DIR_PIN  POS_LOOP_M0_DIR_PIN
#define POS_LOOP_M1_EN_PIN  POS_LOOP_M0_EN_PIN
#define POS_LOOP_M1_ENC_TIM_INDEX  POS_LOOP_M0_ENC_TIM_INDEX
#define POS_LOOP_M1_ENC_CH1_PORT_INDEX  POS_LOOP_M0_ENC_CH1_PORT_INDEX
#define POS_LOOP_M1_ENC_CH1_PIN  POS_LOOP_M0_ENC_CH1_PIN
#define POS_LOOP_M1_ENC_CH1_AF  POS_LOOP_M0_ENC_CH1_AF
#define POS_LOOP_M1_ENC_CH2_PORT_INDEX  POS_LOOP_M0_ENC_CH2_PORT_INDEX
#define POS_LOOP_M1_ENC_CH2_PIN  POS_LOOP_M0_ENC_CH2_PIN
#define POS_LOOP_M1_ENC_CH2_AF  POS_LOOP_M0_ENC_CH2_AF
#define POS_LOOP_M1_CTRL_TIMER  POS_LOOP_M0_CTRL_TIMER
#define POS_LOOP_M1_ENC_INVERT  POS_LOOP_M0_ENC_INVERT
#define POS_LOOP_M1_DIR_INVERT  POS_LOOP_M0_DIR_INVERT
#define POS_LOOP_M1_SPEED_LIMIT  POS_LOOP_M0_SPEED_LIMIT
#define POS_LOOP_M1_MOVE_THRESHOLD  POS_LOOP_M0_MOVE_THRESHOLD
#define POS_LOOP_M1_MAX_ACCEL  POS_LOOP_M0_MAX_ACCEL
#define POS_LOOP_M1_RAMP_MAX_SPEED  POS_LOOP_M0_RAMP_MAX_SPEED
#define POS_LOOP_M1_KP  POS_LOOP_M0_KP
#define POS_LOOP_M1_KI  POS_LOOP_M0_KI
#define POS_LOOP_M1_KD  POS_LOOP_M0_KD
#define POS_LOOP_M1_S_CURVE_JERK  POS_LOOP_M0_S_CURVE_JERK
#define POS_LOOP_M1_FAULT_PIN  POS_LOOP_M0_FAULT_PIN
#define POS_LOOP_M1_HOME_PIN  POS_LOOP_M0_HOME_PIN
#define POS_LOOP_M1_BRAKE_DECEL  POS_LOOP_M0_BRAKE_DECEL
#define POS_LOOP_M1_POS_MIN  POS_LOOP_M0_POS_MIN
#define POS_LOOP_M1_POS_MAX  POS_LOOP_M0_POS_MAX
#define POS_LOOP_M1_COUNTS_PER_UNIT  POS_LOOP_M0_COUNTS_PER_UNIT
#define POS_LOOP_M1_UNIT_NAME  POS_LOOP_M0_UNIT_NAME
#endif

#ifndef POS_LOOP_M2_TIM_INDEX
#define POS_LOOP_M2_TIM_INDEX  POS_LOOP_M0_TIM_INDEX
#define POS_LOOP_M2_TIM_CH  POS_LOOP_M0_TIM_CH
#define POS_LOOP_M2_TIM_CLK_HZ  POS_LOOP_M0_TIM_CLK_HZ
#define POS_LOOP_M2_PULSE_PORT_INDEX  POS_LOOP_M0_PULSE_PORT_INDEX
#define POS_LOOP_M2_PULSE_PIN  POS_LOOP_M0_PULSE_PIN
#define POS_LOOP_M2_PULSE_AF  POS_LOOP_M0_PULSE_AF
#define POS_LOOP_M2_DIR_PIN  POS_LOOP_M0_DIR_PIN
#define POS_LOOP_M2_EN_PIN  POS_LOOP_M0_EN_PIN
#define POS_LOOP_M2_ENC_TIM_INDEX  POS_LOOP_M0_ENC_TIM_INDEX
#define POS_LOOP_M2_ENC_CH1_PORT_INDEX  POS_LOOP_M0_ENC_CH1_PORT_INDEX
#define POS_LOOP_M2_ENC_CH1_PIN  POS_LOOP_M0_ENC_CH1_PIN
#define POS_LOOP_M2_ENC_CH1_AF  POS_LOOP_M0_ENC_CH1_AF
#define POS_LOOP_M2_ENC_CH2_PORT_INDEX  POS_LOOP_M0_ENC_CH2_PORT_INDEX
#define POS_LOOP_M2_ENC_CH2_PIN  POS_LOOP_M0_ENC_CH2_PIN
#define POS_LOOP_M2_ENC_CH2_AF  POS_LOOP_M0_ENC_CH2_AF
#define POS_LOOP_M2_CTRL_TIMER  POS_LOOP_M0_CTRL_TIMER
#define POS_LOOP_M2_ENC_INVERT  POS_LOOP_M0_ENC_INVERT
#define POS_LOOP_M2_DIR_INVERT  POS_LOOP_M0_DIR_INVERT
#define POS_LOOP_M2_SPEED_LIMIT  POS_LOOP_M0_SPEED_LIMIT
#define POS_LOOP_M2_MOVE_THRESHOLD  POS_LOOP_M0_MOVE_THRESHOLD
#define POS_LOOP_M2_MAX_ACCEL  POS_LOOP_M0_MAX_ACCEL
#define POS_LOOP_M2_RAMP_MAX_SPEED  POS_LOOP_M0_RAMP_MAX_SPEED
#define POS_LOOP_M2_KP  POS_LOOP_M0_KP
#define POS_LOOP_M2_KI  POS_LOOP_M0_KI
#define POS_LOOP_M2_KD  POS_LOOP_M0_KD
#define POS_LOOP_M2_S_CURVE_JERK  POS_LOOP_M0_S_CURVE_JERK
#define POS_LOOP_M2_FAULT_PIN  POS_LOOP_M0_FAULT_PIN
#define POS_LOOP_M2_BRAKE_DECEL  POS_LOOP_M0_BRAKE_DECEL
#endif

#ifndef POS_LOOP_M3_TIM_INDEX
#define POS_LOOP_M3_TIM_INDEX  POS_LOOP_M0_TIM_INDEX
#define POS_LOOP_M3_TIM_CH  POS_LOOP_M0_TIM_CH
#define POS_LOOP_M3_TIM_CLK_HZ  POS_LOOP_M0_TIM_CLK_HZ
#define POS_LOOP_M3_PULSE_PORT_INDEX  POS_LOOP_M0_PULSE_PORT_INDEX
#define POS_LOOP_M3_PULSE_PIN  POS_LOOP_M0_PULSE_PIN
#define POS_LOOP_M3_PULSE_AF  POS_LOOP_M0_PULSE_AF
#define POS_LOOP_M3_DIR_PIN  POS_LOOP_M0_DIR_PIN
#define POS_LOOP_M3_EN_PIN  POS_LOOP_M0_EN_PIN
#define POS_LOOP_M3_ENC_TIM_INDEX  POS_LOOP_M0_ENC_TIM_INDEX
#define POS_LOOP_M3_ENC_CH1_PORT_INDEX  POS_LOOP_M0_ENC_CH1_PORT_INDEX
#define POS_LOOP_M3_ENC_CH1_PIN  POS_LOOP_M0_ENC_CH1_PIN
#define POS_LOOP_M3_ENC_CH1_AF  POS_LOOP_M0_ENC_CH1_AF
#define POS_LOOP_M3_ENC_CH2_PORT_INDEX  POS_LOOP_M0_ENC_CH2_PORT_INDEX
#define POS_LOOP_M3_ENC_CH2_PIN  POS_LOOP_M0_ENC_CH2_PIN
#define POS_LOOP_M3_ENC_CH2_AF  POS_LOOP_M0_ENC_CH2_AF
#define POS_LOOP_M3_CTRL_TIMER  POS_LOOP_M0_CTRL_TIMER
#define POS_LOOP_M3_ENC_INVERT  POS_LOOP_M0_ENC_INVERT
#define POS_LOOP_M3_DIR_INVERT  POS_LOOP_M0_DIR_INVERT
#define POS_LOOP_M3_SPEED_LIMIT  POS_LOOP_M0_SPEED_LIMIT
#define POS_LOOP_M3_MOVE_THRESHOLD  POS_LOOP_M0_MOVE_THRESHOLD
#define POS_LOOP_M3_MAX_ACCEL  POS_LOOP_M0_MAX_ACCEL
#define POS_LOOP_M3_RAMP_MAX_SPEED  POS_LOOP_M0_RAMP_MAX_SPEED
#define POS_LOOP_M3_KP  POS_LOOP_M0_KP
#define POS_LOOP_M3_KI  POS_LOOP_M0_KI
#define POS_LOOP_M3_KD  POS_LOOP_M0_KD
#define POS_LOOP_M3_S_CURVE_JERK  POS_LOOP_M0_S_CURVE_JERK
#define POS_LOOP_M3_FAULT_PIN  POS_LOOP_M0_FAULT_PIN
#define POS_LOOP_M3_BRAKE_DECEL  POS_LOOP_M0_BRAKE_DECEL
#endif

/* ---- 配置结构体 ---- */
typedef struct {
    int tim_index, tim_ch; rt_uint32_t tim_clk_hz;
    int pulse_port_idx, pulse_pin, pulse_af;
    int dir_pin, en_pin;
    int enc_tim_index;
    int enc_ch1_port_idx, enc_ch1_pin, enc_ch1_af;
    int enc_ch2_port_idx, enc_ch2_pin, enc_ch2_af;
    const char *ctrl_timer;
    int enc_invert;
    int dir_invert;
    float speed_limit, move_threshold, max_accel, s_curve_jerk, ramp_max_speed;
    int kp, ki, kd;
    int fault_pin, home_pin, brake_decel;
    float counts_per_unit;
    const char *unit_name;
    float pos_min, pos_max;
} pl_hw_cfg_t;

#define PL_MAX_MOTORS  POS_LOOP_MOTOR_COUNT

static const pl_hw_cfg_t g_pl_cfg[PL_MAX_MOTORS] = {
    /* ---- 电机 0 ---- */
    [0] = {
        .tim_index       = POS_LOOP_M0_TIM_INDEX,
        .tim_ch          = POS_LOOP_M0_TIM_CH,
        .tim_clk_hz      = POS_LOOP_M0_TIM_CLK_HZ,
        .pulse_port_idx  = POS_LOOP_M0_PULSE_PORT_INDEX,
        .pulse_pin       = POS_LOOP_M0_PULSE_PIN,
        .pulse_af        = POS_LOOP_M0_PULSE_AF,
        .dir_pin         = POS_LOOP_M0_DIR_PIN,
        .en_pin          = POS_LOOP_M0_EN_PIN,
        .enc_tim_index   = POS_LOOP_M0_ENC_TIM_INDEX,
        .enc_ch1_port_idx = POS_LOOP_M0_ENC_CH1_PORT_INDEX,
        .enc_ch1_pin     = POS_LOOP_M0_ENC_CH1_PIN,
        .enc_ch1_af      = POS_LOOP_M0_ENC_CH1_AF,
        .enc_ch2_port_idx = POS_LOOP_M0_ENC_CH2_PORT_INDEX,
        .enc_ch2_pin     = POS_LOOP_M0_ENC_CH2_PIN,
        .enc_ch2_af      = POS_LOOP_M0_ENC_CH2_AF,
        .ctrl_timer      = POS_LOOP_M0_CTRL_TIMER,
        .enc_invert      = POS_LOOP_M0_ENC_INVERT,
        .dir_invert      = POS_LOOP_M0_DIR_INVERT,
        .speed_limit     = POS_LOOP_M0_SPEED_LIMIT,
        .move_threshold  = POS_LOOP_M0_MOVE_THRESHOLD,
        .max_accel       = POS_LOOP_M0_MAX_ACCEL,
        .s_curve_jerk    = POS_LOOP_M0_S_CURVE_JERK,
        .ramp_max_speed  = POS_LOOP_M0_RAMP_MAX_SPEED,
        .kp              = POS_LOOP_M0_KP,
        .ki              = POS_LOOP_M0_KI,
        .kd              = POS_LOOP_M0_KD,
        .fault_pin       = POS_LOOP_M0_FAULT_PIN,
        .home_pin        = POS_LOOP_M0_HOME_PIN,
        .brake_decel     = POS_LOOP_M0_BRAKE_DECEL,
        .counts_per_unit = POS_LOOP_M0_COUNTS_PER_UNIT,
        .unit_name       = POS_LOOP_M0_UNIT_NAME,
        .pos_min         = POS_LOOP_M0_POS_MIN,
        .pos_max         = POS_LOOP_M0_POS_MAX,
    },
#if PL_MAX_MOTORS >= 2
    /* ---- 电机 1 ---- */
    [1] = {
        .tim_index       = POS_LOOP_M1_TIM_INDEX,
        .tim_ch          = POS_LOOP_M1_TIM_CH,
        .tim_clk_hz      = POS_LOOP_M1_TIM_CLK_HZ,
        .pulse_port_idx  = POS_LOOP_M1_PULSE_PORT_INDEX,
        .pulse_pin       = POS_LOOP_M1_PULSE_PIN,
        .pulse_af        = POS_LOOP_M1_PULSE_AF,
        .dir_pin         = POS_LOOP_M1_DIR_PIN,
        .en_pin          = POS_LOOP_M1_EN_PIN,
        .enc_tim_index   = POS_LOOP_M1_ENC_TIM_INDEX,
        .enc_ch1_port_idx = POS_LOOP_M1_ENC_CH1_PORT_INDEX,
        .enc_ch1_pin     = POS_LOOP_M1_ENC_CH1_PIN,
        .enc_ch1_af      = POS_LOOP_M1_ENC_CH1_AF,
        .enc_ch2_port_idx = POS_LOOP_M1_ENC_CH2_PORT_INDEX,
        .enc_ch2_pin     = POS_LOOP_M1_ENC_CH2_PIN,
        .enc_ch2_af      = POS_LOOP_M1_ENC_CH2_AF,
        .ctrl_timer      = POS_LOOP_M1_CTRL_TIMER,
        .enc_invert      = POS_LOOP_M1_ENC_INVERT,
        .dir_invert      = POS_LOOP_M1_DIR_INVERT,
        .speed_limit     = POS_LOOP_M1_SPEED_LIMIT,
        .move_threshold  = POS_LOOP_M1_MOVE_THRESHOLD,
        .max_accel       = POS_LOOP_M1_MAX_ACCEL,
        .s_curve_jerk    = POS_LOOP_M1_S_CURVE_JERK,
        .ramp_max_speed  = POS_LOOP_M1_RAMP_MAX_SPEED,
        .kp              = POS_LOOP_M1_KP,
        .ki              = POS_LOOP_M1_KI,
        .kd              = POS_LOOP_M1_KD,
        .fault_pin       = POS_LOOP_M1_FAULT_PIN,
        .brake_decel     = POS_LOOP_M1_BRAKE_DECEL,
        .counts_per_unit = POS_LOOP_M1_COUNTS_PER_UNIT,
        .unit_name       = POS_LOOP_M1_UNIT_NAME,
        .pos_min         = POS_LOOP_M1_POS_MIN,
        .pos_max         = POS_LOOP_M1_POS_MAX,
    },
#endif
#if PL_MAX_MOTORS >= 3
    /* ---- 电机 2 ---- */
    [2] = {
        .tim_index       = POS_LOOP_M2_TIM_INDEX,
        .tim_ch          = POS_LOOP_M2_TIM_CH,
        .tim_clk_hz      = POS_LOOP_M2_TIM_CLK_HZ,
        .pulse_port_idx  = POS_LOOP_M2_PULSE_PORT_INDEX,
        .pulse_pin       = POS_LOOP_M2_PULSE_PIN,
        .pulse_af        = POS_LOOP_M2_PULSE_AF,
        .dir_pin         = POS_LOOP_M2_DIR_PIN,
        .en_pin          = POS_LOOP_M2_EN_PIN,
        .enc_tim_index   = POS_LOOP_M2_ENC_TIM_INDEX,
        .enc_ch1_port_idx = POS_LOOP_M2_ENC_CH1_PORT_INDEX,
        .enc_ch1_pin     = POS_LOOP_M2_ENC_CH1_PIN,
        .enc_ch1_af      = POS_LOOP_M2_ENC_CH1_AF,
        .enc_ch2_port_idx = POS_LOOP_M2_ENC_CH2_PORT_INDEX,
        .enc_ch2_pin     = POS_LOOP_M2_ENC_CH2_PIN,
        .enc_ch2_af      = POS_LOOP_M2_ENC_CH2_AF,
        .ctrl_timer      = POS_LOOP_M2_CTRL_TIMER,
        .enc_invert      = POS_LOOP_M2_ENC_INVERT,
        .dir_invert      = POS_LOOP_M2_DIR_INVERT,
        .speed_limit     = POS_LOOP_M2_SPEED_LIMIT,
        .move_threshold  = POS_LOOP_M2_MOVE_THRESHOLD,
        .max_accel       = POS_LOOP_M2_MAX_ACCEL,
        .s_curve_jerk    = POS_LOOP_M2_S_CURVE_JERK,
        .ramp_max_speed  = POS_LOOP_M2_RAMP_MAX_SPEED,
        .kp              = POS_LOOP_M2_KP,
        .ki              = POS_LOOP_M2_KI,
        .kd              = POS_LOOP_M2_KD,
        .fault_pin       = POS_LOOP_M2_FAULT_PIN,
        .brake_decel     = POS_LOOP_M2_BRAKE_DECEL,
        .counts_per_unit = POS_LOOP_M2_COUNTS_PER_UNIT,
        .unit_name       = POS_LOOP_M2_UNIT_NAME,
        .pos_min         = POS_LOOP_M2_POS_MIN,
        .pos_max         = POS_LOOP_M2_POS_MAX,
    },
#endif
#if PL_MAX_MOTORS >= 4
    /* ---- 电机 3 ---- */
    [3] = {
        .tim_index       = POS_LOOP_M3_TIM_INDEX,
        .tim_ch          = POS_LOOP_M3_TIM_CH,
        .tim_clk_hz      = POS_LOOP_M3_TIM_CLK_HZ,
        .pulse_port_idx  = POS_LOOP_M3_PULSE_PORT_INDEX,
        .pulse_pin       = POS_LOOP_M3_PULSE_PIN,
        .pulse_af        = POS_LOOP_M3_PULSE_AF,
        .dir_pin         = POS_LOOP_M3_DIR_PIN,
        .en_pin          = POS_LOOP_M3_EN_PIN,
        .enc_tim_index   = POS_LOOP_M3_ENC_TIM_INDEX,
        .enc_ch1_port_idx = POS_LOOP_M3_ENC_CH1_PORT_INDEX,
        .enc_ch1_pin     = POS_LOOP_M3_ENC_CH1_PIN,
        .enc_ch1_af      = POS_LOOP_M3_ENC_CH1_AF,
        .enc_ch2_port_idx = POS_LOOP_M3_ENC_CH2_PORT_INDEX,
        .enc_ch2_pin     = POS_LOOP_M3_ENC_CH2_PIN,
        .enc_ch2_af      = POS_LOOP_M3_ENC_CH2_AF,
        .ctrl_timer      = POS_LOOP_M3_CTRL_TIMER,
        .enc_invert      = POS_LOOP_M3_ENC_INVERT,
        .dir_invert      = POS_LOOP_M3_DIR_INVERT,
        .speed_limit     = POS_LOOP_M3_SPEED_LIMIT,
        .move_threshold  = POS_LOOP_M3_MOVE_THRESHOLD,
        .max_accel       = POS_LOOP_M3_MAX_ACCEL,
        .s_curve_jerk    = POS_LOOP_M3_S_CURVE_JERK,
        .ramp_max_speed  = POS_LOOP_M3_RAMP_MAX_SPEED,
        .kp              = POS_LOOP_M3_KP,
        .ki              = POS_LOOP_M3_KI,
        .kd              = POS_LOOP_M3_KD,
        .fault_pin       = POS_LOOP_M3_FAULT_PIN,
        .brake_decel     = POS_LOOP_M3_BRAKE_DECEL,
        .counts_per_unit = POS_LOOP_M3_COUNTS_PER_UNIT,
        .unit_name       = POS_LOOP_M3_UNIT_NAME,
        .pos_min         = POS_LOOP_M3_POS_MIN,
        .pos_max         = POS_LOOP_M3_POS_MAX,
    },
#endif
};

static stepper_motor_driver_t   g_pl_motors[PL_MAX_MOTORS];
static stepper_encoder_driver_t g_pl_encoders[PL_MAX_MOTORS];
static stepper_pid_pos_t        g_pl_pids[PL_MAX_MOTORS];
static position_loop_t          g_pl[PL_MAX_MOTORS];
static int                      g_pl_inited[PL_MAX_MOTORS];

static int ensure_inited(int id)
{
    const pl_hw_cfg_t *cfg;
    rt_err_t ret;
    if (id < 0 || id >= PL_MAX_MOTORS) return -1;
    if (g_pl_inited[id]) return 0;
    cfg = &g_pl_cfg[id];

    ret = stepper_motor_driver_init(&g_pl_motors[id],
            tim_from_index(cfg->tim_index), tim_ch_from_index(cfg->tim_ch),
            cfg->tim_clk_hz, gpio_from_index(cfg->pulse_port_idx),
            (rt_uint16_t)cfg->pulse_pin, (rt_uint8_t)cfg->pulse_af,
            (rt_base_t)cfg->dir_pin, (rt_base_t)cfg->en_pin,
            (rt_base_t)cfg->fault_pin, cfg->dir_invert);
    /* 配置制动参数 */
    if (ret == RT_EOK) {
        g_pl_motors[id].brake_decel = (rt_uint32_t)cfg->brake_decel;
        stepper_motor_driver_set_idle_reduce(&g_pl_motors[id], 1, 1000);
    }
    if (ret) { LOG_E("M%d motor failed", id); return -1; }

    ret = stepper_encoder_driver_init(&g_pl_encoders[id],
            tim_from_index(cfg->enc_tim_index),
            gpio_from_index(cfg->enc_ch1_port_idx),
            (rt_uint16_t)cfg->enc_ch1_pin, (rt_uint8_t)cfg->enc_ch1_af,
            gpio_from_index(cfg->enc_ch2_port_idx),
            (rt_uint16_t)cfg->enc_ch2_pin, (rt_uint8_t)cfg->enc_ch2_af);
    if (ret) { LOG_E("M%d enc failed", id); return -1; }

    stepper_pid_pos_init(&g_pl_pids[id],
            cfg->kp / 1000.0f, cfg->ki / 1000.0f, cfg->kd / 1000.0f,
            cfg->speed_limit * 200.0f, cfg->speed_limit, cfg->speed_limit);

    ret = position_loop_init(&g_pl[id], &g_pl_motors[id], &g_pl_encoders[id],
            &g_pl_pids[id], cfg->ctrl_timer, POS_LOOP_CTRL_PERIOD_US,
            cfg->enc_invert, cfg->speed_limit, cfg->move_threshold,
            cfg->max_accel, cfg->s_curve_jerk, cfg->ramp_max_speed,
            cfg->counts_per_unit, cfg->unit_name,
            cfg->pos_min, cfg->pos_max);
    if (ret) { LOG_E("M%d pl init failed", id); return -1; }

    g_pl_inited[id] = 1;
    LOG_I("M%d init OK  kp=%.3f ki=%.3f kd=%.3f", id,
          cfg->kp / 1000.0, cfg->ki / 1000.0, cfg->kd / 1000.0);
    return 0;
}

/* ---- posloop [motor_id] [target] ----
 *   target 带小数点 → 物理单位; 纯整数 → counts */
static int cmd_posloop(int argc, char *argv[])
{
    int id;
    if (argc < 2) {
        for (int i = 0; i < PL_MAX_MOTORS; i++) {
            if (!g_pl_inited[i]) rt_kprintf("M%d: not inited\n", i);
            else {
                float p = position_loop_get_position_physical(&g_pl[i]);
                float t = position_scale_to_physical(&g_pl[i].scale,
                                                     g_pl[i].target_pos);
                rt_kprintf("M%d: run=%s  pos=%.1f%s  target=%.1f%s  "
                           "err=%.1f%s  out=%.0fHz\n",
                           i, position_loop_is_running(&g_pl[i]) ? "Y":"N",
                           (double)p, g_pl[i].scale.unit,
                           (double)t, g_pl[i].scale.unit,
                           (double)(t - p), g_pl[i].scale.unit,
                           (double)g_pl[i].output_freq);
            }
        }
        return 0;
    }
    id = atoi(argv[1]);
    if (ensure_inited(id)) return -1;
    if (argc < 3) {
        float pf = position_loop_get_position_physical(&g_pl[id]);
        LOG_I("M%d: pos=%.1f%s  arrived=%s", id,
              (double)pf, g_pl[id].scale.unit,
              position_loop_is_arrived(&g_pl[id]) ? "Y":"N");
        return 0;
    }
    if (!position_loop_is_running(&g_pl[id]))
        position_loop_start(&g_pl[id]);

    /* 含小数点 → 物理单位; 纯整数 → counts */
    int ret;
    if (strchr(argv[2], '.') != RT_NULL) {
        float target_f = (float)atof(argv[2]);
        LOG_I("M%d -> %.1f%s", id, (double)target_f, g_pl[id].scale.unit);
        ret = position_loop_move_to_physical(&g_pl[id], target_f, 0);
    } else {
        rt_int32_t target = (rt_int32_t)atol(argv[2]);
        LOG_I("M%d -> %ld counts", id, (long)target);
        ret = position_loop_move_to(&g_pl[id], target, 0);
    }
    if (ret == MOTOR_E_OK)
        LOG_I("M%d arrived  pos=%.1f%s", id,
              (double)position_loop_get_position_physical(&g_pl[id]),
              g_pl[id].scale.unit);
    else
        LOG_E("M%d move failed  err=%d", id, ret);
    return ret;
}
MSH_CMD_EXPORT(cmd_posloop, posloop [motor_id] [counts]);

/* ---- posloop_tune <id> <kp> <ki> <kd> ---- */
static int cmd_posloop_tune(int argc, char *argv[])
{
    int id; float kp, ki, kd;
    if (argc < 5) { LOG_E("usage: posloop_tune <id> <kp> <ki> <kd>"); return -1; }
    id = atoi(argv[1]);
    if (id < 0 || id >= PL_MAX_MOTORS || !g_pl_inited[id]) return -1;
    kp = (float)atof(argv[2]); ki = (float)atof(argv[3]); kd = (float)atof(argv[4]);
    stepper_pid_pos_set_gains(&g_pl_pids[id], kp, ki, kd);
    LOG_I("M%d PID -> Kp=%.4f Ki=%.4f Kd=%.4f", id, (double)kp, (double)ki, (double)kd);
    return 0;
}
MSH_CMD_EXPORT(cmd_posloop_tune, posloop_tune id kp ki kd);

/* ---- posloop_start / posloop_stop ---- */
static int cmd_posloop_start(int argc, char *argv[])
{
    int id; if (argc<2) return -1; id=atoi(argv[1]);
    if (ensure_inited(id)) return -1; position_loop_start(&g_pl[id]);
    LOG_I("M%d started", id); return 0;
}
MSH_CMD_EXPORT(cmd_posloop_start, posloop_start id);

static int cmd_posloop_stop(int argc, char *argv[])
{
    int id; if (argc<2) return -1; id=atoi(argv[1]);
    if (id<0||id>=PL_MAX_MOTORS||!g_pl_inited[id]) return -1;
    position_loop_stop(&g_pl[id]); LOG_I("M%d stopped", id); return 0;
}
MSH_CMD_EXPORT(cmd_posloop_stop, posloop_stop id);

/* ---- posloop_home <id> [cw|ccw] [speed_hz] ---- */
static int cmd_posloop_home(int argc, char *argv[])
{
    int id, home_dir = MOTOR_DIR_CW;
    rt_base_t home_pin;
    rt_uint32_t speed = 2000;

    if (argc < 2) {
        rt_kprintf("usage: posloop_home <id> [cw|ccw] [speed_hz]\n");
        return -1;
    }
    id = atoi(argv[1]);
    if (ensure_inited(id)) return -1;
    home_pin = (rt_base_t)g_pl_cfg[id].home_pin;
    if (argc >= 3) {
        if (rt_strcmp(argv[2], "ccw") == 0) home_dir = MOTOR_DIR_CCW;
    }
    if (argc >= 4) speed = (rt_uint32_t)atol(argv[3]);

    LOG_I("M%d homing  pin=%d dir=%s speed=%luHz",
          id, (int)home_pin, home_dir == MOTOR_DIR_CW ? "CW" : "CCW",
          (unsigned long)speed);

    if (!position_loop_is_running(&g_pl[id]))
        position_loop_start(&g_pl[id]);

    int ret = position_loop_home(&g_pl[id], &g_pl_encoders[id],
                                  home_pin, home_dir, speed, 30000);
    if (ret == MOTOR_E_OK)
        LOG_I("M%d home OK  pos=0", id);
    else
        LOG_E("M%d home failed  err=%d", id, ret);
    return ret;
}
MSH_CMD_EXPORT(cmd_posloop_home, posloop_home id [home_pin] [cw|ccw] [speed_hz]);
