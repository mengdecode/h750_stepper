/**
 * @file test_speedloop.c
 * @brief 速度环电机控制 —— 测试例程 & 配置
 * @author hm
 * @version 1.0
 * @date 2026-06-13
 *
 * @copyright Copyright (c) 2026, hm
 *
 * @logs:
 * Date           Version     Author      Description
 * 2026-06-13     v1.0        hm          the first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <stdlib.h>
#include <board.h>

#include "../stepper_drv/stepper_pid_pos.h"
#include "../stepper_drv/stepper_motor_driver.h"
#include "../stepper_drv/stepper_encoder_driver.h"
#include "speed_loop.h"

#define LOG_TAG  "speedloop"
#include <ulog.h>

/* ================================================================
 *  Kconfig → 硬件 映射表
 * ================================================================ */

/* TIM 索引 → 外设指针 */
static TIM_TypeDef* tim_from_index(int idx)
{
    switch (idx) {
        case 1:  return TIM1;
        case 2:  return TIM2;
        case 3:  return TIM3;
        case 4:  return TIM4;
        case 5:  return TIM5;
        case 8:  return TIM8;
        case 12: return TIM12;
        case 15: return TIM15;
        default: return TIM2;
    }
}

/* GPIO 端口索引 → 外设指针 */
static GPIO_TypeDef* gpio_from_index(int idx)
{
    switch (idx) {
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

/* TIM 通道索引 → HAL 通道 */
static rt_uint32_t tim_ch_from_index(int ch)
{
    switch (ch) {
        case 1: return TIM_CHANNEL_1;
        case 2: return TIM_CHANNEL_2;
        case 3: return TIM_CHANNEL_3;
        case 4: return TIM_CHANNEL_4;
        default: return TIM_CHANNEL_1;
    }
}

/* ================================================================
 *  默认配置 (如 Kconfig 未定义则用这些)
 * ================================================================ */

#ifndef SPEED_LOOP_MOTOR_COUNT
#define SPEED_LOOP_MOTOR_COUNT  1
#endif

#ifndef SPEED_LOOP_CTRL_PERIOD_US
#define SPEED_LOOP_CTRL_PERIOD_US  20000
#endif

/* ---- 电机 0 默认配置 ---- */
#ifndef SPEED_LOOP_M0_TIM_INDEX
#define SPEED_LOOP_M0_TIM_INDEX  2       /* TIM2 */
#endif
#ifndef SPEED_LOOP_M0_TIM_CH
#define SPEED_LOOP_M0_TIM_CH  1          /* CH1 */
#endif
#ifndef SPEED_LOOP_M0_TIM_CLK_HZ
#define SPEED_LOOP_M0_TIM_CLK_HZ  200000000UL  /* APB1*2 */
#endif
#ifndef SPEED_LOOP_M0_PULSE_PORT_INDEX
#define SPEED_LOOP_M0_PULSE_PORT_INDEX  0  /* GPIOA */
#endif
#ifndef SPEED_LOOP_M0_PULSE_PIN
#define SPEED_LOOP_M0_PULSE_PIN  0          /* PA0 */
#endif
#ifndef SPEED_LOOP_M0_PULSE_AF
#define SPEED_LOOP_M0_PULSE_AF  1           /* AF1_TIM2 */
#endif
#ifndef SPEED_LOOP_M0_DIR_PIN
#define SPEED_LOOP_M0_DIR_PIN  33           /* GET_PIN(C,1) */
#endif
#ifndef SPEED_LOOP_M0_EN_PIN
#define SPEED_LOOP_M0_EN_PIN  35            /* GET_PIN(C,3) = PC3 */
#endif
#ifndef SPEED_LOOP_M0_ENC_TIM_INDEX
#define SPEED_LOOP_M0_ENC_TIM_INDEX  3      /* TIM3 */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH1_PORT_INDEX
#define SPEED_LOOP_M0_ENC_CH1_PORT_INDEX  2 /* GPIOC */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH1_PIN
#define SPEED_LOOP_M0_ENC_CH1_PIN  6        /* PC6 */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH1_AF
#define SPEED_LOOP_M0_ENC_CH1_AF  2         /* AF2_TIM3 */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH2_PORT_INDEX
#define SPEED_LOOP_M0_ENC_CH2_PORT_INDEX  2 /* GPIOC */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH2_PIN
#define SPEED_LOOP_M0_ENC_CH2_PIN  7        /* PC7 */
#endif
#ifndef SPEED_LOOP_M0_ENC_CH2_AF
#define SPEED_LOOP_M0_ENC_CH2_AF  2         /* AF2_TIM3 */
#endif
#ifndef SPEED_LOOP_M0_CTRL_TIMER
#define SPEED_LOOP_M0_CTRL_TIMER  "timer4"
#endif
#ifndef SPEED_LOOP_M0_STEPS_PER_REV
#define SPEED_LOOP_M0_STEPS_PER_REV  6400     /* 360/1.8 * 32 */
#endif
#ifndef SPEED_LOOP_M0_ENC_COUNTS_PER_REV
#define SPEED_LOOP_M0_ENC_COUNTS_PER_REV  4000 /* 1000 * 4 */
#endif
#ifndef SPEED_LOOP_M0_ENC_INVERT
#define SPEED_LOOP_M0_ENC_INVERT  1           /* 反转编码器方向 */
#endif
#ifndef SPEED_LOOP_M0_SPEED_LIMIT
#define SPEED_LOOP_M0_SPEED_LIMIT  100000
#endif
#ifndef SPEED_LOOP_M0_MOVE_THRESHOLD
#define SPEED_LOOP_M0_MOVE_THRESHOLD  50
#endif
#ifndef SPEED_LOOP_M0_MAX_ACCEL
#define SPEED_LOOP_M0_MAX_ACCEL  500
#endif
#ifndef SPEED_LOOP_M0_KP
#define SPEED_LOOP_M0_KP  500    /* scaled x1000: 500/1000 = 0.5 */
#endif
#ifndef SPEED_LOOP_M0_KI
#define SPEED_LOOP_M0_KI  200    /* scaled x1000: 200/1000 = 0.2 */
#endif
#ifndef SPEED_LOOP_M0_KD
#define SPEED_LOOP_M0_KD  50     /* scaled x1000: 50/1000 = 0.05 */
#endif

/* ---- 电机 1~3: 如未定义, 复制电机0配置为默认 ---- */
/* 这样即使 Kconfig 没配 M1~M3, 代码也能编译通过 */
#ifndef SPEED_LOOP_M1_TIM_INDEX
#define SPEED_LOOP_M1_TIM_INDEX  SPEED_LOOP_M0_TIM_INDEX
#define SPEED_LOOP_M1_TIM_CH  SPEED_LOOP_M0_TIM_CH
#define SPEED_LOOP_M1_TIM_CLK_HZ  SPEED_LOOP_M0_TIM_CLK_HZ
#define SPEED_LOOP_M1_PULSE_PORT_INDEX  SPEED_LOOP_M0_PULSE_PORT_INDEX
#define SPEED_LOOP_M1_PULSE_PIN  SPEED_LOOP_M0_PULSE_PIN
#define SPEED_LOOP_M1_PULSE_AF  SPEED_LOOP_M0_PULSE_AF
#define SPEED_LOOP_M1_DIR_PIN   SPEED_LOOP_M0_DIR_PIN
#define SPEED_LOOP_M1_EN_PIN    SPEED_LOOP_M0_EN_PIN
#define SPEED_LOOP_M1_ENC_TIM_INDEX  SPEED_LOOP_M0_ENC_TIM_INDEX
#define SPEED_LOOP_M1_ENC_CH1_PORT_INDEX  SPEED_LOOP_M0_ENC_CH1_PORT_INDEX
#define SPEED_LOOP_M1_ENC_CH1_PIN  SPEED_LOOP_M0_ENC_CH1_PIN
#define SPEED_LOOP_M1_ENC_CH1_AF  SPEED_LOOP_M0_ENC_CH1_AF
#define SPEED_LOOP_M1_ENC_CH2_PORT_INDEX  SPEED_LOOP_M0_ENC_CH2_PORT_INDEX
#define SPEED_LOOP_M1_ENC_CH2_PIN  SPEED_LOOP_M0_ENC_CH2_PIN
#define SPEED_LOOP_M1_ENC_CH2_AF  SPEED_LOOP_M0_ENC_CH2_AF
#define SPEED_LOOP_M1_CTRL_TIMER  SPEED_LOOP_M0_CTRL_TIMER
#define SPEED_LOOP_M1_STEPS_PER_REV     SPEED_LOOP_M0_STEPS_PER_REV
#define SPEED_LOOP_M1_ENC_COUNTS_PER_REV SPEED_LOOP_M0_ENC_COUNTS_PER_REV
#define SPEED_LOOP_M1_ENC_INVERT        SPEED_LOOP_M0_ENC_INVERT
#define SPEED_LOOP_M1_SPEED_LIMIT    SPEED_LOOP_M0_SPEED_LIMIT
#define SPEED_LOOP_M1_MOVE_THRESHOLD SPEED_LOOP_M0_MOVE_THRESHOLD
#define SPEED_LOOP_M1_MAX_ACCEL      SPEED_LOOP_M0_MAX_ACCEL
#define SPEED_LOOP_M1_KP  SPEED_LOOP_M0_KP
#define SPEED_LOOP_M1_KI  SPEED_LOOP_M0_KI
#define SPEED_LOOP_M1_KD  SPEED_LOOP_M0_KD
#endif

#ifndef SPEED_LOOP_M2_TIM_INDEX
#define SPEED_LOOP_M2_TIM_INDEX  SPEED_LOOP_M0_TIM_INDEX
#define SPEED_LOOP_M2_TIM_CH  SPEED_LOOP_M0_TIM_CH
#define SPEED_LOOP_M2_TIM_CLK_HZ  SPEED_LOOP_M0_TIM_CLK_HZ
#define SPEED_LOOP_M2_PULSE_PORT_INDEX  SPEED_LOOP_M0_PULSE_PORT_INDEX
#define SPEED_LOOP_M2_PULSE_PIN  SPEED_LOOP_M0_PULSE_PIN
#define SPEED_LOOP_M2_PULSE_AF  SPEED_LOOP_M0_PULSE_AF
#define SPEED_LOOP_M2_DIR_PIN   SPEED_LOOP_M0_DIR_PIN
#define SPEED_LOOP_M2_EN_PIN    SPEED_LOOP_M0_EN_PIN
#define SPEED_LOOP_M2_ENC_TIM_INDEX  SPEED_LOOP_M0_ENC_TIM_INDEX
#define SPEED_LOOP_M2_ENC_CH1_PORT_INDEX  SPEED_LOOP_M0_ENC_CH1_PORT_INDEX
#define SPEED_LOOP_M2_ENC_CH1_PIN  SPEED_LOOP_M0_ENC_CH1_PIN
#define SPEED_LOOP_M2_ENC_CH1_AF  SPEED_LOOP_M0_ENC_CH1_AF
#define SPEED_LOOP_M2_ENC_CH2_PORT_INDEX  SPEED_LOOP_M0_ENC_CH2_PORT_INDEX
#define SPEED_LOOP_M2_ENC_CH2_PIN  SPEED_LOOP_M0_ENC_CH2_PIN
#define SPEED_LOOP_M2_ENC_CH2_AF  SPEED_LOOP_M0_ENC_CH2_AF
#define SPEED_LOOP_M2_CTRL_TIMER  SPEED_LOOP_M0_CTRL_TIMER
#define SPEED_LOOP_M2_STEPS_PER_REV     SPEED_LOOP_M0_STEPS_PER_REV
#define SPEED_LOOP_M2_ENC_COUNTS_PER_REV SPEED_LOOP_M0_ENC_COUNTS_PER_REV
#define SPEED_LOOP_M2_ENC_INVERT        SPEED_LOOP_M0_ENC_INVERT
#define SPEED_LOOP_M2_SPEED_LIMIT    SPEED_LOOP_M0_SPEED_LIMIT
#define SPEED_LOOP_M2_MOVE_THRESHOLD SPEED_LOOP_M0_MOVE_THRESHOLD
#define SPEED_LOOP_M2_MAX_ACCEL      SPEED_LOOP_M0_MAX_ACCEL
#define SPEED_LOOP_M2_KP  SPEED_LOOP_M0_KP
#define SPEED_LOOP_M2_KI  SPEED_LOOP_M0_KI
#define SPEED_LOOP_M2_KD  SPEED_LOOP_M0_KD
#endif

#ifndef SPEED_LOOP_M3_TIM_INDEX
#define SPEED_LOOP_M3_TIM_INDEX  SPEED_LOOP_M0_TIM_INDEX
#define SPEED_LOOP_M3_TIM_CH  SPEED_LOOP_M0_TIM_CH
#define SPEED_LOOP_M3_TIM_CLK_HZ  SPEED_LOOP_M0_TIM_CLK_HZ
#define SPEED_LOOP_M3_PULSE_PORT_INDEX  SPEED_LOOP_M0_PULSE_PORT_INDEX
#define SPEED_LOOP_M3_PULSE_PIN  SPEED_LOOP_M0_PULSE_PIN
#define SPEED_LOOP_M3_PULSE_AF  SPEED_LOOP_M0_PULSE_AF
#define SPEED_LOOP_M3_DIR_PIN   SPEED_LOOP_M0_DIR_PIN
#define SPEED_LOOP_M3_EN_PIN    SPEED_LOOP_M0_EN_PIN
#define SPEED_LOOP_M3_ENC_TIM_INDEX  SPEED_LOOP_M0_ENC_TIM_INDEX
#define SPEED_LOOP_M3_ENC_CH1_PORT_INDEX  SPEED_LOOP_M0_ENC_CH1_PORT_INDEX
#define SPEED_LOOP_M3_ENC_CH1_PIN  SPEED_LOOP_M0_ENC_CH1_PIN
#define SPEED_LOOP_M3_ENC_CH1_AF  SPEED_LOOP_M0_ENC_CH1_AF
#define SPEED_LOOP_M3_ENC_CH2_PORT_INDEX  SPEED_LOOP_M0_ENC_CH2_PORT_INDEX
#define SPEED_LOOP_M3_ENC_CH2_PIN  SPEED_LOOP_M0_ENC_CH2_PIN
#define SPEED_LOOP_M3_ENC_CH2_AF  SPEED_LOOP_M0_ENC_CH2_AF
#define SPEED_LOOP_M3_CTRL_TIMER  SPEED_LOOP_M0_CTRL_TIMER
#define SPEED_LOOP_M3_STEPS_PER_REV     SPEED_LOOP_M0_STEPS_PER_REV
#define SPEED_LOOP_M3_ENC_COUNTS_PER_REV SPEED_LOOP_M0_ENC_COUNTS_PER_REV
#define SPEED_LOOP_M3_ENC_INVERT        SPEED_LOOP_M0_ENC_INVERT
#define SPEED_LOOP_M3_SPEED_LIMIT    SPEED_LOOP_M0_SPEED_LIMIT
#define SPEED_LOOP_M3_MOVE_THRESHOLD SPEED_LOOP_M0_MOVE_THRESHOLD
#define SPEED_LOOP_M3_MAX_ACCEL      SPEED_LOOP_M0_MAX_ACCEL
#define SPEED_LOOP_M3_KP  SPEED_LOOP_M0_KP
#define SPEED_LOOP_M3_KI  SPEED_LOOP_M0_KI
#define SPEED_LOOP_M3_KD  SPEED_LOOP_M0_KD
#endif

/* ================================================================
 *  用宏生成每电机的硬件配置和实例
 *
 *  用法: M0_CFG(...) 展开为电机0的配置代码
 * ================================================================ */

/* 展开硬件配置的辅助宏 */
#define SL_XCONFIG(N)                                           \
    {                                                           \
        .tim_index       = SPEED_LOOP_M##N##_TIM_INDEX, \
        .tim_ch          = SPEED_LOOP_M##N##_TIM_CH,    \
        .tim_clk_hz      = SPEED_LOOP_M##N##_TIM_CLK_HZ,\
        .pulse_port_idx  = SPEED_LOOP_M##N##_PULSE_PORT_INDEX,\
        .pulse_pin       = SPEED_LOOP_M##N##_PULSE_PIN, \
        .pulse_af        = SPEED_LOOP_M##N##_PULSE_AF,  \
        .dir_pin         = SPEED_LOOP_M##N##_DIR_PIN,   \
        .en_pin          = SPEED_LOOP_M##N##_EN_PIN,    \
        .enc_tim_index   = SPEED_LOOP_M##N##_ENC_TIM_INDEX,\
        .enc_ch1_port_idx= SPEED_LOOP_M##N##_ENC_CH1_PORT_INDEX,\
        .enc_ch1_pin     = SPEED_LOOP_M##N##_ENC_CH1_PIN,\
        .enc_ch1_af      = SPEED_LOOP_M##N##_ENC_CH1_AF,\
        .enc_ch2_port_idx= SPEED_LOOP_M##N##_ENC_CH2_PORT_INDEX,\
        .enc_ch2_pin     = SPEED_LOOP_M##N##_ENC_CH2_PIN,\
        .enc_ch2_af      = SPEED_LOOP_M##N##_ENC_CH2_AF,\
        .ctrl_timer      = SPEED_LOOP_M##N##_CTRL_TIMER,\
        .steps_per_rev   = SPEED_LOOP_M##N##_STEPS_PER_REV,\
        .enc_counts_per_rev = SPEED_LOOP_M##N##_ENC_COUNTS_PER_REV,\
        .enc_invert      = SPEED_LOOP_M##N##_ENC_INVERT,\
        .speed_limit     = SPEED_LOOP_M##N##_SPEED_LIMIT,\
        .move_threshold  = SPEED_LOOP_M##N##_MOVE_THRESHOLD,\
        .max_accel       = SPEED_LOOP_M##N##_MAX_ACCEL,\
        .kp              = SPEED_LOOP_M##N##_KP,        \
        .ki              = SPEED_LOOP_M##N##_KI,        \
        .kd              = SPEED_LOOP_M##N##_KD,        \
    }

/* 电机硬件配置结构体 */
typedef struct {
    int          tim_index;
    int          tim_ch;
    rt_uint32_t  tim_clk_hz;
    int          pulse_port_idx;
    int          pulse_pin;
    int          pulse_af;
    int          dir_pin;
    int          en_pin;
    int          enc_tim_index;
    int          enc_ch1_port_idx;
    int          enc_ch1_pin;
    int          enc_ch1_af;
    int          enc_ch2_port_idx;
    int          enc_ch2_pin;
    int          enc_ch2_af;
    const char  *ctrl_timer;
    int          steps_per_rev;       /* 电机每转步数          */
    int          enc_counts_per_rev;  /* 编码器每转计数        */
    int          enc_invert;          /* 编码器方向反转        */
    float        speed_limit;
    float        move_threshold;
    float        max_accel;
    int          kp;             /* scaled x1000 */
    int          ki;             /* scaled x1000 */
    int          kd;             /* scaled x1000 */
} motor_hw_cfg_t;

/* ================================================================
 *  多实例数组
 * ================================================================ */

#define SL_MAX_MOTORS  SPEED_LOOP_MOTOR_COUNT

static const motor_hw_cfg_t g_motor_cfg[SL_MAX_MOTORS] = {
    SL_XCONFIG(0),
#if SL_MAX_MOTORS >= 2
    SL_XCONFIG(1),
#endif
#if SL_MAX_MOTORS >= 3
    SL_XCONFIG(2),
#endif
#if SL_MAX_MOTORS >= 4
    SL_XCONFIG(3),
#endif
};

static stepper_motor_driver_t   g_motors[SL_MAX_MOTORS];
static stepper_encoder_driver_t g_encoders[SL_MAX_MOTORS];
static stepper_pid_pos_t        g_pids[SL_MAX_MOTORS];
static speed_loop_t             g_sl[SL_MAX_MOTORS];
static int                      g_inited[SL_MAX_MOTORS];

/* ================================================================
 *  自动初始化
 * ================================================================ */

static int ensure_inited(int motor_id)
{
    const motor_hw_cfg_t *cfg;
    rt_err_t ret;

    if (motor_id < 0 || motor_id >= SL_MAX_MOTORS) {
        LOG_E("motor_id %d out of range [0..%d]", motor_id, SL_MAX_MOTORS - 1);
        return -1;
    }
    if (g_inited[motor_id]) return 0;

    cfg = &g_motor_cfg[motor_id];

    /* 电机驱动初始化 */
    ret = stepper_motor_driver_init(&g_motors[motor_id],
                                    tim_from_index(cfg->tim_index),
                                    tim_ch_from_index(cfg->tim_ch),
                                    cfg->tim_clk_hz,
                                    gpio_from_index(cfg->pulse_port_idx),
                                    (rt_uint16_t)cfg->pulse_pin,
                                    (rt_uint8_t)cfg->pulse_af,
                                    (rt_base_t)cfg->dir_pin,
                                    (rt_base_t)cfg->en_pin,
                                    -1,  /* fault_pin: not used */
                                    0);  /* dir_invert: speed_loop not using */
    if (ret) {
        LOG_E("M%d motor init failed %d", motor_id, ret);
        return -1;
    }

    /* 编码器初始化 */
    ret = stepper_encoder_driver_init(&g_encoders[motor_id],
                                      tim_from_index(cfg->enc_tim_index),
                                      gpio_from_index(cfg->enc_ch1_port_idx),
                                      (rt_uint16_t)cfg->enc_ch1_pin,
                                      (rt_uint8_t)cfg->enc_ch1_af,
                                      gpio_from_index(cfg->enc_ch2_port_idx),
                                      (rt_uint16_t)cfg->enc_ch2_pin,
                                      (rt_uint8_t)cfg->enc_ch2_af);
    if (ret) {
        LOG_E("M%d encoder init failed %d", motor_id, ret);
        return -1;
    }

    /* PID 初始化 (Kp/Ki/Kd 在 Kconfig 中为 x1000 整数, 这里还原为浮点) */
    stepper_pid_pos_init(&g_pids[motor_id],
                         cfg->kp / 1000.0f,
                         cfg->ki / 1000.0f,
                         cfg->kd / 1000.0f,
                         cfg->speed_limit * 200.0f,  /* 积分限幅 */
                         cfg->speed_limit,            /* 输出限幅 */
                         5000.0f);                     /* 误差限幅: 防目标跳变飞车 */

    /* 速度环初始化 */
    ret = speed_loop_init(&g_sl[motor_id],
                          &g_motors[motor_id],
                          &g_encoders[motor_id],
                          &g_pids[motor_id],
                          cfg->ctrl_timer,
                          SPEED_LOOP_CTRL_PERIOD_US,
                          (rt_uint32_t)cfg->steps_per_rev,
                          (rt_uint32_t)cfg->enc_counts_per_rev,
                          cfg->enc_invert,
                          cfg->speed_limit,
                          cfg->move_threshold,
                          cfg->max_accel);
    if (ret) {
        LOG_E("M%d speed_loop init failed %d", motor_id, ret);
        return -1;
    }

    g_inited[motor_id] = 1;
    LOG_I("M%d init OK  kp=%.4f ki=%.4f kd=%.4f  lim=%.0fHz",
          motor_id,
          cfg->kp / 1000.0, cfg->ki / 1000.0, cfg->kd / 1000.0,
          (double)cfg->speed_limit);
    return 0;
}

/* ================================================================
 *  MSH 命令
 * ================================================================ */

/* ---- speedloop [motor_id] [freq_hz] ---- */
static int cmd_speedloop(int argc, char *argv[])
{
    int   motor_id;
    float freq;

    /* 无参数: 显示所有电机状态 */
    if (argc < 2) {
        int i;
        for (i = 0; i < SL_MAX_MOTORS; i++) {
            if (!g_inited[i]) {
                rt_kprintf("M%d: not initialized\n", i);
            } else {
                rt_kprintf("M%d: run=%s  target=%.0f  cur=%.1f  out=%.1fHz  "
                           "enc=%ld  delta=%ld\n",
                           i,
                           speed_loop_is_running(&g_sl[i]) ? "Y" : "N",
                           (double)stepper_pid_pos_get_target(&g_pids[i]),
                           (double)speed_loop_get_current_speed(&g_sl[i]),
                           (double)speed_loop_get_output_freq(&g_sl[i]),
                           (long)speed_loop_get_encoder_position(&g_sl[i]),
                           (long)g_sl[i].enc_delta);
            }
        }
        return 0;
    }

    motor_id = atoi(argv[1]);

    if (argc < 3) {
        /* 只给了 motor_id: 显示该电机状态 */
        if (!g_inited[motor_id]) {
            LOG_W("M%d not initialized", motor_id);
            return -1;
        }
        LOG_I("M%d: run=%s  target=%.0f  cur=%.1f  out=%.1fHz  "
              "ramp=%.1f/%.1f  enc=%ld",
              motor_id,
              speed_loop_is_running(&g_sl[motor_id]) ? "Y" : "N",
              (double)stepper_pid_pos_get_target(&g_pids[motor_id]),
              (double)speed_loop_get_current_speed(&g_sl[motor_id]),
              (double)speed_loop_get_output_freq(&g_sl[motor_id]),
              (double)g_sl[motor_id].ramp_current,
              (double)g_sl[motor_id].ramp_target,
              (long)speed_loop_get_encoder_position(&g_sl[motor_id]));
        return 0;
    }

    if (ensure_inited(motor_id)) return -1;

    freq = (float)atof(argv[2]);

    if (freq == 0) {
        /* 停止 */
        speed_loop_stop(&g_sl[motor_id]);
        LOG_I("M%d stopped", motor_id);
        return 0;
    }

    /* 自动启动 + 设目标 */
    if (!speed_loop_is_running(&g_sl[motor_id]))
        speed_loop_start(&g_sl[motor_id]);

    speed_loop_set_target(&g_sl[motor_id], freq);
    LOG_I("M%d -> %.0f Hz  (%s)",
          motor_id, (double)freq,
          freq > 0 ? "CW" : "CCW");
    return 0;
}
MSH_CMD_EXPORT(cmd_speedloop, speedloop [motor_id] [freq_hz] (0=stop, noarg=status));

/* ---- speedloop_tune <motor_id> <kp> <ki> <kd> ---- */
static int cmd_speedloop_tune(int argc, char *argv[])
{
    int   motor_id;
    float kp, ki, kd;

    if (argc < 5) {
        LOG_E("usage: speedloop_tune <motor_id> <kp> <ki> <kd>");
        return -1;
    }

    motor_id = atoi(argv[1]);
    if (motor_id < 0 || motor_id >= SL_MAX_MOTORS || !g_inited[motor_id]) {
        LOG_E("M%d not inited", motor_id);
        return -1;
    }

    kp = (float)atof(argv[2]);
    ki = (float)atof(argv[3]);
    kd = (float)atof(argv[4]);

    stepper_pid_pos_set_gains(&g_pids[motor_id], kp, ki, kd);
    LOG_I("M%d PID -> Kp=%.4f Ki=%.4f Kd=%.4f",
          motor_id, (double)kp, (double)ki, (double)kd);
    return 0;
}
MSH_CMD_EXPORT(cmd_speedloop_tune, speedloop_tune motor_id kp ki kd);

/* ---- speedloop_start <motor_id> ---- */
static int cmd_speedloop_start(int argc, char *argv[])
{
    int motor_id;
    if (argc < 2) { LOG_E("usage: speedloop_start <motor_id>"); return -1; }
    motor_id = atoi(argv[1]);
    if (ensure_inited(motor_id)) return -1;
    speed_loop_start(&g_sl[motor_id]);
    LOG_I("M%d started", motor_id);
    return 0;
}
MSH_CMD_EXPORT(cmd_speedloop_start, speedloop_start motor_id);

/* ---- speedloop_stop <motor_id> ---- */
static int cmd_speedloop_stop(int argc, char *argv[])
{
    int motor_id;
    if (argc < 2) { LOG_E("usage: speedloop_stop <motor_id>"); return -1; }
    motor_id = atoi(argv[1]);
    if (motor_id < 0 || motor_id >= SL_MAX_MOTORS || !g_inited[motor_id]) {
        LOG_E("M%d not inited", motor_id);
        return -1;
    }
    speed_loop_stop(&g_sl[motor_id]);
    LOG_I("M%d stopped", motor_id);
    return 0;
}
MSH_CMD_EXPORT(cmd_speedloop_stop, speedloop_stop motor_id);
