/*
 * 步进电机开环控制
 *
 *   stepol <Hz>        → 自动初始化+启动+设频率
 *   stepol 0           → 停止
 *   stepol_dir cw|ccw  → 换向
 */

#include <rtthread.h>
#include <board.h>
#include <stdlib.h>
#include "stepper_openloop.h"

#define LOG_TAG  "ol"
#include <ulog.h>

#define M1_TIM        TIM2
#define M1_TIM_CH     TIM_CHANNEL_1
#define M1_PULSE_PORT GPIOA
#define M1_PULSE_PIN  GPIO_PIN_0
#define M1_PULSE_AF   GPIO_AF1_TIM2
#define M1_DIR_PIN    GET_PIN(C, 1)
#define M1_EN_PIN     GET_PIN(C, 3)

#define CTRL_TIMER   "timer4"
#define CTRL_PERIOD  20000
#define MAX_ACCEL    200.0f

static stepper_motor_driver_t g_motor;
static stepper_openloop_t g_ol;
static int                g_inited = 0;

static int ensure_inited(void)
{
    if (g_inited) return 0;
    if (stepper_motor_driver_init(&g_motor,
                           M1_TIM, M1_TIM_CH,
                           HAL_RCC_GetPCLK1Freq() * 2,
                           M1_PULSE_PORT, M1_PULSE_PIN, M1_PULSE_AF,
                           M1_DIR_PIN, M1_EN_PIN, 0)) return -1;
    if (stepper_openloop_init(&g_ol, &g_motor, CTRL_TIMER,
                              CTRL_PERIOD, MAX_ACCEL)) return -1;
    g_inited = 1;
    return 0;
}

/* ---- stepol <Hz> ---- */
static int cmd_stepol(int argc, char *argv[])
{
    float f;
    if (argc < 2) {
        LOG_I("run=%s  dir=%s  tgt=%.0f  cur=%.0f Hz",
              stepper_openloop_is_running(&g_ol) ? "Y" : "N",
              g_ol.direction == MOTOR_DIR_CW ? "CW" : "CCW",
              (double)g_ol.target_freq, (double)g_ol.current_freq);
        return 0;
    }
    if (ensure_inited()) return -1;

    f = (float)atof(argv[1]);

    if (f == 0) {
        stepper_openloop_stop(&g_ol);
        LOG_I("stopped");
        return 0;
    }

    /* 自动启动 + 设定频率 */
    if (!stepper_openloop_is_running(&g_ol)) {
        stepper_openloop_start(&g_ol);
        stepper_openloop_set_direction(&g_ol, MOTOR_DIR_CW);
    }
    stepper_openloop_set_freq(&g_ol, f);

    LOG_I("-> %.0f Hz  (cur=%.0f)", (double)f, (double)g_ol.current_freq);
    return 0;
}
MSH_CMD_EXPORT(cmd_stepol, stepol freq(Hz) 0=stop);

/* ---- stepol_dir cw|ccw ---- */
static int cmd_stepol_dir(int argc, char *argv[])
{
    if (argc < 2) return -1;
    ensure_inited();
    stepper_openloop_set_direction(&g_ol,
        rt_strcmp(argv[1], "ccw") == 0 ? MOTOR_DIR_CCW : MOTOR_DIR_CW);
    return 0;
}
MSH_CMD_EXPORT(cmd_stepol_dir, stepol_dir cw or ccw);
