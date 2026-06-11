/*
 * 位置式 PID 控制器 —— 纯算法模块，无任何硬件依赖
 *
 * 公式: u(k) = Kp*e(k) + Ki*Σe + Kd*(e(k)-e(k-1))
 *
 * 与增量式的区别:
 *   - 增量式: 输出 Δu, 由调用者累加
 *   - 位置式: 输出绝对值 u(k), 直接作为控制量
 *
 * 使用方式（多实例安全）:
 *   stepper_pid_pos_t pid;
 *   stepper_pid_pos_init(&pid, 0.1, 0.01, 0, 20000, 20000);
 *   stepper_pid_pos_set_target(&pid, 5000);
 *   float out = stepper_pid_pos_calculate(&pid, actual_value);
 *   // out 即目标频率 (Hz)，直接传给电机
 */

#ifndef STEPPER_PID_POS_H__
#define STEPPER_PID_POS_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float target;           /* 目标值 (setpoint)              */
    float actual;           /* 最近一次反馈值                  */
    float err;              /* e(k): 当前误差 (已限幅)        */
    float err_last;         /* e(k-1): 上一次误差              */
    float kp, ki, kd;       /* PID 增益                       */
    float integral_acc;     /* 积分累加 Σe                    */
    float integral_limit;   /* 积分限幅 [-limit, +limit]      */
    float output_limit;     /* 输出限幅 [-limit, +limit]      */
    float error_limit;      /* 误差限幅 (防目标跳变飞车)       */
    float output;           /* 最近一次输出值 u(k)             */
} stepper_pid_pos_t;

/**
 * 初始化位置式 PID 实例
 * @param pid             实例指针（调用者分配）
 * @param kp, ki, kd      PID 增益
 * @param integral_limit  积分限幅（0 = 不限幅）
 * @param output_limit    输出限幅（通常设为速度上限 Hz）
 * @param error_limit     误差限幅（0 = 不限幅, 防目标突变飞车）
 */
void stepper_pid_pos_init(stepper_pid_pos_t *pid,
                          float kp, float ki, float kd,
                          float integral_limit,
                          float output_limit,
                          float error_limit);

/** 重置内部状态（误差清零，积分清零，输出清零） */
void stepper_pid_pos_reset(stepper_pid_pos_t *pid);

/** 设置目标值 */
void stepper_pid_pos_set_target(stepper_pid_pos_t *pid, float target);

/** 获取目标值 */
float stepper_pid_pos_get_target(const stepper_pid_pos_t *pid);

/** 获取最近一次输出 */
float stepper_pid_pos_get_output(const stepper_pid_pos_t *pid);

/** 更新 PID 增益（运行时调参） */
void stepper_pid_pos_set_gains(stepper_pid_pos_t *pid, float kp, float ki, float kd);

/** 更新积分限幅（运行时调参） */
void stepper_pid_pos_set_integral_limit(stepper_pid_pos_t *pid, float limit);

/** 更新输出限幅（运行时调参） */
void stepper_pid_pos_set_output_limit(stepper_pid_pos_t *pid, float limit);

/**
 * 执行一次位置式 PID 计算
 * @param actual  当前实测值（反馈）
 * @return        绝对输出值 u(k)，已限幅
 *
 * 公式: u = Kp*e(k) + Ki*Σe + Kd*(e(k)-e(k-1))
 * 其中 e(k) = clamp(target - actual, ±error_limit)
 */
float stepper_pid_pos_calculate(stepper_pid_pos_t *pid, float actual);

#ifdef __cplusplus
}
#endif
#endif
