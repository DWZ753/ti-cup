#ifndef __PID_H__
#define __PID_H__

typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float target;
    float error;
    float last_error;
    float prev_error;
    float integral;
    float integral_limit;
    float output_limit;
    float output;
} PID_Controller;

/**
 * @brief 初始化 PID 控制器
 * @param pid 控制器指针
 * @param kp/kd/ki PID 参数
 * @param integral_limit 积分限幅
 * @param output_limit 输出限幅
 */
void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit);

/**
 * @brief PID 计算核心
 * @param pid 控制器指针
 * @param current_value 当前反馈值
 * @return 控制输出量
 */
float PID_Compute(PID_Controller *pid, float current_value);

/**
 * @brief 设置目标值
 */
void PID_SetTarget(PID_Controller *pid, float target);

/**
 * @brief 清零积分和误差历史（模式切换时使用）
 */
void PID_Reset(PID_Controller *pid);

#endif
