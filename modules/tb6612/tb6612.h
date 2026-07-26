#ifndef __TB6612_H__
#define __TB6612_H__

#include <stdint.h>

// 以下常量供外部使用（motor.h 等），引脚宏已移至 tb6612.c
#define TB6612_PWM_PERIOD_COUNT  10000

/**
 * @brief 初始化 TB6612 电机驱动模块，启动 PWM 定时器
 */
void TB6612_Init(void);

/**
 * @brief 将 PWM 占空比限制在有效范围内
 * @param period_count 原始占空比计数值
 * @return 限制后的占空比计数值 [0, TB6612_PWM_PERIOD_COUNT]
 */
uint32_t TB6612_LimitPWM(uint32_t period_count);

/**
 * @brief A 电机正转
 * @param duty PWM 占空比计数值，越大转速越快
 */
void TB6612_A_Forward(uint32_t duty);

/**
 * @brief A 电机反转
 * @param duty PWM 占空比计数值，越大转速越快
 */
void TB6612_A_Backward(uint32_t duty);

/**
 * @brief A 电机制动（两路控制引脚均置高）
 */
void TB6612_A_Brake(void);

/**
 * @brief A 电机停止（两路控制引脚均置低，滑行停止）
 */
void TB6612_A_Stop(void);

/**
 * @brief B 电机正转
 * @param duty PWM 占空比计数值，越大转速越快
 */
void TB6612_B_Forward(uint32_t duty);

/**
 * @brief B 电机反转
 * @param duty PWM 占空比计数值，越大转速越快
 */
void TB6612_B_Backward(uint32_t duty);

/**
 * @brief B 电机制动（两路控制引脚均置高）
 */
void TB6612_B_Brake(void);

/**
 * @brief B 电机停止（两路控制引脚均置低，滑行停止）
 */
void TB6612_B_Stop(void);

#endif
