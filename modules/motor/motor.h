/*
 * 本模块在使用 TB6612 电机驱动芯片时不需要修改
 * @todo 添加使用其他电机驱动芯片的接口
 * @todo 添加使用其他电机驱动芯片的接口
 */
#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "tb6612.h"

#define MOTOR_MAX_PWM_DUTY (TB6612_PWM_PERIOD_COUNT - 1)

// 电机最高转速 RPM（实测）
#define MOTOR_MAX_RPM 300.0f

// 编码器参数：11线 × 1:30减速比 = 330 脉冲/输出轴转
#define MOTOR_ENCODER_PPR                   11
#define MOTOR_ENCODER_GEAR_RATIO            30
#define MOTOR_ENCODER_PULSES_PER_OUTPUT_REV (MOTOR_ENCODER_PPR * MOTOR_ENCODER_GEAR_RATIO)

// 编码器 A 相引脚宏定义
#define MOTOR_ENCODER1_OUT_A_PORT           GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PORT
#define MOTOR_ENCODER1_OUT_A_PIN            GPIO_MOTORs_GPIO_MOTOR1_OUT_A_PIN
#define MOTOR_ENCODER1_OUT_A_IIDX           GPIO_MOTORs_GPIO_MOTOR1_OUT_A_IIDX
#define MOTOR_ENCODER2_OUT_A_PORT           GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PORT
#define MOTOR_ENCODER2_OUT_A_PIN            GPIO_MOTORs_GPIO_MOTOR2_OUT_A_PIN
#define MOTOR_ENCODER2_OUT_A_IIDX           GPIO_MOTORs_GPIO_MOTOR2_OUT_A_IIDX

// 编码器 B 相引脚宏定义（仅读电平，不触发中断）
#define MOTOR_ENCODER1_OUT_B_PORT           GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PORT
#define MOTOR_ENCODER1_OUT_B_PIN            GPIO_MOTORs_GPIO_MOTOR1_OUT_B_PIN
#define MOTOR_ENCODER2_OUT_B_PORT           GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PORT
#define MOTOR_ENCODER2_OUT_B_PIN            GPIO_MOTORs_GPIO_MOTOR2_OUT_B_PIN

// 轮子参数
#define WHEEL_RADIUS_MM                     68.0f
#define WHEEL_CIRCUMFERENCE_MM              (2.0f * 3.1415926f * WHEEL_RADIUS_MM)

// 最大线速度 mm/s = 最高转速 × 周长 / 60
#define MOTOR_MAX_SPEED_MM_S                (MOTOR_MAX_RPM / 60.0f * WHEEL_CIRCUMFERENCE_MM)

/* ========== 测速滤波参数（改这里调响应速度） ========== */

// 测速窗口：多少个控制 Tick（每个 20ms）累加一次速度
// 增大 → 原始精度更高、跳动更小，但测量延迟增大
//   1 = 20ms（原始行为）
//   3 = 60ms（推荐值，30 RPM 时分辨率约 3 RPM）
//   5 = 100ms（极低速时更稳，但 PID 响应慢）
#define MOTOR_SPEED_WINDOW_TICKS             3

// EMA 滤波增益：new_filtered = old + (raw - old) * GAIN
// 取值 0.0~1.0，越小越平滑但滞后越大
//   1.00f = 无滤波（原始值）
//   0.50f = 轻度平滑（滞后 ~2 个窗口周期）
//   0.25f = 中度平滑（滞后 ~4 个窗口周期，推荐值）
//   0.10f = 重度平滑（低速循迹场景）
#define MOTOR_SPEED_EMA_GAIN                 0.25f

/**
 * @brief 电机初始化，将电机置于停止状态
 */
void Motor_Init(void);

/**
 * @brief 设置电机目标线速度（统一接口，正值前进 / 负值后退）
 * @param speed_mm_s 目标线速度 mm/s，范围 [-MOTOR_MAX_SPEED_MM_S, MOTOR_MAX_SPEED_MM_S]
 */
void Motor_SetSpeed(float speed_mm_s);

/**
 * @brief 独立设置左右轮线速度（差速控制）
 * @param left_mm_s  左轮线速度 mm/s，正值前进 / 负值后退
 * @param right_mm_s 右轮线速度 mm/s，正值前进 / 负值后退
 * @note  右轮安装方向与左轮镜像，内部自动处理方向反转
 */
void Motor_SetSpeedLR(float left_mm_s, float right_mm_s);

/**
 * @brief 两路电机同时制动
 */
void Motor_Brake(void);

/**
 * @brief 两路电机同时滑行停止
 */
void Motor_Stop(void);

/**
 * @brief 获取电机1当前转速
 * @return 转速值（RPM）
 */
float Motor_GetEncoder1RPM(void);

/**
 * @brief 获取电机2当前转速
 * @return 转速值（RPM）
 */
float Motor_GetEncoder2RPM(void);

/**
 * @brief 获取电机1当前线速度（原始值，未滤波）
 * @return 线速度（mm/s）
 */
float Motor_GetEncoder1Speed(void);

/**
 * @brief 获取电机2当前线速度（原始值，未滤波）
 * @return 线速度（mm/s）
 */
float Motor_GetEncoder2Speed(void);

/**
 * @brief 获取电机1滤波后线速度
 *
 * 该值经过窗口累积平均 + EMA 低通滤波，已消除编码器量化跳动，
 * 适合直接喂给 PID 控制器。
 *
 * @return 滤波后线速度（mm/s），每 MOTOR_SPEED_WINDOW_TICKS × 20ms 更新一次
 */
float Motor_GetFilteredSpeed1(void);

/**
 * @brief 获取电机2滤波后线速度
 * @return 滤波后线速度（mm/s）
 */
float Motor_GetFilteredSpeed2(void);

/**
 * @brief 获取电机1滤波后转速
 * @return 滤波后转速（RPM）
 */
float Motor_GetFilteredRPM1(void);

/**
 * @brief 获取电机2滤波后转速
 * @return 滤波后转速（RPM）
 */
float Motor_GetFilteredRPM2(void);

/**
 * @brief 获取电机1编码器原始脉冲计数
 * @return 脉冲计数值
 */
int32_t Motor_GetEncoder1Pulse(void);

/**
 * @brief 获取电机2编码器原始脉冲计数
 * @return 脉冲计数值
 */
int32_t Motor_GetEncoder2Pulse(void);

/**
 * @brief 编码器转速更新（PIT 控制中断回调，每 20ms 调用一次）
 *
 * 实现"窗口累积 + EMA 滤波"两阶段测速：
 * - 阶段一（每 tick）：读取脉冲 → 计算差值 → 累加至窗口缓冲区
 * - 阶段二（窗口期满）：从累计差计算原始 RPM → EMA 低通滤波 → 输出滤波值
 *
 * 测速精度和平滑速度分别由 MOTOR_SPEED_WINDOW_TICKS 和
 * MOTOR_SPEED_EMA_GAIN 控制，均在 motor.h 顶部定义。
 *
 * @note PID 建议使用 Motor_GetFilteredSpeed() 而非原始的 Motor_GetEncoderSpeed()
 */
void Motor_TickHandler(void);

/**
 * @brief 清零编码器脉冲、速度及滤波状态
 *
 * 复位范围：原始脉冲、原始 RPM/速度、滤波后 RPM/速度、窗口累积器。
 *
 * @note Motor_Init() 内部调用；急停/重启动时可手动调用以清除历史
 */
void Motor_ResetEncoder(void);

#endif
