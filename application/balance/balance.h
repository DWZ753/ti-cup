/**
 * @file    balance.h
 * @brief   球-梁平衡控制模块 API
 *
 * 控制摆杆倾角使钢球稳定在目标位置。
 * 依赖 ZDT 闭环步进电机驱动摆杆。
 *
 * 使用方式：
 *   1. Board_Init() → Balance_Init()  上电初始化（含回零+调平）
 *   2. Balance_SetTarget(0)           设球目标位置（0 = O 点）
 *   3. Balance_Update(pos, vel, conf) 每收到 Pi 球位置时调用（50Hz）
 *   4. Balance_ChassisFF(accel)       每循迹周期调用底盘前馈
 */

#ifndef __BALANCE_H__
#define __BALANCE_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== API ========== */

/**
 * @brief 初始化平衡模块
 *
 * 内部流程：
 *   - ZDT_Motor_Init() 注册 UART
 *   - 使能电机 + 设置 QPos 参数
 *   - 若 BALANCE_SKIP_HOMING=0：触发硬停回零（阻塞 ~3s）
 *   - 若 BALANCE_SKIP_HOMING=1：跳过回零，当前位置 = 原点
 */
void Balance_Init(void);

/**
 * @brief 设置球目标位置
 * @param pos_mm 球偏 O 点的位置 (mm)，正=右，负=左，0=O 点
 */
void Balance_SetTarget(float pos_mm);

/**
 * @brief 球位置更新入口（Pi @50Hz 调用）
 * @param ball_pos_mm  球当前位置 (mm)
 * @param ball_vel_mm_s 球当前速度 (mm/s)
 * @param confidence   0=丢球 1=低置信 2=正常
 * @note  内部执行 PD 控制 + 低通滤波 + 倾角钳位
 */
void Balance_Update(float ball_pos_mm, float ball_vel_mm_s, uint8_t confidence);

/**
 * @brief 底盘加速度前馈
 * @param accel_m_s2 底盘加速度 (m/s²)，加速为正，减速为负
 * @note  每循迹周期调用一次（~10ms），内部叠加到下次 PD 输出
 */
void Balance_ChassisFF(float accel_m_s2);

/**
 * @brief 查询当前摆杆角度指令
 * @return 摆杆倾角 (°)，正=右倾，负=左倾
 */
float Balance_GetAngle(void);

/**
 * @brief 直接设置摆杆倾角（遥控用，绕过 PD）
 * @param angle_deg 目标倾角 (°)，钳位到 [-MAX, +MAX]
 */
void Balance_SetAngle(float angle_deg);

/**
 * @brief 停止平衡控制，摆杆回水平
 */
void Balance_Stop(void);

#endif /* __BALANCE_H__ */
