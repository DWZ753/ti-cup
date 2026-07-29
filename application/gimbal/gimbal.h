/**
 * @file    gimbal.h
 * @brief   单轴云台控制模块（pitch 轴，ZDT 闭环步进电机）
 *
 * 机械结构：云台搭载摄像头 + 机械臂（末端电磁铁），共用 pitch 轴。
 * 默认状态机械臂触地，以此为原点(0°)。
 *
 * 安全保护：
 *   - 上电硬停回零：电机下转 → 机械臂碰地面 → 电流尖峰 → 自动停止 → 设零点
 *   - 过流保护：ZDT 驱动板固件层 OCP，MSPM0 侧设保守阈值
 *   - 角度钳位：[0, GIMBAL_MAX_ANGLE_DEG]，防止上抬过高扯线
 */

#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== 可调参数（待上赛道实测调优） ========== */

/** 最大向上角度（°），防止看天 + 扯线 */
#define GIMBAL_MAX_ANGLE_DEG    45.0f

/** 16 细分下 1 圈 = 3200 pulse，1° 对应 pulse 数 */
#define GIMBAL_PULSE_PER_DEG    (3200.0f / 360.0f)

/** 硬停回零：碰撞检测速度(RPM)，慢速减少撞地冲击 */
#define GIMBAL_ZERO_VEL         30

/** 硬停回零：碰撞检测电流(mA) */
#define GIMBAL_ZERO_CUR_MA      500

/** 硬停回零：碰撞检测时间(ms) */
#define GIMBAL_ZERO_TIME_MS     50

/** 过流保护阈值(mA) */
#define GIMBAL_OCP_MA           800

/** 过流触发时间(ms) */
#define GIMBAL_OCP_TIME_MS      200

/** 正常工作速度(RPM) */
#define GIMBAL_WORK_VEL         200

/* ========== API ========== */

/**
 * @brief 云台初始化 + 上电自动归零
 *
 * 内部流程：
 *   1. ZDT_Motor_Init() — 注册 UART，等待驱动板上电
 *   2. 配置硬停回零参数（速度/电流/时间）
 *   3. 配置过流保护阈值
 *   4. 使能电机
 *   5. 触发硬停回零 → 电机下转 → 碰地面 → 电流尖峰 → 自动停止
 *   6. 等待回零完成 → Reset_CurPos_To_Zero()
 *
 * @note  阻塞调用，约需 2~5 秒（取决于起始位置和回零速度）
 */
void Gimbal_Init(void);

/**
 * @brief 设置云台 pitch 角度
 * @param angle_deg 目标角度（°），0 = 机械臂触地，正值 = 向上抬起
 * @note  内部钳位到 [0, GIMBAL_MAX_ANGLE_DEG]
 *        使用 ZDT 快速位置模式（QPos），非阻塞
 */
void Gimbal_SetAngle(float angle_deg);

/**
 * @brief 查询云台是否到达目标位置
 * @retval true  已到位（位置误差在窗口内）
 * @retval false 运动中
 */
bool Gimbal_IsIdle(void);

/**
 * @brief 紧急停止云台运动
 */
void Gimbal_Stop(void);

/**
 * @brief 手动触发回零（运行时重新校准）
 * @note  阻塞调用，同 Gimbal_Init 中的回零流程
 */
void Gimbal_Rehome(void);

#endif
