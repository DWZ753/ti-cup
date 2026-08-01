/**
 * @file    tracking.h
 * @brief   循迹模块 — 灰度传感器位置解算 + 循迹控制 + 停车检测
 *
 * 所有循迹相关状态和逻辑封装在此模块，main.c 仅通过 API 调度。
 */

#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== 传感器参数 ========== */

#define SENSOR_COUNT    8
#define SENSOR_CENTER   ((SENSOR_COUNT - 1) / 2.0f)

/* ========== 位置解算（纯函数，无状态） ========== */

/**
 * @brief 根据灰度传感器原始读数计算黑线位置
 * @param mask 8 位掩码，bit[i]=1 白/未压线，bit[i]=0 黑/压线
 * @return [-1.0f, +1.0f] 负左正右，0 居中；99.0f = 丢线
 */
float Tracking_CalcPosition(uint8_t mask);

/* ========== 循迹控制 API ========== */

void  Tracking_Init(void);                              // 初始化 PID + 状态
void  Tracking_Start(void);                             // 启动循迹（复位 PID）
void  Tracking_Stop(void);                              // 停车（刹车 + 舵机回中 + PID 复位）
bool  Tracking_IsRunning(void);                         // 查询是否在循迹中
void  Tracking_Update(uint32_t now_ms, uint8_t mask);   // 自适应速度 + 循迹 PID + 停车检测
void  Tracking_SetSpeedParams(float straight, float arc, float diff_gain);  // 运行时切换速度/差速参数
void  Tracking_SetStopOnCurve(bool enable);                                  // AB段：首次转弯自动停车
void  Tracking_SetSpeedRamp(bool enable);                                    // 渐加速/渐减速开关

#endif
