/**
 * @file    tracking.h
 * @brief   循迹模块 — 灰度传感器位置解算
 *
 * 不依赖 IMU / motor / servo，只依赖灰度 mask 输入。
 */

#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

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

#endif
