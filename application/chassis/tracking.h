/**
 * @file    tracking.h
 * @brief   循迹模块 — 位置解算 + 直角弯检测
 *
 * 检测原理：
 *   接近直角弯时，黑线快速偏移到传感器阵列边缘，|position| 瞬间跳变到
 *   ~1.0。此时方向信号清晰（偏右=右转，偏左=左转），比"全白丢线"更早、
 *   更可靠。全白丢线作为后备触发。
 *
 * 不依赖 IMU / motor / servo，只依赖灰度 mask 输入。
 */

#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ========== 传感器参数 ========== */

#define SENSOR_COUNT    8
#define SENSOR_CENTER   ((SENSOR_COUNT - 1) / 2.0f)

/* ========== 直角检测可调参数 ========== */

/**
 * 位置跳变阈值：|position| 超过此值视为接近直角弯
 * 发生在丢线之前，方向信号最清晰
 */
#define TRACKING_CORNER_POS_THRESHOLD  0.8f

/**
 * 连续满足触发条件的帧数（位置跳变和丢线共用同一计数器）
 * @20ms 周期：2 帧 = 40ms，兼顾响应速度和抗噪
 */
#define TRACKING_CORNER_DEBOUNCE  2

/**
 * 转弯中连续读到多少次黑线确认已重新捕获
 */
#define TRACKING_REACQUIRE_CNT  3

/* ========== 类型定义 ========== */

typedef enum {
	TURN_UNKNOWN = 0,   /**< 无法判断，零初始化默认值 */
	TURN_LEFT    = 1,   /**< 左转 90° */
	TURN_RIGHT   = 2,   /**< 右转 90° */
} tracking_turn_dir_e;

/* ========== 位置解算（纯函数，无状态） ========== */

/**
 * @brief 根据灰度传感器原始读数计算黑线位置
 * @param mask 8 位掩码，bit[i]=1 白/未压线，bit[i]=0 黑/压线
 * @return [-1.0f, +1.0f] 负左正右，0 居中；99.0f = 丢线
 */
float Tracking_CalcPosition(uint8_t mask);

/* ========== 直角弯检测（有状态，依赖 Update 驱动） ========== */

/**
 * @brief 每控制周期调用一次，喂入灰度掩码
 */
void Tracking_Update(uint8_t mask);

/**
 * @brief 是否应进入转弯模式
 * @retval true  位置跳变或丢线达到 debounce 阈值
 */
bool Tracking_IsCorner(void);

/**
 * @brief 获取转弯方向
 * @note  优先级：SetTurnDirection() > 跳变时的位置符号 > UNKNOWN
 *        因为跳变在丢线之前发生，此时位置信号是干净的
 */
tracking_turn_dir_e Tracking_GetTurnDirection(void);

/**
 * @brief 强制覆盖转弯方向（上层根据赛道布局预设）
 */
void Tracking_SetTurnDirection(tracking_turn_dir_e dir);

/**
 * @brief 转弯中是否已重新捕获黑线（可退出转弯）
 */
bool Tracking_IsLineReacquired(void);

/**
 * @brief 获取当前归一化位置
 * @return [-1.0, +1.0] 或 99.0f
 */
float Tracking_GetPosition(void);

/**
 * @brief 退出转弯后重置状态机
 */
void Tracking_Reset(void);

#endif
