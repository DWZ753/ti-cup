/**
 * @file    chassis_config.h
 * @brief   底盘控制可调参数 — 所有需要上赛道后实测调整的参数集中在此
 * @note    修改此文件后重新编译即可生效，无需改动 chassis.c
 */

#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__

/* ========== 控制周期（ms） ========== */

#define IMU_UPDATE_DT_MS       10    // IMU 姿态更新周期
#define ANGLE_PID_DT_MS        80    // 角度环 PID 计算周期
#define TELEMETRY_DT_MS        100   // UART 遥测输出周期

/* ========== 段切换检测 ========== */

#define LINE_LOST_DEBOUNCE      5    // 丢线防抖次数（弧线段用）

/* ========== 固定速度（mm/s，开环基准） ========== */

#define SPEED_ARC               1000.0f   // 弧线段固定速度
#define SPEED_STRAIGHT          1000.0f   // 直线段固定速度

/* ========== 差速增益 ========== */

// 舵机角度 × 增益 = 两轮速度差 (mm/s)
// 值越大差速转向越猛，值越小越柔和
#define ARC_DIFF_GAIN           7.3f

/* ========== 循迹 PID（弧线段使用） ========== */

// position ∈ [-1, 1]，OUT_LIMIT=100 对应满偏
#define TRACKING_KP             100.0f
#define TRACKING_KI             0.5f
#define TRACKING_KD             0.0f   // 用 slew rate 替代 D 项
#define TRACKING_INT_LIMIT      20.0f
#define TRACKING_OUT_LIMIT      100.0f

// slew rate: 每次调用最大变化量，替代微分项防止震荡
#define TRACKING_SLEW_MAX       4

#endif /* __CHASSIS_CONFIG_H__ */
