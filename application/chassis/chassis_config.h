/**
 * @file    chassis_config.h
 * @brief   底盘控制可调参数 — 所有需要上赛道后实测调整的参数集中在此
 * @note    修改此文件后重新编译即可生效，无需改动 main.c
 */

#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__

/* ========== 控制周期（ms） ========== */

#define TRACKING_CTRL_DT_MS    10    // 循迹 PID 计算周期

/* ========== 固定速度（mm/s，开环基准） ========== */

#define SPEED_STRAIGHT          1000.0f  // 直道速度
#define SPEED_ARC               600.0f   // 弯道速度

// |舵机输出| 超过此值判定为弯道，切换到 SPEED_ARC
#define SERVO_CURVE_THRESHOLD   25

/* ========== 差速增益 ========== */

// 舵机角度 × 增益 = 两轮速度差 (mm/s)
// 值越大差速转向越猛，值越小越柔和
#define ARC_DIFF_GAIN           8.0f

/* ========== 循迹 PID ========== */

// position ∈ [-1, 1]，OUT_LIMIT=100 对应满偏
#define TRACKING_KP             100.0f
#define TRACKING_KI             0.5f
#define TRACKING_KD             0.0f   // 用 slew rate 替代 D 项
#define TRACKING_INT_LIMIT      20.0f
#define TRACKING_OUT_LIMIT      100.0f

// slew rate: 每次调用最大变化量，替代微分项防止震荡
#define TRACKING_SLEW_MAX       4

/* ========== 丢线防抖 ========== */

#define LINE_LOST_DEBOUNCE      5      // 连续丢线帧数（弧线段退出用）

/* ========== 停车检测 ========== */

// A 点停车横线：所有传感器同时压黑线（mask == 0x00）
// 连续检测 N 次确认停车
#define STOP_LINE_DEBOUNCE      3

/* ========== 遥测周期（0=关闭） ========== */

#define TELEMETRY_DT_MS         0      // UART 调试输出周期，0=关闭

#endif /* __CHASSIS_CONFIG_H__ */
