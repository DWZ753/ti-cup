/**
 * @file    chassis_config.h
 * @brief   底盘控制可调参数 — 所有需要上赛道后实测调整的参数集中在此
 * @note    修改此文件后重新编译即可生效，无需改动 main.c
 */

#ifndef __CHASSIS_CONFIG_H__
#define __CHASSIS_CONFIG_H__

/* ========== 控制周期（ms） ========== */

#define TRACKING_CTRL_DT_MS    10    // 循迹 PID 计算周期

/* ========== 渐加速/渐减速 (mm/s²) ========== */

#define TRACK_SPEED_RAMP_UP     300.0f  // 加速斜率
#define TRACK_SPEED_RAMP_DOWN   500.0f  // 减速斜率（刹车要快）

/* ========== 速度 + 差速（各任务独立可调） ========== */

// |舵机输出| 超过此值判定为弯道，切换到弯道速度
#define SERVO_CURVE_THRESHOLD   25

/* 要求2：纯循迹 */
#define TASK2_SPEED_STRAIGHT    800.0f
#define TASK2_SPEED_ARC         800.0f
#define TASK2_DIFF_GAIN         7.8f

/* 要求4：AB段 循迹+平衡（首次转弯停止） */
#define TASK4_SPEED_STRAIGHT    600.0f
#define TASK4_SPEED_ARC         600.0f
#define TASK4_DIFF_GAIN         7.3f

/* 要求5：一圈+O点 循迹+平衡 */
#define TASK5_SPEED_STRAIGHT    600.0f
#define TASK5_SPEED_ARC         600.0f
#define TASK5_DIFF_GAIN         7.3f

/* 要求6：一圈+任意点 循迹+平衡 */
#define TASK6_SPEED_STRAIGHT    600.0f
#define TASK6_SPEED_ARC         600.0f
#define TASK6_DIFF_GAIN         7.3f

/* ========== 传感器权重（索引 0..7，左→右） ========== */

// 每个传感器在加权平均中的贡献权重
// 权重越大该位置对结果影响越大；中心高、边缘低 → 边缘柔和
// 传感器阵列:     0    1    2    3    4    5    6    7
//                左 ← ← ← 中心 → → → 右
#define TRACKING_W0             0.6f
#define TRACKING_W1             0.65f
#define TRACKING_W2             0.95f
#define TRACKING_W3             1.0f
#define TRACKING_W4             1.0f
#define TRACKING_W5             0.95f
#define TRACKING_W6             0.65f
#define TRACKING_W7             0.6f

/* ========== 循迹位置曲线 ========== */

// 指数: 1=线性, 2=平方, 3=立方（与权重叠加使用，设为 1 则仅靠权重控制）
#define TRACKING_CURVE_POWER    1

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
