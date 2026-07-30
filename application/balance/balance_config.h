/**
 * @file    balance_config.h
 * @brief   球-梁平衡控制 — 可调参数集中配置
 */

#ifndef __BALANCE_CONFIG_H__
#define __BALANCE_CONFIG_H__

/* ========== ZDT 电机 ========== */

#define BALANCE_MOTOR_ID          1
#define BALANCE_PULSE_PER_DEG     (3200.0f / 360.0f)  // 16细分
#define BALANCE_WORK_VEL          200                  // 工作转速(RPM)
#define BALANCE_MAX_ANGLE_DEG     10.0f                // 摆杆最大倾角(°)

/* 合页未到，跳过机械回零；设为 0 启用硬停回零 */
#define BALANCE_SKIP_HOMING       1

/* 硬停回零参数（BALANCE_SKIP_HOMING=0 时生效） */
#define BALANCE_ZERO_VEL          30
#define BALANCE_ZERO_CUR_MA       500
#define BALANCE_ZERO_TIME_MS      50

/* ========== 球位置 PD ========== */

/* °/mm，位置误差→倾角 */
#define BALANCE_KP                0.3f
/* °/(mm/s)，球速→反向倾角 */
#define BALANCE_KD                0.5f

/* ========== 底盘前馈 ========== */

/* °/(m/s²)，底盘加速度→补偿倾角 */
#define FF_ACCEL_GAIN             5.8f

/* ========== 低通滤波器 ========== */

#define POS_FILTER_ALPHA          0.3f   // 位置（越大越灵敏）
#define VEL_FILTER_ALPHA          0.2f   // 速度（越大噪声越大）

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200    // 超时判丢球

/* ========== 静态平衡序列（要求3，开环时序） ========== */

/* 各阶段：{倾角(°), 持续时间(ms)}，最后一段 angle=0 表示完成 */
#define STATIC_SEQ_LEN            7

/* 倾角 >0 = 球往 +x 滚，<0 = 球往 -x 滚 */
#define STATIC_SEQ_ANGLE_0        5.0f   // 加速向 +5cm
#define STATIC_SEQ_TIME_0         500
#define STATIC_SEQ_ANGLE_1        -4.0f  // 减速
#define STATIC_SEQ_TIME_1         200
#define STATIC_SEQ_ANGLE_2        0.0f   // 停在 +5cm
#define STATIC_SEQ_TIME_2         2000
#define STATIC_SEQ_ANGLE_3        -5.0f  // 加速向 -5cm
#define STATIC_SEQ_TIME_3         500
#define STATIC_SEQ_ANGLE_4        4.0f   // 减速
#define STATIC_SEQ_TIME_4         200
#define STATIC_SEQ_ANGLE_5        0.0f   // 停在 -5cm
#define STATIC_SEQ_TIME_5         2000
#define STATIC_SEQ_ANGLE_6        0.0f   // 结束
#define STATIC_SEQ_TIME_6         0

#endif /* __BALANCE_CONFIG_H__ */
