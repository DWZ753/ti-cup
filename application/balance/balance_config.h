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

/* 暂时跳过回零，上电直接转 */
#define BALANCE_SKIP_HOMING       0

/* 硬停回零参数 */
#define BALANCE_ZERO_VEL          30
#define BALANCE_ZERO_CUR_MA       800   // 堵转检测电流(mA)，太低会误触发
#define BALANCE_ZERO_TIME_MS      100   // 堵转检测时间(ms)，太短会启动误触发
#define BALANCE_ZERO_DIR          1     // 回零方向: 0=CW, 1=CCW（先试往上碰限位）

/* 过流保护（驱动板硬件 OCP，须在使能前设置） */
#define BALANCE_OCP_MA            800   // OCP 阈值(mA)
#define BALANCE_OCP_TIME_MS       200   // OCP 触发时间(ms)

/**
 * 回零后摆杆从地面抬起到水平位置的偏移角度 (°)。
 * 正值 = 上抬，负值 = 下压。
 * ⚠️ 必须实测标定：先设 -25°. CCW方向（回零反方向），上电看是否水平，再微调。
 */
#define BALANCE_HOME_OFFSET_DEG   -45.5f

/* ========== 球位置 PD ========== */

/* °/mm，位置误差→倾角 */
#define BALANCE_KP                0.3f
/* °/(mm/s)，球速→反向倾角 */
#define BALANCE_KD                0.5f

/* ========== 底盘前馈 ========== */

/* °/(m/s²)，底盘加速度→补偿倾角
   理论值 ≈ 1/g × 180/π ≈ 5.8
   实测调节：
     - 车加速时球往后滚 → 加大此值（如 6.5 ~ 8.0）
     - 车加速时球往前冲 → 减小此值（如 3.0 ~ 5.0）
     - 刹车时球往前滚 → 加大；球往后靠 → 减小 */
#define FF_ACCEL_GAIN             7.5f

/* m/s²，加速度死区：低于此值判定为匀速，摆杆回水平 */
#define FF_ACCEL_DEADZONE         0.01f

/* 加速度低通系数（0~1）：越大越灵敏但越抖，越小越平滑但滞后 */
#define FF_ACCEL_FILTER           0.5f

/* ========== 低通滤波器 ========== */

#define POS_FILTER_ALPHA          0.3f   // 位置（越大越灵敏）
#define VEL_FILTER_ALPHA          0.2f   // 速度（越大噪声越大）

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200    // 超时判丢球

/* ========== 静态平衡序列（要求3，开环时序） ========== */

/* 各阶段：{倾角(°), 持续时间(ms)}，最后一段 angle=0 表示完成 */
#define STATIC_SEQ_LEN            7

/* 倾角 >0 = 球往 +x 滚，<0 = 球往 -x 滚 */
#define STATIC_SEQ_ANGLE_0        10.0f   // 加速向 +5cm
#define STATIC_SEQ_TIME_0         600
#define STATIC_SEQ_ANGLE_1        -20.0f  // 减速
#define STATIC_SEQ_TIME_1         400
#define STATIC_SEQ_ANGLE_2        0.0f   // 停在 +5cm
#define STATIC_SEQ_TIME_2         0
#define STATIC_SEQ_ANGLE_3        -10.0f  // 加速向 -5cm
#define STATIC_SEQ_TIME_3         150
#define STATIC_SEQ_ANGLE_4        30.0f   // 减速
#define STATIC_SEQ_TIME_4         1500
#define STATIC_SEQ_ANGLE_5        0.0f   // 停在 -5cm
#define STATIC_SEQ_TIME_5         0
#define STATIC_SEQ_ANGLE_6        0.0f   // 结束
#define STATIC_SEQ_TIME_6         0

#endif /* __BALANCE_CONFIG_H__ */
