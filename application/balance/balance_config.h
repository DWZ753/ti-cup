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
#define BALANCE_MAX_ANGLE_DEG     5.0f                // 摆杆最大倾角(°)

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
#define BALANCE_KP                0.15f
/* °/(mm·s)，I 项增益→积分消除静态偏差 */
#define BALANCE_KI                0.01f
/* °/(mm/s)，球速→反向倾角 */
#define BALANCE_KD                0.25f

/* ========== 底盘前馈 ========== */

/* °/(m/s²)，底盘加速度→补偿倾角
   理论值 ≈ 1/g × 180/π ≈ 5.8
   实测调节：
     - 车加速时球往后滚 → 加大此值（如 6.5 ~ 8.0）
     - 车加速时球往前冲 → 减小此值（如 3.0 ~ 5.0）
     - 刹车时球往前滚 → 加大；球往后靠 → 减小 */
#define FF_ACCEL_GAIN             7.5f

/* m/s²，加速度死区：低于此值判定为匀速，摆杆回水平 */
#define FF_ACCEL_DEADZONE         0.5f

/* 加速度低通系数（0~1）：越大越灵敏但越抖，越小越平滑但滞后 */
#define FF_ACCEL_FILTER           0.5f

/* ========== 低通滤波器 ========== */

#define POS_FILTER_ALPHA          0.3f   // 位置（越大越灵敏）
#define VEL_FILTER_ALPHA          0.2f   // 速度（越大噪声越大）

/* 死区 (mm)：球在目标 ± 此范围内时 D 项衰减，防噪声抖动 */
#define BALANCE_DEADBAND_MM        2.0f

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200    // 超时判丢球

/* ========== 静态平衡序列（要求3，闭环位置控制） ========== */

/*
 * 各阶段：{球目标位置(mm), 到达后停留(ms)}。
 * 球到达目标 ± SEQ_THRESHOLD 后开始停留计时，到时推进下一步。
 * 全部步骤完成后自动结束。正=右，负=左。
 */
#define STATIC_SEQ_LEN            3

#define STATIC_SEQ_TARGET_0       0.0f    // 起始 O 点
#define STATIC_SEQ_DWELL_0        500
#define STATIC_SEQ_TARGET_1       50.0f   // +5cm
#define STATIC_SEQ_DWELL_1        1000
#define STATIC_SEQ_TARGET_2       -50.0f  // -5cm
#define STATIC_SEQ_DWELL_2        1000

/* 球到达目标判定阈值 (mm) */
#define BALANCE_SEQ_THRESHOLD_MM   5.0f

#endif /* __BALANCE_CONFIG_H__ */
