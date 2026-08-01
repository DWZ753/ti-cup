/**
 * @file    balance_config.h
 * @brief   球-梁平衡控制 — 可调参数集中配置（旧版手写 PD+I）
 */

#ifndef __BALANCE_CONFIG_H__
#define __BALANCE_CONFIG_H__

/* ========== ZDT 电机 ========== */

#define BALANCE_MOTOR_ID          1
#define BALANCE_PULSE_PER_DEG     (3200.0f / 360.0f)  // 16细分

/**
 * 控制方向符号：+1 或 -1。
 * 摆杆正倾角应使球向正方向滚动。
 * 若球滚向更远（正反馈），将此值取反。
 */
#define BALANCE_CONTROL_SIGN      1

#define BALANCE_WORK_VEL          70                   // 工作转速(RPM)
#define BALANCE_WORK_ACC          5                    // 工作加速度
#define BALANCE_MAX_ANGLE_DEG     5.0f                 // 摆杆总限幅(°)

/* 暂时跳过回零，上电直接转 */
#define BALANCE_SKIP_HOMING       0

/* 硬停回零参数 */
#define BALANCE_ZERO_VEL          30
#define BALANCE_ZERO_CUR_MA       800
#define BALANCE_ZERO_TIME_MS      100
#define BALANCE_ZERO_DIR          1

/* 过流保护（驱动板硬件 OCP，须在使能前设置） */
#define BALANCE_OCP_MA            800
#define BALANCE_OCP_TIME_MS       200

/**
 * 回零后摆杆从地面抬起到水平位置的偏移角度 (°)。
 * 正值 = 上抬，负值 = 下压。
 * ⚠️ 必须实测标定。
 */
#define BALANCE_HOME_OFFSET_DEG   -45.5f

/* ========== 球位置 PD+I（手写，速度阻尼型 D） ========== */

/* °/mm，位置误差→倾角 */
#define BALANCE_KP                0.15f

/* P 项 ±限幅 (°) */
#define BALANCE_P_LIMIT_DEG       800.0f

/* °/(mm·s)，I 项增益 */
#define BALANCE_KI                0.01f

/* I 项 ±限幅 (°)，防积分饱和 */
#define BALANCE_I_LIMIT_DEG       5.0f

/* °/(mm/s)，球速→反向倾角（速度阻尼） */
#define BALANCE_KD                0.3f

/* D 项 ±限幅 (°) */
#define BALANCE_D_LIMIT_DEG       150.0f

/* mm，D 项死区：|error| < 此值时 D 按比例衰减 */
#define BALANCE_DEADBAND_MM       2.0f

/* ========== 速度估计 ========== */

#define VEL_FILTER_ALPHA          0.2f

/* ========== 目标死区 ========== */

/* 球在目标 ±此范围内且静止 → 冻结角度 */
#define TARGET_DEADBAND_MM        3.0f

/* ========== 低通滤波器 ========== */

#define POS_FILTER_ALPHA          0.3f

/* ========== 底盘前馈 ========== */

/* °/(m/s²)，底盘加速度→补偿倾角 */
#define FF_ACCEL_GAIN             7.5f

/* m/s²，加速度死区：低于此值判定为匀速 */
#define FF_ACCEL_DEADZONE         0.3f

/* 加速度低通系数（0~1）：越大越灵敏但越抖 */
#define FF_ACCEL_FILTER           0.5f

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200

#endif /* __BALANCE_CONFIG_H__ */
