/**
 * @file    balance_config.h
 * @brief   球-梁平衡控制 — 可调参数集中配置
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
#define BALANCE_MAX_ANGLE_DEG     15.0f                // 摆杆总限幅(°)

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

/* ========== 球位置 PID（标准位置式，modules/algorithm/pid） ========== */

<<<<<<< HEAD
/* °/mm，位置误差→倾角 */
#define BALANCE_KP                0.03f

/* P 项 ±限幅 (°) */
#define BALANCE_P_LIMIT_DEG       800.0f

/* °/(mm·s)，I 项增益 */
#define BALANCE_KI                0.00f

/* I 项 ±限幅 (°)，防积分饱和 */
#define BALANCE_I_LIMIT_DEG       100.0f

/* °/(mm/s)，球速→反向倾角 */
#define BALANCE_KD                0.5f

/* D 项 ±限幅 (°) */
#define BALANCE_D_LIMIT_DEG       150.0f

/* ========== 速度估计窗口 ========== */

/* 环形缓冲样本数 (8 × 20ms ≈ 160ms) */
#define VEL_WINDOW_SIZE           8

/* 速度估计窗口时长 (ms) */
#define VEL_WINDOW_MS             160

/* ========== 运动状态判定 ========== */

/* 低于此速度判定为静止 (mm/s) */
#define STATIONARY_VEL_MM_S       3.0f

/* 持续静止确认时间 (ms) */
#define STATIONARY_CONFIRM_MS     120

/* ========== 静止起动补偿 ========== */

/* 补偿爬升速率 (°/s) */
#define BREAKAWAY_BOOST_DEG_PER_S 8.0f

/* 补偿最大角度 (°) */
#define BREAKAWAY_MAX_BOOST_DEG   8.0f

/* 触发补偿的最小误差 (mm) */
#define BREAKAWAY_ERROR_MIN_MM    4.0f

/* 离开目标方向滚动的速度阈值 (mm/s)，超过此值立即释放补偿 */
#define BREAKAWAY_RELEASE_VEL_MM_S 6.0f

/* ========== D 项制动渐进 ========== */

/* 此距离以上 D×BRAKE_FAR_SCALE */
#define BRAKE_START_ERROR_MM      40.0f

/* 此距离以下 D×1.0 */
#define BRAKE_FULL_ERROR_MM       8.0f

/* 远处 D 衰减系数 */
#define BRAKE_FAR_SCALE           0.2f

/* ========== 目标死区 ========== */

/* 球在目标 ±此范围内且静止 → 冻结角度 */
#define TARGET_DEADBAND_MM        3.0f

/* ========== 低通滤波器 ========== */

#define POS_FILTER_ALPHA          0.35f
=======
/*
 * PID 公式（PID_Compute 内部，~50Hz 固定周期调用）：
 *   error  = target - ball_pos          (mm)
 *   P_out  = KP × error                 (°)
 *   I_out  = KI × Σ(error)              (°)，Σ 限幅 integral_limit
 *   D_out  = KD × (e - 2·e₁ + e₂)      (°)，二阶差分 = 加速度阻尼
 *   output = P + I + D                  (°)，限幅 MAX_ANGLE_DEG
 *
 * 注意 KI/KD 不含 dt！与旧手写 PID 的参数不可直接对比。
 *   - KI 等效 ≈ 旧KI × dt（旧 0.15×0.02=0.003）
 *   - KD 等效 ≈ 旧KD × dt（旧 0.50×0.02=0.010）
 *   D 项从速度阻尼变为加速度阻尼，对噪声更不敏感。
 */

/* °/mm */
#define BALANCE_KP                    0.40f
/* °/(mm·sample)，积分增益 */
#define BALANCE_KI                    0.005f
/* °/(mm/sample²)，微分增益（加速度阻尼） */
#define BALANCE_KD                    0.010f
/* mm·sample，积分限幅（Σerror 上限，Ki×limit = I输出上限°） */
#define BALANCE_PID_INTEGRAL_LIMIT    1000.0f
>>>>>>> f978f42be76382ac9f7c8c321f9f48e7c114aadb

/* ========== 底盘前馈 ========== */

/* °/(m/s²)，底盘加速度→补偿倾角
   理论值 ≈ 1/g × 180/π ≈ 5.8
   实测调节：
     - 车加速时球往后滚 → 加大此值（如 6.5 ~ 8.0）
     - 车加速时球往前冲 → 减小此值（如 3.0 ~ 5.0）
     - 刹车时球往前滚 → 加大；球往后靠 → 减小 */
#define FF_ACCEL_GAIN             7.5f

/* m/s²，加速度死区：低于此值判定为匀速 */
#define FF_ACCEL_DEADZONE         0.5f

/* 加速度低通系数（0~1）：越大越灵敏但越抖 */
#define FF_ACCEL_FILTER           0.5f

<<<<<<< HEAD
/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200
=======
/* ========== 低通滤波器 ========== */

/*
 * 一阶低通: out = out·(1-α) + in·α
 *   α → 0: 强滤波，响应慢，滞后大（如 0.1~0.3）
 *   α → 1: 弱滤波/直通，响应快，噪声大
 *   α = 1.0: 完全直通，无滤波（调试时用）
 */
#define POS_FILTER_ALPHA          1.0f   // 位置低通（1.0=直通）

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200    // 超时判丢球

/* ========== 静态平衡序列（要求3，闭环位置控制） ========== */

/*
 * 各阶段：{球目标位置(mm), 到达后停留(ms)}。
 * 球到达目标 ± SEQ_THRESHOLD 后开始停留计时，到时推进下一步。
 * 全部步骤完成后自动结束。正=右，负=左。
 */
#define STATIC_SEQ_LEN            2

#define STATIC_SEQ_TARGET_0       50.0f   // +5cm
#define STATIC_SEQ_DWELL_0        1000
#define STATIC_SEQ_TARGET_1       -50.0f  // -5cm
#define STATIC_SEQ_DWELL_1        1000

/* 球到达目标判定阈值 (mm) */
#define BALANCE_SEQ_THRESHOLD_MM   10.0f
>>>>>>> f978f42be76382ac9f7c8c321f9f48e7c114aadb

#endif /* __BALANCE_CONFIG_H__ */
