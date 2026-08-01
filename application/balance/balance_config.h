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

/**
 * 摆杆最大倾角 (°)：电机允许的最大倾斜角度。
 * 增大 → 两端回拉力更强；减小 → 更安全但可能拉不回远处的球。
 * 调参用 Pi 发 0x0C（MAX_ANG）。
 */
#define BALANCE_MAX_ANGLE_DEG     5.0f

/* 暂时跳过回零，上电直接转 */
#define BALANCE_SKIP_HOMING       0

/* 硬停回零参数 */
#define BALANCE_ZERO_VEL          30
#define BALANCE_ZERO_CUR_MA       800
#define BALANCE_ZERO_TIME_MS      100
#define BALANCE_ZERO_DIR          1

/* 过流保护 */
#define BALANCE_OCP_MA            800
#define BALANCE_OCP_TIME_MS       200

/**
 * 回零后摆杆从地面抬起到水平位置的偏移角度 (°)。
 * 正值 = 上抬，负值 = 下压。
 * ⚠️ 必须实测标定。
 */
#define BALANCE_HOME_OFFSET_DEG   -45.5f

/* ========== 球位置 PID 参数 ========== */

/**
 * KP (°/mm)：比例增益。
 * 球偏离目标 1mm → 摆杆倾斜 KP 度。
 * KP 越大响应越快，但太大 → 振荡。太小 → 球反应迟钝。
 * 调参用 Pi 发 0x00。
 */
#define BALANCE_KP                0.15f

/**
 * P 项限幅 (°)：对 P 项输出的硬钳位。
 * 设成 800° 基本等于不限制（摆杆总限幅只有 5°）。
 * 如果你想限制"纯比例"的贡献可以改小，一般不需要。
 * 调参用 Pi 发 0x03。
 */
#define BALANCE_P_LIMIT_DEG       800.0f

/**
 * KI (°/(mm·s))：积分增益。
 * 球长时间停在目标旁边但到不了 → 加大 KI（消除稳态偏差）。
 * 球在目标周围来回摆动 → KI 太大，减小。
 * 调参用 Pi 发 0x01。
 */
#define BALANCE_KI                0.01f

/**
 * I 项限幅 (°)：积分累加器的硬上限。
 * 防止长时间大误差导致积分值过大（积分饱和），造成过冲。
 * 一般设成跟 MAX_ANG 差不多或稍小。太小 → 稳态偏差消不掉。
 * 调参用 Pi 发 0x04。
 */
#define BALANCE_I_LIMIT_DEG       5.0f

/**
 * KD (°/(mm/s))：微分增益（速度阻尼）。
 * 球速 1mm/s → 摆杆反向倾斜 KD 度，相当于"刹车"。
 * KD 越大 → 阻尼越强，球停得快但可能抖。太小 → 球来回冲。
 * 调参用 Pi 发 0x02。
 */
#define BALANCE_KD                0.3f

/**
 * D 项限幅 (°)：对 D 项输出的硬钳位。
 * 设成 150° 基本等于不限制。D 项正常不会超过几度。
 * 调参用 Pi 发 0x05。
 */
#define BALANCE_D_LIMIT_DEG       150.0f

/**
 * D 项死区 (mm)：球离目标小于此距离时，D 项按比例衰减。
 * 为什么？目标附近位置噪声被 D 放大 → 摆杆高频抖动。
 * 例如死区=2mm，球距目标1mm时 D×0.5，球在目标上时 D=0。
 * 调参用 Pi 发 0x06。
 */
#define BALANCE_DEADBAND_MM       2.0f

/* ========== 滤波 ========== */

/**
 * 位置低通系数 (0~1)：球位置的平滑程度。
 * 1.0 = 无滤波（直通），0.1 = 强滤波（平滑但滞后大）。
 * 调参用 Pi 发 0x07。
 */
#define POS_FILTER_ALPHA          0.3f

/**
 * 速度低通系数 (0~1)：球速度的平滑程度。
 * 速度估计本身噪声就大，建议比位置系数更小。
 * 调参用 Pi 发 0x08。
 */
#define VEL_FILTER_ALPHA          0.2f

/* ========== 底盘前馈 ========== */

/**
 * FF 增益 (°/(m/s²))：底盘加速度 → 摆杆补偿倾角。
 * 车加速时球受惯性后滚，摆杆需要往前倾来抵消。
 * 理论值 ≈ 1/g × 180/π ≈ 5.8。实际需要实测微调。
 * 调参用 Pi 发 0x09。
 */
#define FF_ACCEL_GAIN             7.5f

/**
 * FF 加速度死区 (m/s²)：低于此值的加速度视为匀速/静止。
 * 匀速时不需要前馈补偿（球不会惯性后滚）。
 * 调参用 Pi 发 0x0A。
 */
#define FF_ACCEL_DEADZONE         0.3f

/**
 * FF 低通系数 (0~1)：加速度估计的平滑程度。
 * 底盘加速度噪声大，建议设 0.3~0.5。
 * 调参用 Pi 发 0x0B。
 */
#define FF_ACCEL_FILTER           0.5f

/* ========== 静态平衡序列（要求3） ========== */

/*
 * 球目标位置(mm)，到达后停留(ms)。
 * 球进入目标 ± SEQ_THRESHOLD 后开始停留计时，到时推进下一步。
 * 全部步骤完成后自动结束。正=右，负=左。
 */
#define STATIC_SEQ_LEN            2

#define STATIC_SEQ_TARGET_0       50.0f   // +5cm
#define STATIC_SEQ_DWELL_0        1500
#define STATIC_SEQ_TARGET_1       -50.0f  // -5cm
#define STATIC_SEQ_DWELL_1        1500

/* 球到达目标判定阈值 (mm) */
#define BALANCE_SEQ_THRESHOLD_MM   5.0f

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200

#endif /* __BALANCE_CONFIG_H__ */
