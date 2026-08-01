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

/* 硬停回零使能（碰地有效，标零在 Set_Angle 中通过 HOME_OFFSET 处理） */
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
#define BALANCE_HOME_OFFSET_DEG   -45.5f   // 碰地后上抬到水平的角度，Pi 可在线微调(0x0E)

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

/* ========== D 项刹车渐进（Brake Fading） ========== */

/*
 * 球滚向目标时 D 项会刹掉 P 产生的向心速度，导致球停在半路。
 * 解决方案：球滚向目标时根据距离衰减 D 项（远处弱 D → 让球自由加速，
 * 近处全 D → 精确制动）。球远离目标时始终全 D（全力刹车）。
 */
#define BRAKE_START_ERROR_MM      40.0f   /**< 此距离以上 D×0.2 */
#define BRAKE_FULL_ERROR_MM       8.0f    /**< 此距离以下 D×1.0 */
#define BRAKE_FAR_SCALE           0.2f    /**< 远处 D 衰减系数 */

/* ========== Coulomb 摩擦补偿（Stiction Bump） ========== */

/*
 * 球在目标附近可能因静摩擦卡住不动，PD 输出不足以克服。
 * 检测卡住状态后叠加脉冲偏置破坏静摩擦，同时冻结 I 项防过冲。
 */
#define STICTION_VEL_MMS          5.0f    /**< mm/s，低于此速度视为卡住 */
#define STICTION_ERR_MM           3.0f    /**< mm，大于此误差才触发补偿 */
#define STICTION_ANGLE_DEG        1.0f    /**< °，破坏静摩擦的偏置角度 */
#define KP_FAR_SCALE              1.5f    /**< |误差|>30mm 时 KP 倍率 */
#define KP_NEAR_SCALE             0.6f    /**< |误差|<10mm 时 KP 倍率 */

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
#define FF_ACCEL_GAIN             10.0f

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
#define BALANCE_SEQ_THRESHOLD_MM   10.0f   // 要求3允许±1cm误差

/* ========== 开环角度序列（要求3 Pi 离线降级方案） ========== */

/*
 * 当 Pi 离线超过 PI_TIMEOUT_MS 时，静态平衡序列自动切到开环模式。
 * 每步：摆杆倾角 → 停留时间，推进下一帧。
 * 角度基于 balance控制说明.md 中的物理计算，实际需实测微调。
 */
#define OPEN_LOOP_SEQ_LEN         6

#define OPEN_LOOP_SEQ_ANGLE_0     5.0f    // +5° 右倾 → 球滚向右边
#define OPEN_LOOP_SEQ_DWELL_0     600
#define OPEN_LOOP_SEQ_ANGLE_1     -4.0f   // 减速/刹车
#define OPEN_LOOP_SEQ_DWELL_1     250
#define OPEN_LOOP_SEQ_ANGLE_2     0.0f    // 停在 +5cm 附近
#define OPEN_LOOP_SEQ_DWELL_2     1500
#define OPEN_LOOP_SEQ_ANGLE_3     -5.0f   // -5° 左倾 → 球滚回左边
#define OPEN_LOOP_SEQ_DWELL_3     600
#define OPEN_LOOP_SEQ_ANGLE_4     4.0f    // 减速/刹车
#define OPEN_LOOP_SEQ_DWELL_4     250
#define OPEN_LOOP_SEQ_ANGLE_5     0.0f    // 停在 -5cm 附近
#define OPEN_LOOP_SEQ_DWELL_5     1500

/* ========== 通信 ========== */

#define PI_TIMEOUT_MS             200

/** 丢球超时：超过此时长无有效帧 → 摆杆渐进回水平 */
#define BALL_LOST_TIMEOUT_MS      500

#endif /* __BALANCE_CONFIG_H__ */
