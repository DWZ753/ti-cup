/**
 * @file    balance.c
 * @brief   球-梁平衡控制模块实现
 *
 * 控制架构：
 *   outer: ball_pos → PD(vel_window, brake_fade) → θ_cmd
 *   inner: θ_cmd → ZDT QPos_Control 相对位移
 *
 * 坐标系：电机零点在上限位，水平位置 = HOME_OFFSET_DEG。
 * HOME_OFFSET 在 Set_Angle() 的 delta 计算中自然抵消，
 * 运行时 delta_pulses = (new_angle - old_angle) * PULSE_PER_DEG。
 */

#include "balance.h"
#include "balance_config.h"
#include "zdt_motor.h"
#include "delay.h"
#include "board.h"

/* ========== 内部状态 ========== */

/* 位置与速度 */
static float    s_ball_pos;                     // 低通后球位置 (mm)
static float    s_ball_vel;                     // 窗口速度 (mm/s)
static float    s_pos_ring[VEL_WINDOW_SIZE];    // 位置环形缓冲
static uint32_t s_time_ring[VEL_WINDOW_SIZE];   // 时间戳环形缓冲
static uint8_t  s_ring_idx;                     // 当前写入位置
static uint8_t  s_ring_count;                   // 已填充样本数

/* 控制输出 */
static float    s_target_pos;                   // 目标位置 (mm)
static float    s_angle_cmd;                    // 当前逻辑角度指令 (°)
static float    s_i_accum;                      // I 项积分器
static float    s_ff_accel;                     // 待叠加 FF 倾角 (°)
static uint8_t  s_confidence;                   // 最近一次置信度

/* 运动状态 */
typedef enum {
	MOTION_UNKNOWN,
	MOTION_STATIONARY,
	MOTION_ROLLING,
} MotionState;

static MotionState s_motion;                    // 当前运动状态
static uint32_t    s_stationary_since_ms;       // 静止开始时间
static float       s_stationary_start_pos;      // 静止起始位置

/* 静止起动补偿 */
static float    s_breakaway_boost;              // 当前起动补偿 (°)
static float    s_breakaway_start_ms;           // 补偿开始时间
static bool     s_breakaway_active;             // 补偿激活中
static int8_t   s_breakaway_dir;                // 补偿方向 (+1/-1)

/* 时间戳 */
static uint32_t s_last_update_ms;

/* ========== 内部控制 ========== */

/**
 * @brief 设置摆杆倾角 — 相对位移模式
 * @param angle_deg 逻辑目标倾角 (°)，正=右倾，负=左倾，0=水平
 * @note  使用 QPos_Control（相对位移），与 init 上抬动作一致。
 *        避免 Pos_Control(raF=1) 和 QPos_Control 位置基准不一致的问题。
 */
static void Set_Angle(float angle_deg)
{
	float    motor_angle;
	float    last_motor_angle;
	float    delta_deg;
	int32_t  delta_pulses;

	/* 钳位 */
	if (angle_deg > BALANCE_MAX_ANGLE_DEG)
		angle_deg = BALANCE_MAX_ANGLE_DEG;
	else if (angle_deg < -BALANCE_MAX_ANGLE_DEG)
		angle_deg = -BALANCE_MAX_ANGLE_DEG;

	/* 逻辑角度 → 电机角度（叠加 HOME_OFFSET） */
	motor_angle      = angle_deg       + BALANCE_HOME_OFFSET_DEG;
	last_motor_angle = s_angle_cmd     + BALANCE_HOME_OFFSET_DEG;
	delta_deg        = motor_angle - last_motor_angle;
	delta_pulses     = (int32_t)(delta_deg * BALANCE_PULSE_PER_DEG);

	if (delta_pulses == 0)
		return;

	s_angle_cmd = angle_deg;

	ZDT_Motor_QPos_Control(BALANCE_MOTOR_ID, delta_pulses);
}

/* ========== 初始化 ========== */

void Balance_Init(void)
{
	/*
	 * 0. 等待电机驱动板上电完成
	 *    MCU 启动比驱动板快，不发这个等会导致命令丢失。
	 */
	delay_ms(1500);

	/*
	 * 1. ZDT 电机通信初始化
	 *    内部 UART 自注册 + 500ms 等待驱动板启动
	 */
	ZDT_Motor_Init();

#if BALANCE_SKIP_HOMING
	/*
	 * 合页未到：跳过机械回零，当前位置作为原点
	 */
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(50);
	ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);
#else
	/*
	 * 配置硬停回零参数（不存储到 flash）
	 * o_mode=2: 硬停回零（电机转动直到机械碰撞触发停转）
	 */
	ZDT_Motor_Origin_Modify_Params(BALANCE_MOTOR_ID, false,
	                               2,                  /* o_mode */
	                               BALANCE_ZERO_DIR,   /* o_dir */
	                               BALANCE_ZERO_VEL,
	                               10000,              /* 10s 超时 */
	                               BALANCE_ZERO_VEL,
	                               BALANCE_ZERO_CUR_MA,
	                               BALANCE_ZERO_TIME_MS,
	                               false);             /* 不上电自动回零 */

	/* 配置过流保护（不存储到 flash，须在使能前设置） */
	ZDT_Motor_Modify_Otocp(BALANCE_MOTOR_ID, false,
	                       100,               /* otp = 100°C */
	                       BALANCE_OCP_MA,
	                       BALANCE_OCP_TIME_MS);

	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(50);

	/* 触发硬停回零（阻塞 ~3s） */
	ZDT_Motor_Origin_Trigger_Return(BALANCE_MOTOR_ID, 2, false);
	delay_ms(3000);
	ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);

	/* 回零后重新使能，确保退出回零状态 */
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, false, false);
	delay_ms(50);
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(100);
#endif

	/*
	 * 2. 预设 QPos 参数（仅用于初始化阶段的相对上抬）
	 */
	ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
	                          BALANCE_WORK_VEL,
	                          BALANCE_WORK_ACC,
	                          0,     /* raF = 相对上次目标 */
	                          false);

	/*
	 * 3. 回零后上抬到水平位置
	 *    回零时摆杆碰地/硬停 → motor pos=0。
	 *    水平位置 = HOME_OFFSET_DEG（电机坐标系）。
	 *    不重置电机原点（Reset/Origin_Set_O 均无法更改电机原点），
	 *    改为在 Set_Angle() 中叠加 HOME_OFFSET 完成坐标转换。
	 */
	if (BALANCE_HOME_OFFSET_DEG != 0.0f)
	{
		int32_t pulses;

		pulses = (int32_t)(BALANCE_HOME_OFFSET_DEG
		                   * BALANCE_PULSE_PER_DEG);
		if (pulses != 0)
		{
			delay_ms(100);
			ZDT_Motor_QPos_Control(BALANCE_MOTOR_ID, pulses);
			delay_ms(1000);
		}
	}

	/* 4. 初始化内部状态（s_angle_cmd=0 表示水平位置） */
	s_angle_cmd      = 0.0f;
	s_target_pos     = 0.0f;
	s_ball_pos       = 0.0f;
	s_ball_vel       = 0.0f;
	s_ff_accel       = 0.0f;
	s_confidence     = 0;
	s_last_update_ms = 0;
	s_i_accum        = 0.0f;

	s_ring_idx       = 0;
	s_ring_count     = 0;

	s_motion               = MOTION_UNKNOWN;
	s_stationary_since_ms  = 0;
	s_stationary_start_pos = 0.0f;

	s_breakaway_boost      = 0.0f;
	s_breakaway_start_ms   = 0.0f;
	s_breakaway_active     = false;
	s_breakaway_dir        = 0;
}

/* ========== 目标设定 ========== */

void Balance_SetTarget(float pos_mm)
{
	s_target_pos = pos_mm;
}

/* ========== PD 控制更新（Pi @50Hz 调用） ========== */

void Balance_Update(float ball_pos_mm, float ball_vel_mm_s, uint8_t confidence)
{
	float    pos_error;
	float    angle_p;
	float    angle_d;
	float    angle_cmd;
	uint32_t now_ms;
	float    dt_s;
	uint8_t  newest_idx;
	uint8_t  oldest_idx;
	float    abs_err;
	bool     toward_target;
	float    d_scale;

	s_confidence = confidence;

	/* 丢球 → 冻结角度，不更新 PD */
	if (confidence == 0)
		return;

	now_ms = Board_GetTickMs();

	/* ---- 1. 位置低通滤波 ---- */
	s_ball_pos = s_ball_pos * (1.0f - POS_FILTER_ALPHA)
	           + ball_pos_mm    * POS_FILTER_ALPHA;

	/* ---- 2. 环形缓冲更新 ---- */
	s_pos_ring[s_ring_idx]  = s_ball_pos;
	s_time_ring[s_ring_idx] = now_ms;
	s_ring_idx = (s_ring_idx + 1) % VEL_WINDOW_SIZE;
	if (s_ring_count < VEL_WINDOW_SIZE)
		s_ring_count++;

	/* ---- 3. 窗口速度估计 ---- */
	if (s_ring_count >= VEL_WINDOW_SIZE)
	{
		uint32_t dt_ms;
		float    dp;
		float    raw_vel;

		/*
		 * 最旧样本 = 当前写入位置。
		 * 环形缓冲满后，s_ring_idx 指向最旧的元素。
		 */
		oldest_idx = s_ring_idx;
		newest_idx = (s_ring_idx + VEL_WINDOW_SIZE - 1) % VEL_WINDOW_SIZE;

		dt_ms = s_time_ring[newest_idx] - s_time_ring[oldest_idx];

		/* 至少 80% 窗口时长才计算有效速度 */
		if (dt_ms >= (uint32_t)(VEL_WINDOW_MS * 0.8f))
		{
			dp  = s_pos_ring[newest_idx]
			    - s_pos_ring[oldest_idx];
			raw_vel = dp / (float)dt_ms * 1000.0f;

			/* 一阶 IIR 平滑 */
			s_ball_vel = s_ball_vel * 0.7f + raw_vel * 0.3f;
		}
	}

	/* ---- 4. dt 计算（I 项用） ---- */
	dt_s = 0.0f;
	if (s_last_update_ms > 0)
	{
		dt_s = (float)(now_ms - s_last_update_ms) * 0.001f;
		if (dt_s > 0.1f)
			dt_s = 0.1f;
	}
	s_last_update_ms = now_ms;

	/* ---- 5. 运动状态判定 ---- */
	{
		float abs_vel;

		abs_vel = (s_ball_vel >= 0.0f) ? s_ball_vel : -s_ball_vel;
		(void)abs_vel;

		if (abs_vel < STATIONARY_VEL_MM_S)
		{
			if (s_motion != MOTION_STATIONARY)
			{
				if (s_stationary_since_ms == 0)
				{
					s_stationary_since_ms  = now_ms;
					s_stationary_start_pos = s_ball_pos;
				}
				else if (now_ms - s_stationary_since_ms
				         >= STATIONARY_CONFIRM_MS)
				{
					s_motion = MOTION_STATIONARY;
				}
			}
		}
		else
		{
			s_motion              = MOTION_ROLLING;
			s_stationary_since_ms = 0;
		}
	}

	/* ---- 6. PID 计算 ---- */

	pos_error = s_target_pos - s_ball_pos;

	/* 低置信时降 KP */
	if (confidence == 1)
		pos_error *= 0.5f;

	/* 6a. 目标死区：球在目标附近且静止 → 冻结角度 */
	abs_err = (pos_error >= 0.0f) ? pos_error : -pos_error;
	if (abs_err < TARGET_DEADBAND_MM
	    && s_motion == MOTION_STATIONARY)
	{
		s_i_accum            = 0.0f;
		s_breakaway_active   = false;
		s_breakaway_boost    = 0.0f;
		s_ff_accel           = 0.0f;
		return;
	}

	/* 6b. P 项 + 限幅 */
	angle_p = BALANCE_KP * pos_error;
	if (angle_p > BALANCE_P_LIMIT_DEG)
		angle_p = BALANCE_P_LIMIT_DEG;
	else if (angle_p < -BALANCE_P_LIMIT_DEG)
		angle_p = -BALANCE_P_LIMIT_DEG;

	/* 6c. D 项 + 制动渐进 */
	angle_d   = BALANCE_KD * s_ball_vel;
	toward_target = (pos_error > 0.0f && s_ball_vel < 0.0f)
	             || (pos_error < 0.0f && s_ball_vel > 0.0f);

	if (toward_target)
	{
		/*
		 * 球正滚向目标 → 按距离衰减 D。
		 * 远处只保留 20% D，防止过早压平轨道。
		 * 近处全 D，精确制动。
		 */
		if (abs_err > BRAKE_START_ERROR_MM)
			d_scale = BRAKE_FAR_SCALE;
		else if (abs_err > BRAKE_FULL_ERROR_MM)
			d_scale = BRAKE_FAR_SCALE
			        + (1.0f - BRAKE_FAR_SCALE)
			          * (BRAKE_START_ERROR_MM - abs_err)
			          / (BRAKE_START_ERROR_MM - BRAKE_FULL_ERROR_MM);
		else
			d_scale = 1.0f;
	}
	else
	{
		/* 球滚离目标 → 全 D，全力制动 */
		d_scale = 1.0f;
	}
	angle_d *= d_scale;

	if (angle_d > BALANCE_D_LIMIT_DEG)
		angle_d = BALANCE_D_LIMIT_DEG;
	else if (angle_d < -BALANCE_D_LIMIT_DEG)
		angle_d = -BALANCE_D_LIMIT_DEG;

	/* 6d. I 项 + anti-windup */
	{
		float pd_sum;

		pd_sum = angle_p - angle_d;

		if (s_motion == MOTION_ROLLING
		    && dt_s > 0.001f
		    && pd_sum > -BALANCE_MAX_ANGLE_DEG
		    && pd_sum < BALANCE_MAX_ANGLE_DEG)
		{
			s_i_accum += BALANCE_KI * pos_error * dt_s;
		}

		if (s_i_accum > BALANCE_I_LIMIT_DEG)
			s_i_accum = BALANCE_I_LIMIT_DEG;
		else if (s_i_accum < -BALANCE_I_LIMIT_DEG)
			s_i_accum = -BALANCE_I_LIMIT_DEG;

		angle_cmd = pd_sum + s_i_accum;
	}

	/* 6e. 静止起动补偿 */
	if (s_motion == MOTION_STATIONARY
	    && abs_err > BREAKAWAY_ERROR_MIN_MM)
	{
		float boost_dt;
		float boost_target;

		if (!s_breakaway_active)
		{
			s_breakaway_active   = true;
			s_breakaway_dir      = (pos_error > 0.0f) ? 1 : -1;
			s_breakaway_start_ms = (float)now_ms;
			s_breakaway_boost    = 0.0f;
		}

		boost_dt     = ((float)now_ms - s_breakaway_start_ms) * 0.001f;
		boost_target = boost_dt * BREAKAWAY_BOOST_DEG_PER_S;
		if (boost_target > BREAKAWAY_MAX_BOOST_DEG)
			boost_target = BREAKAWAY_MAX_BOOST_DEG;
		s_breakaway_boost = boost_target;

		angle_cmd += s_breakaway_boost * (float)s_breakaway_dir;
	}
	else
	{
		/*
		 * 球在滚动或已进入目标带 → 快速释放补偿。
		 */
		if (s_breakaway_active)
		{
			bool  rolling_away;
			float abs_vel;

			abs_vel = (s_ball_vel >= 0.0f) ? s_ball_vel : -s_ball_vel;
			(void)abs_vel;

			rolling_away =
				(pos_error > 0.0f && s_ball_vel > BREAKAWAY_RELEASE_VEL_MM_S)
				|| (pos_error < 0.0f && s_ball_vel < -BREAKAWAY_RELEASE_VEL_MM_S);

			if (s_motion == MOTION_ROLLING
			    || abs_err < TARGET_DEADBAND_MM
			    || rolling_away)
			{
				s_breakaway_boost *= 0.3f;
				if (s_breakaway_boost < 0.2f)
				{
					s_breakaway_boost  = 0.0f;
					s_breakaway_active = false;
					s_breakaway_dir    = 0;
				}
				angle_cmd += s_breakaway_boost * (float)s_breakaway_dir;
			}
		}
	}

	/* 6f. 叠加底盘前馈 + 控制方向 + 发送 */
	angle_cmd += s_ff_accel;
	s_ff_accel = 0.0f;

	angle_cmd *= (float)BALANCE_CONTROL_SIGN;

	Set_Angle(angle_cmd);
}

/* ========== 底盘加速度前馈 ========== */

void Balance_ChassisFF(float accel_m_s2)
{
	/*
	 * 底盘加速 → 球受惯性后滚 → 需摆杆前倾补偿
	 * θ_ff ≈ a/g × 180/π ≈ 5.8 °/(m/s²)
	 */
	s_ff_accel += FF_ACCEL_GAIN * accel_m_s2;
}

/* ========== 状态查询 ========== */

float Balance_GetAngle(void)
{
	return s_angle_cmd;
}

void Balance_SetAngle(float angle_deg)
{
	Set_Angle(angle_deg);
}

/* ========== 停止 ========== */

/**
 * @brief 停止平衡控制，摆杆回水平
 */
void Balance_Stop(void)
{
	ZDT_Motor_Stop_Now(BALANCE_MOTOR_ID, false);

	s_angle_cmd    = 0.0f;
	s_i_accum      = 0.0f;
	s_ff_accel     = 0.0f;

	s_breakaway_active = false;
	s_breakaway_boost  = 0.0f;
	s_breakaway_dir    = 0;

	s_motion              = MOTION_UNKNOWN;
	s_stationary_since_ms = 0;
}
