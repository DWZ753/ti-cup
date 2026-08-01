/**
 * @file    balance.c
 * @brief   球-梁平衡控制模块实现（旧版手写 PD+I）
 *
 * 控制架构：
 *   outer: ball_pos → PD(vel_damping, brake_fade) → θ_cmd
 *   inner: θ_cmd → ZDT QPos_Control 相对位移
 *
 * 坐标系：电机零点在上限位，水平位置 = 0°（软件归零）。
 * Set_Angle() 使用 QPos_Control（相对位移），s_angle 跟踪逻辑角度。
 */

#include "balance.h"
#include "balance_config.h"
#include "zdt_motor.h"
#include "motor.h"
#include "delay.h"
#include "board.h"

/* ========== 内部状态 ========== */

/* 位置与速度 */
static float    s_ball_pos;                     // 低通后球位置 (mm)
static float    s_last_ball_pos;                // 上次球位置（差分用）
static float    s_ball_vel;                     // 滤波后球速度 (mm/s)

/*
 * 宽窗差分速度估计：用 3 帧间隔的原始位置差分，压制 int16 量化噪声。
 * 延迟 40ms（3 帧 @ 50Hz）+ EMA，替代原来的 125ms 级联低通。
 */
static float    s_vel_base_pos;                 // 宽窗差分基准位置 (mm)
static uint8_t  s_vel_frame_count;              // 帧计数 (0..2)
static float    s_vel_dt_sum;                   // 累积 dt (s)

/* 控制输出 */
static float    s_target_pos;                   // 目标位置 (mm)
static float    s_cmd_angle;                    // 最后发送的角度指令 (°)（查询用）
static float    s_i_accum;                      // I 项积分器
static float    s_ff_accel;                     // 待叠加 FF 倾角 (°)
static uint8_t  s_confidence;                   // 最近一次置信度
static float    s_last_pos_error;                // 上次位置误差（过零检测用）

/* 时间戳 */
static uint32_t s_last_update_ms;
static uint32_t s_last_good_ms;                 // 最后收到有效 Pi 帧的时间

/* ========== 静态平衡序列（要求 3） ========== */

static const struct {
	float    target_mm;
	uint16_t dwell_ms;
} s_seq_table[] = {
	{ STATIC_SEQ_TARGET_0, STATIC_SEQ_DWELL_0 },
	{ STATIC_SEQ_TARGET_1, STATIC_SEQ_DWELL_1 },
};

#define S_SEQ_LEN  (sizeof(s_seq_table) / sizeof(s_seq_table[0]))

static float    s_ball_pos_raw;      // 原始球位置（序列阈值用，无滞后）
static uint8_t  s_seq_step;
static uint32_t s_seq_tick;
static bool     s_seq_running;
static bool     s_seq_done;
static bool     s_seq_reached;
static bool     s_seq_open_loop;  // true=开环模式，false=闭环（Pi 在线）

/*
 * 开环角度序列：Pi 离线时直接驱动摆杆角度完成 O→+5cm→-5cm 往返。
 */
static const struct {
	float    angle_deg;
	uint16_t dwell_ms;
} s_open_loop_seq[] = {
	{ OPEN_LOOP_SEQ_ANGLE_0, OPEN_LOOP_SEQ_DWELL_0 },
	{ OPEN_LOOP_SEQ_ANGLE_1, OPEN_LOOP_SEQ_DWELL_1 },
	{ OPEN_LOOP_SEQ_ANGLE_2, OPEN_LOOP_SEQ_DWELL_2 },
	{ OPEN_LOOP_SEQ_ANGLE_3, OPEN_LOOP_SEQ_DWELL_3 },
	{ OPEN_LOOP_SEQ_ANGLE_4, OPEN_LOOP_SEQ_DWELL_4 },
	{ OPEN_LOOP_SEQ_ANGLE_5, OPEN_LOOP_SEQ_DWELL_5 },
};

#define S_OL_SEQ_LEN  (sizeof(s_open_loop_seq) / sizeof(s_open_loop_seq[0]))

/* ========== 可调参数（运行时副本，Pi 可在线修改） ========== */

static float s_kp          = BALANCE_KP;
static float s_ki          = BALANCE_KI;
static float s_kd          = BALANCE_KD;
static float s_p_limit     = BALANCE_P_LIMIT_DEG;
static float s_i_limit     = BALANCE_I_LIMIT_DEG;
static float s_d_limit     = BALANCE_D_LIMIT_DEG;
static float s_deadband    = BALANCE_DEADBAND_MM;
static float s_pos_alpha   = POS_FILTER_ALPHA;
static float s_vel_alpha   = VEL_FILTER_ALPHA;
static float s_ff_gain     = FF_ACCEL_GAIN;
static float s_ff_deadzone = FF_ACCEL_DEADZONE;
static float s_ff_filter   = FF_ACCEL_FILTER;
static float s_max_angle   = BALANCE_MAX_ANGLE_DEG;
static float s_home_offset  = BALANCE_HOME_OFFSET_DEG;

/* ========== 内部控制 ========== */

/**
 * @brief 设置摆杆倾角 — 绝对位置模式
 * @param angle_deg 逻辑目标倾角 (°)，正=右倾，负=左倾，0=水平
 * @note  使用 QPos_Control（raF=1 绝对模式），原点在 Init 中已设为零点。
 *        驱动器内部位置计数器是唯一真相源，无需软件跟踪位置。
 */
static void Set_Angle(float angle_deg)
{
	int32_t absolute_pulses;

	/* 钳位到安全范围 */
	if (angle_deg > s_max_angle)
		angle_deg = s_max_angle;
	else if (angle_deg < -s_max_angle)
		angle_deg = -s_max_angle;

	/*
	 * raF=1 绝对模式：原点在地面（homing 后 Reset_CurPos_To_Zero 处）。
	 * 水平位 = s_home_offset 度。绝对脉冲 = (angle + home_offset) × PULSE_PER_DEG。
	 * 不再在水平位重新标零——那一步不可靠。
	 */
	absolute_pulses = (int32_t)((angle_deg + s_home_offset)
	                            * BALANCE_PULSE_PER_DEG);

	s_cmd_angle = angle_deg;

	ZDT_Motor_QPos_Control(BALANCE_MOTOR_ID, absolute_pulses);
}

/* ========== 初始化 ========== */

void Balance_Init(void)
{
	/*
	 * 0. 等待电机驱动板上电完成
	 */
	delay_ms(1500);

	/*
	 * 1. ZDT 电机通信初始化
	 */
	ZDT_Motor_Init();

#if BALANCE_SKIP_HOMING
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(50);
	ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);
#else
	/*
	 * 配置硬停回零参数
	 */
	ZDT_Motor_Origin_Modify_Params(BALANCE_MOTOR_ID, false,
	                               2,
	                               BALANCE_ZERO_DIR,
	                               BALANCE_ZERO_VEL,
	                               10000,
	                               BALANCE_ZERO_VEL,
	                               BALANCE_ZERO_CUR_MA,
	                               BALANCE_ZERO_TIME_MS,
	                               false);

	/* 配置过流保护 */
	ZDT_Motor_Modify_Otocp(BALANCE_MOTOR_ID, false,
	                       100,
	                       BALANCE_OCP_MA,
	                       BALANCE_OCP_TIME_MS);

	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(50);

	/* 触发硬停回零（阻塞 ~3s） */
	ZDT_Motor_Origin_Trigger_Return(BALANCE_MOTOR_ID, 2, false);
	delay_ms(3000);
	ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);

	/* 回零后重新使能 */
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, false, false);
	delay_ms(50);
	ZDT_Motor_En_Control(BALANCE_MOTOR_ID, true, false);
	delay_ms(100);
#endif

	/*
	 * 2. 预设 QPos 参数（raF=1 绝对模式）
	 */
	ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
	                          BALANCE_WORK_VEL,
	                          BALANCE_WORK_ACC,
	                          1,     /* raF = 绝对位置 */
	                          false);

	/*
	 * 3. 回零后上抬到水平位置
	 * 使用 Pos_Control（raF=2，相对当前位置），不依赖 QPos 预设。
	 */
	if (s_home_offset != 0.0f)
	{
		int32_t pulses = (int32_t)(s_home_offset * BALANCE_PULSE_PER_DEG);
		if (pulses != 0)
		{
			uint8_t  dir = (pulses >= 0) ? 0 : 1;
			uint32_t clk = (uint32_t)((pulses >= 0) ? pulses : -pulses);
			delay_ms(100);
			ZDT_Motor_Pos_Control(BALANCE_MOTOR_ID, dir,
			                      BALANCE_WORK_VEL, BALANCE_WORK_ACC,
			                      clk, 2,     /* raF=2: 相对当前实际位置 */
			                      false);
			delay_ms(1000);
		}

	}

	/*
	 * 4. 初始化内部状态
	 */
	s_cmd_angle      = 0.0f;
	s_target_pos     = 0.0f;
	s_ball_pos       = 0.0f;
	s_ball_vel       = 0.0f;
	s_ff_accel       = 0.0f;
	s_confidence     = 0;
	s_last_update_ms = 0;
	s_last_good_ms    = 0;
	s_vel_base_pos    = 0.0f;
	s_vel_frame_count = 0;
	s_vel_dt_sum      = 0.0f;
	s_i_accum        = 0.0f;
	/* 显式命令摆杆到水平位，确认坐标系统正确 */
	Set_Angle(0.0f);

	/* 激活闭环就绪：目标 O 点，等待 Pi 帧接管 */
	Balance_Enable();
}

/* ========== 目标设定 ========== */

void Balance_SetTarget(float pos_mm)
{
	s_target_pos = pos_mm;
	s_i_accum    = 0.0f;
}

float Balance_GetTarget(void)
{
	return s_target_pos;
}

float Balance_GetIAccum(void)
{
	return s_i_accum;
}

float Balance_GetP(void)
{
	float error = s_target_pos - s_ball_pos;
	return s_kp * error;
}

float Balance_GetD(void)
{
	return s_kd * s_ball_vel;
}

/* ========== 在线调参 ========== */

void Balance_SetParam(Balance_ParamID id, float value)
{
	switch (id)
	{
	case BALANCE_PARAM_KP:          s_kp          = value; break;
	case BALANCE_PARAM_KI:          s_ki          = value; break;
	case BALANCE_PARAM_KD:          s_kd          = value; break;
	case BALANCE_PARAM_P_LIMIT:     s_p_limit     = value; break;
	case BALANCE_PARAM_I_LIMIT:     s_i_limit     = value; break;
	case BALANCE_PARAM_D_LIMIT:     s_d_limit     = value; break;
	case BALANCE_PARAM_DEADBAND:    s_deadband    = value; break;
	case BALANCE_PARAM_POS_ALPHA:   s_pos_alpha   = value; break;
	case BALANCE_PARAM_VEL_ALPHA:   s_vel_alpha   = value; break;
	case BALANCE_PARAM_FF_GAIN:     s_ff_gain     = value; break;
	case BALANCE_PARAM_FF_DEADZONE: s_ff_deadzone = value; break;
	case BALANCE_PARAM_FF_FILTER:   s_ff_filter   = value; break;
	case BALANCE_PARAM_MAX_ANGLE:   s_max_angle   = value; break;
	case BALANCE_PARAM_HOME_OFFSET: s_home_offset = value;  break;
	case BALANCE_PARAM_RESET:       s_i_accum     = 0.0f;  break;
	}
}

float Balance_GetParam(Balance_ParamID id)
{
	switch (id)
	{
	case BALANCE_PARAM_KP:          return s_kp;
	case BALANCE_PARAM_KI:          return s_ki;
	case BALANCE_PARAM_KD:          return s_kd;
	case BALANCE_PARAM_P_LIMIT:     return s_p_limit;
	case BALANCE_PARAM_I_LIMIT:     return s_i_limit;
	case BALANCE_PARAM_D_LIMIT:     return s_d_limit;
	case BALANCE_PARAM_DEADBAND:    return s_deadband;
	case BALANCE_PARAM_POS_ALPHA:   return s_pos_alpha;
	case BALANCE_PARAM_VEL_ALPHA:   return s_vel_alpha;
	case BALANCE_PARAM_FF_GAIN:     return s_ff_gain;
	case BALANCE_PARAM_FF_DEADZONE: return s_ff_deadzone;
	case BALANCE_PARAM_FF_FILTER:   return s_ff_filter;
	case BALANCE_PARAM_MAX_ANGLE:   return s_max_angle;
	case BALANCE_PARAM_HOME_OFFSET: return s_home_offset;
	default:                        return 0.0f;
	}
}

/* ========== PID 控制更新（Pi @50Hz 调用） ========== */

void Balance_Update(float ball_pos_mm, float ball_vel_mm_s, uint8_t confidence)
{
	uint32_t now_ms;
	float    pos_error;
	float    abs_err;
	float    angle_p;
	float    angle_d;
	float    angle_cmd;
	float    dt_s;

	s_confidence = confidence;
	now_ms = Board_GetTickMs();

	/* ---- 0. 丢球超时检测 ---- */
	if (confidence == 0)
	{
		/*
		 * 连续丢球超过 BALL_LOST_TIMEOUT_MS → 渐进回水平。
		 * 渐进速率 = MAX_ANGLE / 1000ms × dt，避免突变。
		 */
		if (s_last_good_ms > 0
		    && now_ms - s_last_good_ms > BALL_LOST_TIMEOUT_MS)
		{
			float dt = (float)(now_ms - s_last_update_ms) * 0.001f;
			if (dt > 0.001f && dt < 0.5f)
			{
				float step = s_max_angle * 0.001f * dt * 1000.0f;
				if (s_cmd_angle > step)
					Set_Angle(s_cmd_angle - step);
				else if (s_cmd_angle < -step)
					Set_Angle(s_cmd_angle + step);
				else
					Set_Angle(0.0f);
			}
			s_i_accum = 0.0f;
		}
		/* 未超时 → 冻结摆杆（保持上次角度） */
		return;
	}

	s_last_good_ms = now_ms;

	/* ---- 1. 位置低通滤波 ---- */
	s_ball_pos_raw = ball_pos_mm;
	s_ball_pos = s_ball_pos * (1.0f - s_pos_alpha)
	           + ball_pos_mm    * s_pos_alpha;

	/* ---- 2. 宽窗差分速度估计 ---- */
	/*
	 * 对原始位置做 3 帧间隔差分，压制 int16 ±1mm 量化噪声（~3×），
	 * 再用单级 EMA 平滑。延迟 ~40ms（vs 旧方案 ~125ms）。
	 */
	{
		float frame_dt = 0.0f;
		if (s_last_update_ms > 0)
		{
			frame_dt = (float)(now_ms - s_last_update_ms) * 0.001f;
			if (frame_dt > 0.001f && frame_dt < 0.5f)
			{
				s_vel_dt_sum += frame_dt;
				s_vel_frame_count++;

				if (s_vel_frame_count >= 3)
				{
					float raw_vel;
					raw_vel = (s_ball_pos_raw - s_vel_base_pos) / s_vel_dt_sum;
					s_ball_vel = s_ball_vel * (1.0f - s_vel_alpha)
					           + raw_vel    * s_vel_alpha;

					s_vel_base_pos   = s_ball_pos_raw;
					s_vel_frame_count = 0;
					s_vel_dt_sum      = 0.0f;
				}
			}
		}
	}
	s_last_ball_pos = s_ball_pos;

	/* ---- 3. dt 计算（I 项用） ---- */
	dt_s = 0.0f;
	if (s_last_update_ms > 0)
	{
		dt_s = (float)(now_ms - s_last_update_ms) * 0.001f;
		if (dt_s > 0.1f)
			dt_s = 0.1f;
	}
	s_last_update_ms = now_ms;

	/* ---- 4. PID 计算 ---- */

	pos_error = s_target_pos - s_ball_pos;
	abs_err   = (pos_error >= 0.0f) ? pos_error : -pos_error;

	/* 低置信时降 KP */
	if (confidence == 1)
		pos_error *= 0.5f;

	/* 4a. P 项 + 限幅 */
	angle_p = s_kp * pos_error;
	if (angle_p > s_p_limit)
		angle_p = s_p_limit;
	else if (angle_p < -s_p_limit)
		angle_p = -s_p_limit;

	/*
	 * KP 增益调度：远处激进拉回，近处温和防过冲。
	 * 与刹车渐进（D 调度）互补——P 管拉力，D 管刹车。
	 */
	{
		float kp_scale;
		if (abs_err > 30.0f)
			kp_scale = KP_FAR_SCALE;        /* 远：激进 */
		else if (abs_err > 10.0f)
			kp_scale = 1.0f;                /* 中：正常 */
		else
			kp_scale = KP_NEAR_SCALE;       /* 近：温和 */
		angle_p *= kp_scale;
	}

	/* 4b. D 项（速度阻尼）+ 死区衰减 + 刹车渐进 */
	angle_d = s_kd * s_ball_vel;

	if (abs_err < s_deadband)
		angle_d *= (abs_err / s_deadband);

	/*
	 * 刹车渐进：球滚向目标时 D 会反向抵消 P 的拉力，导致球在半路停下。
	 * 解决方案：滚向目标时按距离衰减 D（远→弱，近→强），
	 * 滚离目标时全 D 全力刹车。
	 */
	{
		bool toward = (pos_error > 0.0f && s_ball_vel < 0.0f)
		           || (pos_error < 0.0f && s_ball_vel > 0.0f);
		float d_scale;

		if (!toward)
		{
			d_scale = 1.0f;                    /* 远离 → 全 D */
		}
		else if (abs_err > BRAKE_START_ERROR_MM)
		{
			d_scale = BRAKE_FAR_SCALE;         /* 远 → 弱 D */
		}
		else if (abs_err > BRAKE_FULL_ERROR_MM)
		{
			/* 中段线性过渡 */
			d_scale = BRAKE_FAR_SCALE
			        + (1.0f - BRAKE_FAR_SCALE)
			          * (BRAKE_START_ERROR_MM - abs_err)
			          / (BRAKE_START_ERROR_MM - BRAKE_FULL_ERROR_MM);
		}
		else
		{
			d_scale = 1.0f;                    /* 近 → 全 D */
		}

		angle_d *= d_scale;
	}

	if (angle_d > s_d_limit)
		angle_d = s_d_limit;
	else if (angle_d < -s_d_limit)
		angle_d = -s_d_limit;

	/* 4c. I 项：过零复位 + anti-windup + 卡住冻结 */
	{
		float pd_sum;

		/*
		 * 过零复位：误差穿过零点 → 球越过了目标。
		 * 清空 I 项防止摩擦导致的振荡中 I 项来回积累。
		 */
		if ((pos_error > 0.0f) != (s_last_pos_error > 0.0f))
		{
			s_i_accum = 0.0f;
		}
		s_last_pos_error = pos_error;

		/*
		 * 卡住检测：球静止但偏离目标 → 静摩擦卡住。
		 * 冻结 I 项积累，防止冲破静摩擦瞬间 I 项已过大导致过冲。
		 * 偏置角度在 4d 中叠加。
		 */
		bool stuck = (s_ball_vel > -STICTION_VEL_MMS
		           && s_ball_vel < STICTION_VEL_MMS
		           && abs_err > STICTION_ERR_MM);

		pd_sum = angle_p - angle_d;

		if (!stuck
		    && dt_s > 0.001f
		    && pd_sum > -s_max_angle
		    && pd_sum < s_max_angle)
		{
			s_i_accum += s_ki * pos_error * dt_s;
		}

		if (s_i_accum > s_i_limit)
			s_i_accum = s_i_limit;
		else if (s_i_accum < -s_i_limit)
			s_i_accum = -s_i_limit;

		angle_cmd = pd_sum + s_i_accum;

		/*
		 * Coulomb 摩擦补偿：卡住时叠加固定偏置角度
		 * 破坏静摩擦，方向 = 误差方向。
		 */
		if (stuck)
		{
			float bump = (pos_error > 0.0f)
			             ? STICTION_ANGLE_DEG : -STICTION_ANGLE_DEG;
			angle_cmd += bump;
		}
	}

	/* 4d. 叠加底盘前馈 + 控制方向 */
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
	s_ff_accel += s_ff_gain * accel_m_s2;
}

/* ========== 状态查询 ========== */

float Balance_GetAngle(void)
{
	return s_cmd_angle;
}

void Balance_SetAngle(float angle_deg)
{
	Set_Angle(angle_deg);
}

/* ========== 静态平衡序列（要求 3） ========== */

void Balance_Start(void)
{
	s_seq_step      = 0;
	s_seq_tick      = Board_GetTickMs();
	s_seq_running   = true;
	s_seq_done      = false;
	s_seq_reached   = false;
	s_seq_open_loop = false;  // 默认闭环，检测到 Pi 离线后切换

	if (S_SEQ_LEN > 0)
	{
		s_target_pos = s_seq_table[0].target_mm;
		s_i_accum    = 0.0f;
	}
}

void Balance_SeqUpdate(void)
{
	uint32_t now_ms;
	float    err;
	float    target;

	if (!s_seq_running)
		return;

	now_ms = Board_GetTickMs();

	/*
	 * Pi 连通性检测：超过 PI_TIMEOUT_MS 无有效帧 → 切到开环模式。
	 * 开环模式直接驱动摆杆角度序列，不依赖摄像头反馈。
	 */
	if (!s_seq_open_loop
	    && now_ms - s_last_good_ms > PI_TIMEOUT_MS
	    && s_last_good_ms > 0)
	{
		s_seq_open_loop = true;
		s_seq_step      = 0;
		s_seq_tick      = now_ms;
		s_seq_reached   = false;

		/* 立即应用第一步的开环角度 */
		Set_Angle(s_open_loop_seq[0].angle_deg);
	}

	/* ---- 开环模式：角度序列驱动 ---- */
	if (s_seq_open_loop)
	{
		if (now_ms - s_seq_tick >= s_open_loop_seq[s_seq_step].dwell_ms)
		{
			s_seq_step++;
			s_seq_tick = now_ms;

			if (s_seq_step >= S_OL_SEQ_LEN)
			{
				Set_Angle(0.0f);
				s_seq_running = false;
				s_seq_done    = true;
				return;
			}

			Set_Angle(s_open_loop_seq[s_seq_step].angle_deg);
		}
		return;
	}

	/* ---- 闭环模式：Pi 反馈驱动 ---- */
	target = s_seq_table[s_seq_step].target_mm;

	/*
	 * 球到达目标判定：用原始位置避免滤波滞后。
	 *
	 * 球可能高速冲过目标后停在阈值窗口之外（PD 稳住了但位置偏了），
	 * 仅靠 |误差| < THRESHOLD 会漏判。改为三条件：进入窗口 OR 越过目标
	 * OR 曾经进入过窗口（最近距离记忆）。
	 */
	{
		static float s_best_err = 999.0f;   /* 本步内最接近目标的距离 */
		static float s_last_raw = 0.0f;     /* 上一帧的原始位置（越界检测用） */

		if (!s_seq_reached)
		{
			float current_err = s_ball_pos_raw - target;
			if (current_err < 0.0f) current_err = -current_err;

			/* 记忆最近距离 */
			if (current_err < s_best_err)
				s_best_err = current_err;

			/*
			 * 三个判定条件（满足任一即算到达）：
			 * 1. 当前在阈值内
			 * 2. 曾经在阈值内（最近距离 ≤ 阈值）
			 * 3. 越过了目标（上一帧和目标的关系 ≠ 这一帧）
			 */
			bool crossed = ((s_last_raw - target) > 0.0f)
			            != ((s_ball_pos_raw - target) > 0.0f);
			bool in_window = (current_err <= BALANCE_SEQ_THRESHOLD_MM);
			bool ever_close = (s_best_err <= BALANCE_SEQ_THRESHOLD_MM);

			if (in_window || ever_close || crossed)
			{
				s_seq_reached = true;
				s_seq_tick    = now_ms;
				s_best_err    = 999.0f;  /* 重置，下一步用 */
			}

			s_last_raw = s_ball_pos_raw;
		}
		else
		{
			s_best_err = 999.0f;  /* 已到达，重置等待下一步 */
			s_last_raw = s_ball_pos_raw;
		}

		if (!s_seq_reached)
			return;
	}

	/* 到达后停留计时 → 推进下一步 */
	if (now_ms - s_seq_tick >= s_seq_table[s_seq_step].dwell_ms)
	{
		s_seq_step++;
		s_seq_reached = false;

		if (s_seq_step >= S_SEQ_LEN)
		{
			s_i_accum     = 0.0f;
			s_seq_running = false;
			s_seq_done    = true;
			return;
		}

		s_target_pos = s_seq_table[s_seq_step].target_mm;
		s_i_accum    = 0.0f;
	}
}

bool Balance_IsDone(void)
{
	return s_seq_done;
}

/* ========== 底盘加速度前馈 ========== */

/*
 * FF 内部状态：从编码器速度差分估计底盘加速度，
 * 低通滤波后馈入 Balance_ChassisFF()。
 */
static float    s_ff_last_speed;   /**< 上次左右轮平均速度 (mm/s) */
static uint32_t s_ff_last_ms;      /**< 上次 FF 计算时间戳 */
static float    s_ff_accel_f;      /**< 滤波后加速度 (m/s²)，调试用 */

/**
 * @brief 底盘加速度前馈更新 — 编码器速度差分 → 低通 → Balance_ChassisFF
 * @param pi_active true=Pi 在线（叠加到 PD 输出），false=Pi 离线（仅更新内部状态）
 */
void Balance_ChassisFF_Update(bool pi_active)
{
	uint32_t now = Board_GetTickMs();

	/* 与编码器更新频率对齐（~20ms） */
	if (now - s_ff_last_ms < 20)
		return;

	float speed_now = (Motor_GetFilteredSpeed1() + Motor_GetFilteredSpeed2()) * 0.5f;
	float accel_raw = 0.0f;

	if (s_ff_last_ms > 0)
	{
		float dt_s = (float)(now - s_ff_last_ms) * 0.001f;
		if (dt_s > 0.001f)
		{
			/* mm/s → m/s² */
			accel_raw = (speed_now - s_ff_last_speed) / dt_s * 0.001f;
		}
	}

	s_ff_last_speed = speed_now;
	s_ff_last_ms    = now;

	/*
	 * 非对称低通：匀速/减速阶段更快衰减，加速阶段正常滤波。
	 * decay = 1 - k·α，其中 k=1（同号/加速）或 k=1.5（异号/回零）。
	 * 最坏情况 α=0.5 → decay=0.25，永不为负。
	 */
	{
		float decay = (accel_raw * s_ff_accel_f >= 0.0f)
		              ? (1.0f - s_ff_filter)
		              : (1.0f - s_ff_filter * 1.5f);
		if (decay < 0.0f) decay = 0.0f;

		s_ff_accel_f = s_ff_accel_f * decay + accel_raw * s_ff_filter;
	}

	if (pi_active)
	{
		/* Pi 在线 → 叠加到 PD 控制律 */
		Balance_ChassisFF(s_ff_accel_f);
	}
	else
	{
		/*
		 * Pi 离线 → 不输出 FF（摆杆保持原位），
		 * 但仍更新内部状态以确保 Pi 重连时估计值平滑。
		 */
	}

	/* 速度接近 0 时强制衰减到 0，避免静止噪声累积 */
	if (speed_now > -20.0f && speed_now < 20.0f
	    && accel_raw > -0.5f && accel_raw < 0.5f)
	{
		s_ff_accel_f = 0.0f;
	}
}

float Balance_GetFFAccel(void)
{
	return s_ff_accel_f;
}

void Balance_ResetFF(void)
{
	s_ff_last_speed = 0.0f;
	s_ff_last_ms    = 0;
	s_ff_accel_f    = 0.0f;
}

void Balance_Rezero(void)
{
	/*
	 * raF=1 绝对模式下驱动器的位置计数器始终可信。
	 * 直接发 Set_Angle(0) 让摆杆回到水平位即可，无需 Reset_CurPos_To_Zero。
	 * 如果摆杆本就在水平位 → QPos_Control(addr, home_offset_pulses) 是空操作。
	 */
	Set_Angle(0.0f);
	Balance_Enable();
}

void Balance_Enable(void)
{
	/*
	 * 设置目标为 0（O 点），清零积分和前馈。
	 * Pi 发来 CMD_BALL_POS 时 Balance_Update() 自动接管。
	 * 摆杆由 ZDT 绝对位置闭环保持在 0，无需额外动作。
	 */
	s_target_pos = 0.0f;
	s_i_accum    = 0.0f;
	s_ff_accel   = 0.0f;
}

/* ========== 停止 ========== */

/**
 * @brief 停止平衡控制，摆杆回水平
 */
void Balance_Stop(void)
{
	ZDT_Motor_Stop_Now(BALANCE_MOTOR_ID, false);
	s_cmd_angle    = 0.0f;
	s_i_accum      = 0.0f;
	s_ff_accel     = 0.0f;
}
