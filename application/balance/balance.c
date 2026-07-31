/**
 * @file    balance.c
 * @brief   球-梁平衡控制模块实现
 *
 * 控制律（MCU 开发指引 §3.2）：
 *   angle_cmd = KP * pos_error + KD * ball_vel + chassis_FF
 *
 * 依赖 ZDT 闭环步进电机控制摆杆倾角。
 * 合页未到时 BALANCE_SKIP_HOMING=1，跳过机械回零。
 */

#include "balance.h"
#include "balance_config.h"
#include "zdt_motor.h"
#include "delay.h"
#include "board.h"

/* ========== 内部状态 ========== */

static float    s_angle;             // 当前摆杆倾角指令 (°)
static float    s_target_pos;        // 球目标位置 (mm)
static float    s_ball_pos;          // 低通后球位置 (mm)
static float    s_ball_vel;          // 低通后球速度 (mm/s)
static float    s_last_ball_pos;     // 上次球位置 (速度估计用)
static float    s_ff_accel;          // 待叠加底盘 FF 倾角 (°)
static uint32_t s_last_update_ms;    // 上次 Update() 时间戳
	static float    s_i_accum;           // I 项积分器 (消除稳态偏置)
	static float    s_last_dt;           // 上次 dt (I 项用)
static uint8_t  s_confidence;        // 最近一次置信度

/* ========== 静态平衡序列（要求 3） ========== */

static const struct {
	float    angle_deg;
	uint16_t duration_ms;
} s_seq_table[] = {
	{ STATIC_SEQ_ANGLE_0, STATIC_SEQ_TIME_0 },
	{ STATIC_SEQ_ANGLE_1, STATIC_SEQ_TIME_1 },
	{ STATIC_SEQ_ANGLE_2, STATIC_SEQ_TIME_2 },
	{ STATIC_SEQ_ANGLE_3, STATIC_SEQ_TIME_3 },
	{ STATIC_SEQ_ANGLE_4, STATIC_SEQ_TIME_4 },
	{ STATIC_SEQ_ANGLE_5, STATIC_SEQ_TIME_5 },
	{ STATIC_SEQ_ANGLE_6, STATIC_SEQ_TIME_6 },
};

#define S_SEQ_LEN  (sizeof(s_seq_table) / sizeof(s_seq_table[0]))

static uint8_t  s_seq_step;
static uint32_t s_seq_tick;
static bool     s_seq_running;
static bool     s_seq_done;

/* ========== 内部控制 ========== */

/**
 * @brief 设置摆杆倾角（含钳位 + ZDT QPos 相对运动发送）
 * @param angle_deg 目标倾角 (°)，正=右倾，负=左倾
 */
static void Set_Angle(float angle_deg)
{
	float   delta_deg;
	int32_t delta_pulses;

	/* 钳位 */
	if (angle_deg > BALANCE_MAX_ANGLE_DEG)
		angle_deg = BALANCE_MAX_ANGLE_DEG;
	else if (angle_deg < -BALANCE_MAX_ANGLE_DEG)
		angle_deg = -BALANCE_MAX_ANGLE_DEG;

	/* QPos 相对上次目标运动 */
	delta_deg    = angle_deg - s_angle;
	delta_pulses = (int32_t)(delta_deg * BALANCE_PULSE_PER_DEG);

	if (delta_pulses != 0)
	{
		ZDT_Motor_QPos_Control(BALANCE_MOTOR_ID, delta_pulses);
		s_angle = angle_deg;
	}
}

/* ========== 初始化 ========== */

void Balance_Init(void)
{
	/*
	 * 0. 等待电机驱动板上电完成
	 *    MCU 启动比驱动板快，不发这个等会导致命令丢失，
	 *    表现为上电后必须按 MCU Reset 才执行。
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
	 * 2. 预设 QPos 参数
	 *    raF=0: QPos_Control 的脉冲数 = 相对上次目标的位移量
	 */
	ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
	                          BALANCE_WORK_VEL,
	                          5,     /* acc = 轻微加减速 */
	                          0,     /* raF = 相对上次目标 */
	                          false);

	/*
	 * 3. 回零后上抬到水平位置
	 *    回零时摆杆碰地/硬停 → motor pos=0。
	 *    水平位置在零点上方 BALANCE_HOME_OFFSET_DEG 度。
	 *    首次安装后实测：逐步加大此值直到摆杆肉眼水平。
	 */
	if (BALANCE_HOME_OFFSET_DEG != 0.0f)
	{
		int32_t pulses;

		pulses = (int32_t)(BALANCE_HOME_OFFSET_DEG
		                   * BALANCE_PULSE_PER_DEG);
		if (pulses != 0)
		{
			delay_ms(100);  // 等 QPos 参数生效
			ZDT_Motor_QPos_Control(BALANCE_MOTOR_ID, pulses);
			delay_ms(1000);
		}
		/*
		 * 将平衡位置设为新的零点（0°=水平，方便绝对定位）
		 * 此后 Pos_Control(0) 直接回平衡位，无需叠加 HOME_OFFSET。
		 */
		ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);
			/*
			 * 重设 QPos 参数，强制 QPos 目标同步到新零点。
			 * Reset_CurPos_To_Zero 仅清零绝对位置计数器，
			 * QPos 相对模式的目标寄存器需重新初始化，
			 * 否则每次 QPos_Control 会叠加 HOME_OFFSET 偏移。
			 */
			ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
			                          BALANCE_WORK_VEL,
			                          5,     /* acc */
			                          0,     /* raF = 相对模式 */
			                          false);
	}

	/* 4. 初始化内部状态（此时 s_angle=0 表示水平位置） */
	s_angle          = 0.0f;
	s_target_pos     = 0.0f;
	s_ball_pos       = 0.0f;
	s_ball_vel       = 0.0f;
	s_last_ball_pos  = 0.0f;
	s_ff_accel       = 0.0f;
	s_last_update_ms = 0;
	s_confidence     = 0;

	s_seq_running    = false;
	s_seq_done       = false;
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

	s_confidence = confidence;

	/* 丢球 → 冻结角度，不更新 PD */
	if (confidence == 0)
		return;

	/* 位置低通滤波 */
	s_ball_pos = s_ball_pos * (1.0f - POS_FILTER_ALPHA)
	           + ball_pos_mm    * POS_FILTER_ALPHA;

	/* 速度估计：一阶差分 + 低通 */
	now_ms = Board_GetTickMs();
	if (s_last_update_ms > 0)
	{
		float dt = (float)(now_ms - s_last_update_ms) * 0.001f;

		if (dt > 0.001f)
		{
			float raw_vel;

			raw_vel    = (s_ball_pos - s_last_ball_pos) / dt;
			s_last_dt = dt;
			s_ball_vel = s_ball_vel * (1.0f - VEL_FILTER_ALPHA)
			           + raw_vel     * VEL_FILTER_ALPHA;
		}
	}
	s_last_update_ms = now_ms;
	s_last_ball_pos  = s_ball_pos;

	/* 静态平衡序列运行中 → 角度由序列控制，跳过 PD */
	if (s_seq_running)
		return;

	/* ---- PD 控制律 ---- */

	pos_error = s_target_pos - s_ball_pos;

	/* 低置信时降 KP 减少激进控制 */
	if (confidence == 1)
		pos_error *= 0.5f;

	angle_p   = BALANCE_KP * pos_error;
	angle_d   = BALANCE_KD * s_ball_vel;

	/*
	 * 死区：球在目标附近时，位置噪声被 D 项放大导致摆杆抖动。
	 * 误差 < 死区时 D 项按比例衰减，压低假速度的影响。
	 */
	{
		float abs_err = (pos_error >= 0.0f) ? pos_error : -pos_error;

		if (abs_err < BALANCE_DEADBAND_MM)
			angle_d *= abs_err / BALANCE_DEADBAND_MM;
	}
	angle_cmd = angle_p - angle_d;
		/*
		 * I 项：积分消除恒定偏置（管子不水平、机械不对称）。
		 * Anti-windup：只在未饱和时累积，且 I 项 ±5° 硬限幅。
		 */
		{
		float dt = s_last_dt;

		if (dt > 0.001f && dt < 0.1f)
		{
			/* PD (不含 FF) 未饱和 → 累积积分 */
			if (angle_cmd > -BALANCE_MAX_ANGLE_DEG
			    && angle_cmd < BALANCE_MAX_ANGLE_DEG)
			{
				s_i_accum += BALANCE_KI * pos_error * dt;
			}

			/* I 项硬限幅 */
			if (s_i_accum > 5.0f)  s_i_accum = 5.0f;
			if (s_i_accum < -5.0f) s_i_accum = -5.0f;
		}
		}

		angle_cmd += s_i_accum;

	/* 叠加底盘前馈 */
	angle_cmd += s_ff_accel;
	s_ff_accel = 0.0f;

	/* 钳位 + 发送 */
	if (angle_cmd > BALANCE_MAX_ANGLE_DEG)
		angle_cmd = BALANCE_MAX_ANGLE_DEG;
	else if (angle_cmd < -BALANCE_MAX_ANGLE_DEG)
		angle_cmd = -BALANCE_MAX_ANGLE_DEG;

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
	return s_angle;
}

void Balance_SetAngle(float angle_deg)
{
	Set_Angle(angle_deg);
}

/* ========== 静态平衡序列（要求 3） ========== */

/**
 * @brief 启动静态平衡序列
 * @note  车静止，开环时序：O → +5cm → -5cm
 */
void Balance_Start(void)
{
	s_seq_step    = 0;
	s_seq_tick    = Board_GetTickMs();
	s_seq_running = true;
	s_seq_done    = false;

	/* 目标从 O 点出发 */
	s_target_pos = 0.0f;

	/* 应用第一步的角度 */
	if (S_SEQ_LEN > 0)
		Set_Angle(s_seq_table[0].angle_deg);
}

/**
 * @brief 静态平衡序列状态机 — 每主循环周期调用
 * @note  根据时间推进序列步骤，完成后摆杆回水平
 */
void Balance_SeqUpdate(void)
{
	uint32_t now_ms;

	if (!s_seq_running)
		return;

	now_ms = Board_GetTickMs();

	/* 当前步骤时间到 → 推进 */
	if (now_ms - s_seq_tick >= s_seq_table[s_seq_step].duration_ms)
	{
		s_seq_step++;

		/* 序列结束（超出表长或 duration_ms==0） */
		if (s_seq_step >= S_SEQ_LEN
		    || s_seq_table[s_seq_step].duration_ms == 0)
		{
			Set_Angle(0.0f);
			s_seq_running = false;
			s_seq_done    = true;
			return;
		}

		/* 进入下一步 */
		s_seq_tick = now_ms;
		Set_Angle(s_seq_table[s_seq_step].angle_deg);
	}
}

/**
 * @brief 停止平衡控制，摆杆回水平
 */
void Balance_Stop(void)
{
	ZDT_Motor_Stop_Now(BALANCE_MOTOR_ID, false);
	s_seq_running = false;
	s_angle       = 0.0f;
}

/**
 * @brief 查询静态平衡序列是否执行完毕
 */
bool Balance_IsDone(void)
{
	return s_seq_done;
}
