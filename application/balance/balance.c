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
#include "delay.h"
#include "board.h"

/* ========== 内部状态 ========== */

/* 位置与速度 */
static float    s_ball_pos;                     // 低通后球位置 (mm)
static float    s_last_ball_pos;                // 上次球位置（差分用）
static float    s_ball_vel;                     // 滤波后球速度 (mm/s)

/* 控制输出 */
static float    s_target_pos;                   // 目标位置 (mm)
static float    s_angle_cmd;                    // 当前逻辑角度指令 (°)，0=水平
static float    s_i_accum;                      // I 项积分器
static float    s_ff_accel;                     // 待叠加 FF 倾角 (°)
static uint8_t  s_confidence;                   // 最近一次置信度

/* 时间戳 */
static uint32_t s_last_update_ms;

/* ========== 内部控制 ========== */

/**
 * @brief 设置摆杆倾角 — 相对位移模式
 * @param angle_deg 逻辑目标倾角 (°)，正=右倾，负=左倾，0=水平
 * @note  使用 QPos_Control（相对位移），与 init 上抬动作一致。
 */
static void Set_Angle(float angle_deg)
{
	float    last_motor_angle;
	float    motor_angle;
	float    delta_deg;
	int32_t  delta_pulses;

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
	 * 2. 预设 QPos 参数（raF=0 相对模式）
	 */
	ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
	                          BALANCE_WORK_VEL,
	                          BALANCE_WORK_ACC,
	                          0,     /* raF = 相对上次目标 */
	                          false);

	/*
	 * 3. 回零后上抬到水平位置
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

		/* 设为软件原点：此后 s_angle_cmd=0 表示水平 */
		ZDT_Motor_Reset_CurPos_To_Zero(BALANCE_MOTOR_ID);

		/* 重新预设 QPos 参数（Reset 后需重设） */
		ZDT_Motor_Set_QPos_Params(BALANCE_MOTOR_ID,
		                          BALANCE_WORK_VEL,
		                          BALANCE_WORK_ACC,
		                          0,
		                          false);
	}

	/*
	 * 4. 初始化内部状态
	 */
	s_angle_cmd      = 0.0f;
	s_target_pos     = 0.0f;
	s_ball_pos       = 0.0f;
	s_ball_vel       = 0.0f;
	s_ff_accel       = 0.0f;
	s_confidence     = 0;
	s_last_update_ms = 0;
	s_i_accum        = 0.0f;
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
	return BALANCE_KP * error;
}

float Balance_GetD(void)
{
	return BALANCE_KD * s_ball_vel;
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

	/* 丢球 → 冻结角度，不更新 PID */
	if (confidence == 0)
		return;

	now_ms = Board_GetTickMs();

	/* ---- 1. 位置低通滤波 ---- */
	s_ball_pos = s_ball_pos * (1.0f - POS_FILTER_ALPHA)
	           + ball_pos_mm    * POS_FILTER_ALPHA;

	/* ---- 2. 球速低通滤波（位置差分） ---- */
	if (s_last_update_ms > 0)
	{
		float raw_vel;
		float dt_s = (float)(now_ms - s_last_update_ms) * 0.001f;
		if (dt_s > 0.001f && dt_s < 0.5f)
		{
			raw_vel = (s_ball_pos - s_last_ball_pos) / dt_s;
			s_ball_vel = s_ball_vel * (1.0f - VEL_FILTER_ALPHA)
			           + raw_vel    * VEL_FILTER_ALPHA;
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
	angle_p = BALANCE_KP * pos_error;
	if (angle_p > BALANCE_P_LIMIT_DEG)
		angle_p = BALANCE_P_LIMIT_DEG;
	else if (angle_p < -BALANCE_P_LIMIT_DEG)
		angle_p = -BALANCE_P_LIMIT_DEG;

	/* 4b. D 项（速度阻尼）+ 死区衰减 */
	angle_d = BALANCE_KD * s_ball_vel;

	/* 死区：|error| < DEADBAND 时 D 按比例衰减 */
	if (abs_err < BALANCE_DEADBAND_MM)
		angle_d *= (abs_err / BALANCE_DEADBAND_MM);

	if (angle_d > BALANCE_D_LIMIT_DEG)
		angle_d = BALANCE_D_LIMIT_DEG;
	else if (angle_d < -BALANCE_D_LIMIT_DEG)
		angle_d = -BALANCE_D_LIMIT_DEG;

	/* 4c. I 项 + anti-windup */
	{
		float pd_sum;

		pd_sum = angle_p - angle_d;

		if (dt_s > 0.001f
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
}
