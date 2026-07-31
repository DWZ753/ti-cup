/**
 * @file    balance.c
 * @brief   球-梁平衡控制模块实现
 *
 * 控制律：标准位置式 PID（modules/algorithm/pid）+ 底盘 FF
 *   angle_cmd = PID_Compute(ball_pos) + chassis_FF
 *
 * 依赖 ZDT 闭环步进电机控制摆杆倾角。
 * 合页未到时 BALANCE_SKIP_HOMING=1，跳过机械回零。
 */

#include "balance.h"
#include "balance_config.h"
#include "zdt_motor.h"
#include "delay.h"
#include "board.h"
#include "pid.h"

/* ========== 内部状态 ========== */

static float          s_angle;             // 当前摆杆倾角指令 (°)
static float          s_ball_pos;          // 低通后球位置 (mm)
static float          s_ball_pos_raw;      // 原始球位置（序列阈值用，无滞后）
static float          s_last_ball_pos;     // 上次球位置 (速度估计用)
static float          s_ff_accel;          // 待叠加底盘 FF 倾角 (°)
static uint32_t       s_last_update_ms;    // 上次 Update() 时间戳
static uint8_t        s_confidence;        // 最近一次置信度
static PID_Controller s_balance_pid;       // 标准位置式 PID 控制器

/* ========== 静态平衡序列（要求 3） ========== */

static const struct {
	float    target_mm;
	uint16_t dwell_ms;
} s_seq_table[] = {
	{ STATIC_SEQ_TARGET_0, STATIC_SEQ_DWELL_0 },
	{ STATIC_SEQ_TARGET_1, STATIC_SEQ_DWELL_1 },
};

#define S_SEQ_LEN  (sizeof(s_seq_table) / sizeof(s_seq_table[0]))

static uint8_t  s_seq_step;
static uint32_t s_seq_tick;
static bool     s_seq_running;
static bool     s_seq_done;
static bool     s_seq_reached;

/* ========== 内部控制 ========== */

/**
 * @brief 设置摆杆倾角（含钳位 + ZDT Pos 绝对位置发送）
 * @param angle_deg 水平相对倾角 (°)，正=右倾，负=左倾，0°=水平
 * @note  使用 Pos_Control(raF=1) 绝对位置模式，
 *        电机直接走到目标脉冲数，不依赖 QPos 历史累积。
 */
static void Set_Angle(float angle_deg)
{
	float    motor_angle;
	int32_t  target_pulses;
	uint8_t  dir;
	uint32_t clk;

	/* 坐标转换：水平相对 → 电机绝对坐标 (0°=回零硬停位) */
	motor_angle = angle_deg + BALANCE_HOME_OFFSET_DEG;

	/* 电机坐标系钳位（HOME_OFFSET ± MAX_ANGLE） */
	{
		float max_motor = BALANCE_HOME_OFFSET_DEG + BALANCE_MAX_ANGLE_DEG;
		float min_motor = BALANCE_HOME_OFFSET_DEG - BALANCE_MAX_ANGLE_DEG;

		if (motor_angle > max_motor)
			motor_angle = max_motor;
		else if (motor_angle < min_motor)
			motor_angle = min_motor;
	}

	/* 跳过重复目标 */
	if (motor_angle == s_angle)
		return;

	target_pulses = (int32_t)(motor_angle * BALANCE_PULSE_PER_DEG);
	if (target_pulses >= 0)
	{
		dir = 0;
		clk = (uint32_t)target_pulses;
	}
	else
	{
		dir = 1;
		clk = (uint32_t)(-target_pulses);
	}

	/* 绝对位置控制：电机走到 clk 脉冲位置，raF=1 表示绝对坐标 */
	ZDT_Motor_Pos_Control(BALANCE_MOTOR_ID, dir,
	                      BALANCE_WORK_VEL, 5,
	                      clk, 1, false);
	s_angle = motor_angle;
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
		 * 不再调用 Reset_CurPos_To_Zero（实测失效）。
		 * 改为软件坐标系转换：s_angle 跟踪电机绝对坐标，
		 * Set_Angle() 内部将水平相对角 + HOME_OFFSET 转到电机坐标。
		 */
	}

	/*
	 * 4. 初始化内部状态
	 *    s_angle = HOME_OFFSET_DEG，即电机坐标系中水平位置。
	 *    此后 Set_Angle(0) = 保持水平，Set_Angle(+5) = 右倾 5°。
	 */
	s_angle          = BALANCE_HOME_OFFSET_DEG;
	s_ball_pos       = 0.0f;
	s_ball_pos_raw   = 0.0f;
	s_last_ball_pos  = 0.0f;
	s_ff_accel       = 0.0f;
	s_last_update_ms = 0;
	s_confidence     = 0;

	/*
	 * 5. 初始化 PID 控制器
	 *    integral_limit: Σerror 限幅 (mm·sample)，Ki × limit = I 输出上限
	 *    output_limit:   输出限幅 (°)，对应 BALANCE_MAX_ANGLE_DEG
	 */
	PID_Init(&s_balance_pid,
	         BALANCE_KP, BALANCE_KI, BALANCE_KD,
	         BALANCE_PID_INTEGRAL_LIMIT,
	         BALANCE_MAX_ANGLE_DEG);
	PID_SetTarget(&s_balance_pid, 0.0f);

	s_seq_running    = false;
	s_seq_done       = false;
}

/* ========== 目标设定 ========== */

void Balance_SetTarget(float pos_mm)
{
	PID_SetTarget(&s_balance_pid, pos_mm);
	PID_Reset(&s_balance_pid);
}

float Balance_GetTarget(void)
{
	return s_balance_pid.target;
}

float Balance_GetIAccum(void)
{
	return s_balance_pid.Ki * s_balance_pid.integral;
}

float Balance_GetP(void)
{
	return s_balance_pid.Kp * s_balance_pid.error;
}

float Balance_GetD(void)
{
	return s_balance_pid.Kd * (s_balance_pid.error
	       - 2.0f * s_balance_pid.last_error
	       + s_balance_pid.prev_error);
}

/* ========== PID 控制更新（Pi @50Hz 调用） ========== */

void Balance_Update(float ball_pos_mm, float ball_vel_mm_s, uint8_t confidence)
{
	float    angle_cmd;

	s_confidence = confidence;

	/* 丢球 → 冻结角度，不更新 PID */
	if (confidence == 0)
		return;

	/* 保存原始位置（序列阈值检测用，避免滤波滞后） */
	s_ball_pos_raw = ball_pos_mm;

	/* 位置低通滤波 */
	s_ball_pos = s_ball_pos * (1.0f - POS_FILTER_ALPHA)
	           + ball_pos_mm    * POS_FILTER_ALPHA;

	s_last_update_ms = Board_GetTickMs();
	s_last_ball_pos  = s_ball_pos;

	/*
	 * 标准位置式 PID 计算（modules/algorithm/pid）
	 *   P_out = Kp × error
	 *   I_out = Ki × Σ(error)        （积分限幅 integral_limit）
	 *   D_out = Kd × (e - 2·e₁ + e₂) （二阶差分，加速度阻尼）
	 *   output = P + I + D           （输出限幅 output_limit）
	 */
	angle_cmd = PID_Compute(&s_balance_pid, s_ball_pos);

	/* 叠加底盘前馈（PID 限幅后叠加，惯性补偿不受限） */
	angle_cmd += s_ff_accel;
	s_ff_accel = 0.0f;

	/* 总输出安全钳位 */
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
 * @brief 启动静态平衡序列（闭环）
 * @note  PD 持续运行，按时间推进球目标位置：0 → +5cm → -5cm → 0
 */
void Balance_Start(void)
{
	s_seq_step    = 0;
	s_seq_tick    = Board_GetTickMs();
	s_seq_running = true;
	s_seq_done    = false;
	s_seq_reached = false;

	/* 设第一个目标位置 */
	if (S_SEQ_LEN > 0)
	{
		PID_SetTarget(&s_balance_pid, s_seq_table[0].target_mm);
		PID_Reset(&s_balance_pid);
	}
}

/**
 * @brief 静态平衡序列状态机 — 每主循环周期调用
 * @note  球到达目标 ± SEQ_THRESHOLD 后开始停留计时，到时推进。
 *        全部步骤完成 → 目标归零，标记 done。
 */
void Balance_SeqUpdate(void)
{
	uint32_t now_ms;
	float    err;
	float    target;

	if (!s_seq_running)
		return;

	now_ms = Board_GetTickMs();
	target = s_seq_table[s_seq_step].target_mm;

	/* 球到达目标？ */
	if (!s_seq_reached)
	{
		err = s_ball_pos_raw - target;
		if (err < 0.0f) err = -err;
		if (err < BALANCE_SEQ_THRESHOLD_MM)
		{
			s_seq_reached = true;
			s_seq_tick    = now_ms;
		}
		return;
	}

	/* 到达后停留计时 → 推进下一步 */
	if (now_ms - s_seq_tick >= s_seq_table[s_seq_step].dwell_ms)
	{
		s_seq_step++;
		s_seq_reached = false;

		/* 序列结束 */
		if (s_seq_step >= S_SEQ_LEN)
		{
			PID_SetTarget(&s_balance_pid, 0.0f);
			PID_Reset(&s_balance_pid);
			s_seq_running = false;
			s_seq_done    = true;
			return;
		}

		PID_SetTarget(&s_balance_pid, s_seq_table[s_seq_step].target_mm);
		PID_Reset(&s_balance_pid);
	}
}

/**
 * @brief 停止平衡控制，摆杆回水平
 */
void Balance_Stop(void)
{
	ZDT_Motor_Stop_Now(BALANCE_MOTOR_ID, false);
	s_seq_running = false;
	s_angle       = BALANCE_HOME_OFFSET_DEG;
}

/**
 * @brief 查询静态平衡序列是否执行完毕
 */
bool Balance_IsDone(void)
{
	return s_seq_done;
}
