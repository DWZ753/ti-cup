/**
 * @file    main.c
 * @brief   26H 车载平衡滚球运动控制系统 — 小车端主程序
 *
 * 循迹控制逻辑封装在 tracking.c，main.c 仅负责调度：
 *   - KEY3 选择题目，KEY4 确认选择
 *   - KEY1 启动 + 开始计时，KEY2 停止 + 停止计时
 *   - OLED 显示菜单、计时、状态
 *   - 循环调用 Tracking_Update()
 *   - 树莓派 UART 遥控（要求7 Remote）
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "tracking.h"
#include "grayscale.h"
#include "key.h"
#include "oled.h"
#include "balance.h"
#include "balance_config.h"
#include "protocol.h"
#include "motor.h"
#include "zdt_motor.h"

/* ========== 遥控命令码（Pi → MSPM0） ========== */

#define CMD_MOTOR       0x30  /**< 电机差速: speed(int8), diff(int8) [-100,100] */
#define CMD_BEAM        0x31  /**< 摆杆倾角: angle(int8) [-10,10] 度 */
#define CMD_BALL_POS     0x25  /**< Pi → MCU 球位置: pos_mm(int16,BE) + confidence(uint8) */
#define CMD_STOP         0x32  /**< 立即停止: 无载荷 */
#define CMD_RESUME_LINE  0x2F  /**< 回到循迹: 无载荷 */

#define REMOTE_MAX_SPEED_MM_S  2136.0f   /**< speed=100 时的线速度 */
#define REMOTE_WATCHDOG_MS     500       /**< 无指令超时自动停车 */

/* ========== 题目模式（要求2-7） ========== */

typedef enum {
	TASK_TRACK_ONLY     = 0,  // 要求2：纯循迹一圈，A点停车
	TASK_BAL_STATIC,          // 要求3：静态平衡（车不动，球 O→±5cm）
	TASK_TRACK_BAL_AB,        // 要求4：循迹+O点平衡，AB段
	TASK_TRACK_BAL_LAP_O,     // 要求5：循迹+O点平衡，一圈
	TASK_TRACK_BAL_LAP_X,     // 要求6：循迹+任意位置平衡，一圈
	TASK_REMOTE,              // 要求7：Pi 遥控
	TASK_COUNT
} TaskMode;

static const uint8_t *s_task_names[TASK_COUNT] = {
	(uint8_t*)"2.Run 1 Lap",
	(uint8_t*)"3.Ball Balance",
	(uint8_t*)"4.Run+Bal AB",
	(uint8_t*)"5.Run+Bal 1Lap",
	(uint8_t*)"6.Run+Bal Any",
	(uint8_t*)"7.Remote",
};

/* ========== 菜单状态 ========== */

typedef enum {
	STATE_MENU,          // 选择题目：KEY3 切换，KEY4 确认
	STATE_READY,         // 已确认，等待启动：KEY1 启动，KEY3 返回菜单
	STATE_RUNNING,       // 运行中：KEY2 停止
} AppState;

static AppState  s_state         = STATE_MENU;
static TaskMode  s_selected_task = TASK_TRACK_ONLY;
static bool      s_oled_dirty    = true;

/* ========== 计时 ========== */

static uint32_t s_start_ms;         // 本次启动时刻 (tick)
static uint32_t s_last_time_ms;     // 上次运行用时 (ms)
static uint32_t s_last_oled_ms;     // 上次 OLED 刷新时刻
static uint8_t  s_time_buf[8];      // 格式化时间串 "MM:SS.T"

/* ========== 遥控状态 ========== */

static int8_t   s_rc_speed;       // Pi 下发的速度百分比 [-100, +100]
static int8_t   s_rc_diff;        // Pi 下发的差速百分比 [-100, +100]
static int8_t   s_rc_beam;        // Pi 下发的摆杆倾角 (°)
static uint32_t s_rc_last_ms;     // 最近收到 Pi 指令的时间戳
	/* ========== Pi 球位置调试 ========== */
	static int16_t  s_debug_ball_pos;  // Pi 发来的球位置 (mm)
	static uint8_t  s_debug_ball_conf; // Pi 发来的置信度 0/1/2

/* ========== 底盘前馈 ========== */

static float    s_last_chassis_speed;   // 上次左右轮平均速度 (mm/s)
static uint32_t s_last_ff_ms;          // 上次前馈计算时间戳
static float    s_ff_accel;            // [调试] 当前滤波后加速度 (m/s²)
static float    s_ff_angle;            // [调试] 当前 FF 输出倾角 (°)

/**
 * @brief 将毫秒格式化为 "MM:SS.T" 写入 s_time_buf
 */
static void Format_Time(uint32_t ms)
{
	uint16_t total_tenth = (uint16_t)(ms / 100);
	uint8_t  min         = (uint8_t)(total_tenth / 600);
	uint8_t  sec         = (uint8_t)((total_tenth / 10) % 60);
	uint8_t  tenth       = (uint8_t)(total_tenth % 10);

	s_time_buf[0] = '0' + (min / 10);
	s_time_buf[1] = '0' + (min % 10);
	s_time_buf[2] = ':';
	s_time_buf[3] = '0' + (sec / 10);
	s_time_buf[4] = '0' + (sec % 10);
	s_time_buf[5] = '.';
	s_time_buf[6] = '0' + tenth;
	s_time_buf[7] = '\0';
}

/* ========== Pi 协议帧回调 ========== */

/**
 * @brief 收到 Pi 帧时由 Protocol_Update 同步调用
 */
static void OnPiFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	s_rc_last_ms = Board_GetTickMs();

	switch (cmd)
	{
	case CMD_MOTOR:
		if (len >= 2)
		{
			s_rc_speed = (int8_t)payload[0];
			s_rc_diff  = (int8_t)payload[1];

			/* 非 REMOTE 模式下收到电机指令 → 自动切入 REMOTE */
			if (s_selected_task != TASK_REMOTE)
			{
				s_selected_task = TASK_REMOTE;
				/* 如果正在菜单/待启动，直接进入运行 */
				if (s_state != STATE_RUNNING)
				{
					s_start_ms     = Board_GetTickMs();
					s_last_oled_ms = 0;
					s_state        = STATE_RUNNING;
					s_oled_dirty   = true;
				}
			}
		}
		break;

	case CMD_BEAM:
		if (len >= 1)
		{
			s_rc_beam = (int8_t)payload[0];
			Balance_SetAngle((float)s_rc_beam);
		}
		break;

	case CMD_BALL_POS:
		if (len >= 3)
		{
			int16_t pos_mm = (int16_t)(((uint16_t)payload[0] << 8) | payload[1]);
			uint8_t conf   = payload[2];
			/* 闭环 PD 控制 */
			Balance_Update((float)pos_mm, 0.0f, conf);
			s_debug_ball_pos = pos_mm;
			s_debug_ball_conf = conf;
		}
		break;

	case CMD_STOP:
		Motor_Brake();
		s_rc_speed = 0;
		s_rc_diff  = 0;
		Balance_SetAngle(0.0f);
		break;

	case CMD_RESUME_LINE:
		Motor_Brake();
		s_rc_speed     = 0;
		s_rc_diff      = 0;
		s_selected_task = TASK_TRACK_ONLY;
		Tracking_Start();
		s_start_ms     = Board_GetTickMs();
		s_last_oled_ms = 0;
		s_state        = STATE_RUNNING;
		s_oled_dirty   = true;
		break;
	}
}

/* ========== OLED 显示 ========== */

static void Menu_Show(void)
{
	uint8_t i;

	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"=== Task Select ===", 8);

	for (i = 0; i < TASK_COUNT; i++)
	{
		uint8_t row = 1 + i;

		if (i == s_selected_task)
			OLED_ShowString(0, row, (uint8_t*)">", 8);
		OLED_ShowString(12, row, s_task_names[i], 8);
	}

	OLED_ShowString(0, 7, (uint8_t*)"K3:Next K4:OK", 8);
}

static void Ready_Show(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"Task:", 8);
	OLED_ShowString(0, 2, s_task_names[s_selected_task], 8);

	if (s_last_time_ms > 0)
	{
		Format_Time(s_last_time_ms);
		OLED_ShowString(0, 3, (uint8_t*)"Time:", 8);
		OLED_ShowString(30, 3, s_time_buf, 8);
	}

	OLED_ShowString(0, 5, (uint8_t*)"K1:Start K2:Stop", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K3:Back", 8);
}

/**
 * @brief 运行中首次绘制：清屏 + 静态元素 + 初始时间
 */
static void Running_Show_First(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, s_task_names[s_selected_task], 8);
	OLED_ShowString(0, 7, (uint8_t*)"K2:Stop", 8);

	Format_Time(0);
	/* 16px 大字居中: 7 字符 x 8px = 56px, (128-56)/2 = 36 */
	OLED_ShowString(36, 2, s_time_buf, 16);

	/* 编码器测速 + FF 调试 */
	OLED_ShowString(0, 4, (uint8_t*)"L:", 8);
	OLED_ShowString(0, 5, (uint8_t*)"R:", 8);
	OLED_ShowString(64, 4, (uint8_t*)"a:", 8);
	OLED_ShowString(64, 5, (uint8_t*)"F:", 8);
	OLED_ShowString(0, 6, (uint8_t*)"Pi:", 8);
}

/**
 * @brief 运行中更新时间 + 编码器速度
 */
static void Running_Show_Time(uint32_t elapsed_ms)
{
	Format_Time(elapsed_ms);
	OLED_ShowString(36, 2, s_time_buf, 16);

	/* 刷新编码器速度（带正负号，OLED_ShowNum 只接受无符号数） */
	{
		int32_t spd;
		uint8_t sign;

		spd  = (int32_t)Motor_GetFilteredSpeed1();
		sign = (spd >= 0) ? '+' : '-';
		if (spd < 0) spd = -spd;
		OLED_ShowChar(16, 4, sign, 8);
		OLED_ShowNum(24, 4, (uint32_t)spd, 4, 8);

		spd  = (int32_t)Motor_GetFilteredSpeed2();
		sign = (spd >= 0) ? '+' : '-';
		if (spd < 0) spd = -spd;
		OLED_ShowChar(16, 5, sign, 8);
		OLED_ShowNum(24, 5, (uint32_t)spd, 4, 8);
	}

	/* FF 调试：加速度 (cm/s²) + 倾角 (0.1°) */
	{
		int32_t val;
		uint8_t sign;

		val  = (int32_t)(s_ff_accel * 100.0f);  // m/s² → cm/s²
		sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(76, 4, sign, 8);
		OLED_ShowNum(84, 4, (uint32_t)val, 3, 8);

		val  = (int32_t)(s_ff_angle * 10.0f);   // ° → 0.1°
		sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(76, 5, sign, 8);
		OLED_ShowNum(84, 5, (uint32_t)val, 3, 8);
	}
		/* Pi 球位置调试 */
		{
		int32_t pos = (int32_t)s_debug_ball_pos;
		uint8_t sign;
		uint32_t age_ms;

		sign = (pos >= 0) ? '+' : '-';
		if (pos < 0) pos = -pos;
		OLED_ShowChar(24, 6, sign, 8);
		OLED_ShowNum(32, 6, (uint32_t)pos, 4, 8);
		OLED_ShowString(64, 6, (uint8_t*)"mm", 8);
		OLED_ShowChar(88, 6, 'C', 8);
		OLED_ShowChar(96, 6, '0' + s_debug_ball_conf, 8);

		/* 诊断：距上次 Pi 帧的毫秒数，>500ms = 链路断 */
		age_ms = Board_GetTickMs() - s_rc_last_ms;
		if (age_ms > 999) age_ms = 999;
		OLED_ShowChar(104, 6, 'D', 8);
		OLED_ShowNum(112, 6, age_ms, 3, 8);
	}
}

static void Oled_Refresh(void)
{
	switch (s_state)
	{
	case STATE_MENU:
		if (!s_oled_dirty)
			return;
		s_oled_dirty = false;
		Menu_Show();
		break;

	case STATE_READY:
		if (!s_oled_dirty)
			return;
		s_oled_dirty = false;
		Ready_Show();
		break;

	case STATE_RUNNING:
		{
			uint32_t now_ms = Board_GetTickMs();

			if (s_oled_dirty)
			{
				s_oled_dirty   = false;
				s_last_oled_ms = now_ms;
				Running_Show_First();
				return;
			}

			if (now_ms - s_last_oled_ms < 100)
				return;
			s_last_oled_ms = now_ms;
			Running_Show_Time(now_ms - s_start_ms);
		}
		break;
	}
}

/* ========== 遥控更新 ========== */

static void Remote_Update(void)
{
	float left_speed;
	float right_speed;

	/* 看门狗：超时自动停车 */
	if (Board_GetTickMs() - s_rc_last_ms > REMOTE_WATCHDOG_MS)
	{
		Motor_Brake();
		return;
	}

	/* Pi 差速控制 */
	left_speed  = (float)(s_rc_speed + s_rc_diff)
	              * REMOTE_MAX_SPEED_MM_S / 100.0f;
	right_speed = (float)(s_rc_speed - s_rc_diff)
	              * REMOTE_MAX_SPEED_MM_S / 100.0f;

	Motor_SetSpeedLR(left_speed, right_speed);
}

/* ========== 底盘加速度前馈 ========== */

/**
 * @brief 底盘加速度前馈 — 纯编码器反馈，直接驱动摆杆（无需 Pi）
 *
 * 每循迹周期调用一次。用编码器速度差分估计加速度，
 * 低通滤波后直接设摆杆倾角补偿惯性力。
 *
 * 可调参数（balance_config.h）：
 *   FF_ACCEL_GAIN       — 惯性补偿增益 (°/m/s²)，默认 5.8
 *   FF_ACCEL_DEADZONE   — 加速度死区 (m/s²)，低于此值摆杆回水平
 *   FF_ACCEL_FILTER     — 加速度低通系数，越大越灵敏但越抖
 *   BALANCE_MAX_ANGLE_DEG — 摆杆最大倾角安全限幅
 */
/**
 * @brief 底盘加速度前馈 — 纯编码器反馈，直接驱动摆杆（无需 Pi）
 *
 * 每 ~20ms 计算一次：编码器速度差分 → 低通滤波 → 摆杆倾角补偿。
 * 车加速/减速时摆杆反向倾斜抵消球的惯性力；匀速或静止时摆杆回水平。
 *
 * 可调参数（balance_config.h）：
 *   FF_ACCEL_GAIN       — 补偿增益 (°/m/s²)
 *   FF_ACCEL_DEADZONE   — 加速度死区 (m/s²)
 *   FF_ACCEL_FILTER     — 低通系数，已内置较快衰减（加速快响应，减速快回零）
 */
static void ChassisFF_Update(void)
{
	uint32_t now = Board_GetTickMs();

	/* 与编码器更新频率对齐（50Hz PIT → ~20ms） */
	if (now - s_last_ff_ms < 20)
		return;

	float speed_now = (Motor_GetFilteredSpeed1() + Motor_GetFilteredSpeed2()) * 0.5f;
	float accel_raw = 0.0f;
	float ff_angle;

	if (s_last_ff_ms > 0)
	{
		float dt_s = (float)(now - s_last_ff_ms) * 0.001f;
		if (dt_s > 0.001f)
		{
			accel_raw = (speed_now - s_last_chassis_speed) / dt_s * 0.001f;
		}
	}

	s_last_chassis_speed = speed_now;
	s_last_ff_ms = now;

	/* 非对称低通：加速阶段用 FF_ACCEL_FILTER，减速/匀速阶段快速衰减 */
	static float accel_filtered = 0.0f;
	float decay = (accel_raw * accel_filtered >= 0.0f)  /* 同号=加速中 */
	              ? (1.0f - FF_ACCEL_FILTER)              /* 正常衰减 */
	              : (1.0f - FF_ACCEL_FILTER * 3.0f);     /* 异号=回零中，3倍速衰减 */
	if (decay < 0.0f) decay = 0.0f;  /* 防止衰减系数为负 */

	accel_filtered = accel_filtered * decay + accel_raw * FF_ACCEL_FILTER;

	s_ff_accel = accel_filtered;

	/*
	 * Pi 闭环模式：加速度馈入 Balance_Update() 的 FF 累加器，
	 * 由 PD 控制律统一输出 QPos_Control。
	 * Pi 离线时退回开环 Pos_Control 直接驱动。
	 */
	if (Board_GetTickMs() - s_rc_last_ms < PI_TIMEOUT_MS)
	{
		Balance_ChassisFF(accel_filtered);
		s_ff_angle = accel_filtered * FF_ACCEL_GAIN;
		return;
	}

	/* ---- 开环：直接 Pos_Control（无 Pi 时） ---- */

	/* 速度接近 0 → 强制回平衡位 */
	if (speed_now > -20.0f && speed_now < 20.0f && accel_raw > -0.5f && accel_raw < 0.5f)
	{
		accel_filtered = 0.0f;
		s_ff_accel = 0.0f;
		s_ff_angle = 0.0f;
		ff_angle = 0.0f;  /* 0°=平衡位 */
		goto ff_apply;
	}

	/* 死区 → 回平衡位 */
	if (accel_filtered > -FF_ACCEL_DEADZONE && accel_filtered < FF_ACCEL_DEADZONE)
	{
		s_ff_angle = 0.0f;
		ff_angle = 0.0f;  /* 0°=平衡位 */
		goto ff_apply;
	}

	/* 惯性补偿：平衡位 + FF 偏移 */
	ff_angle = accel_filtered * FF_ACCEL_GAIN;  /* 0°=平衡位 */
	s_ff_angle = ff_angle;
	/* 钳位 ±BALANCE_MAX_ANGLE_DEG */
	if (ff_angle > BALANCE_MAX_ANGLE_DEG)
		ff_angle = BALANCE_MAX_ANGLE_DEG;
	else if (ff_angle < -BALANCE_MAX_ANGLE_DEG)
		ff_angle = -BALANCE_MAX_ANGLE_DEG;


ff_apply:
	/* 绝对位置控制：直接定位到目标角度，不依赖相对运动累积 */
	{
		int32_t target_pulses = (int32_t)(ff_angle * BALANCE_PULSE_PER_DEG);
		uint8_t dir;
		uint32_t clk;

		if (target_pulses >= 0) { dir = 0; clk = (uint32_t)target_pulses; }
		else                     { dir = 1; clk = (uint32_t)(-target_pulses); }

		ZDT_Motor_Pos_Control(BALANCE_MOTOR_ID, dir,
		                      BALANCE_WORK_VEL, 5,
		                      clk, 1, false);
	}
}

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	Tracking_Init();

	/* 初始化 Pi 协议 */
	Protocol_Init(OnPiFrame, Board_GetPiUART());

	/* 遥控初始值 */
	s_rc_speed    = 0;
	s_rc_diff     = 0;
	s_rc_beam     = 0;
	s_rc_last_ms  = Board_GetTickMs();

	/* 显示初始菜单 */
	s_oled_dirty = true;
	Oled_Refresh();

	while (1)
	{
		/* 收 Pi 帧 → 回调 OnPiFrame */
		Protocol_Update();

		/* ======== 按键 ======== */

		switch (s_state)
		{
		case STATE_MENU:
			/* KEY3：切换题目 */
			if (Key_GetFlag(2))
			{
				s_selected_task = (s_selected_task + 1) % TASK_COUNT;
				s_oled_dirty    = true;
			}

			/* KEY4：确认选择 */
			if (Key_GetFlag(3))
			{
				s_state      = STATE_READY;
				s_oled_dirty = true;
			}
			break;

		case STATE_READY:
			/* KEY3：返回菜单重新选择 */
			if (Key_GetFlag(2))
			{
				s_state      = STATE_MENU;
				s_oled_dirty = true;
			}

			/* KEY1：启动 + 开始计时 */
			if (Key_GetFlag(0))
			{
				s_start_ms     = Board_GetTickMs();
				s_last_oled_ms = 0;

				if (s_selected_task == TASK_BAL_STATIC)
				{
					Balance_Start();
				}
				else
				{
					Tracking_Start();
					s_last_chassis_speed = 0.0f;
					s_last_ff_ms         = 0;
				}

				s_state      = STATE_RUNNING;
				s_oled_dirty = true;
			}
			break;

		case STATE_RUNNING:
			{
				bool stopped = false;

				/* KEY2：停止 */
				if (Key_GetFlag(1))
				{
					stopped = true;

					if (s_selected_task == TASK_BAL_STATIC)
						Balance_Stop();
					else if (s_selected_task == TASK_REMOTE)
						Motor_Brake();
					else
						Tracking_Stop();
				}

				/* 自动停止检测 */
				if (s_selected_task == TASK_BAL_STATIC)
				{
					if (Balance_IsDone())
						stopped = true;
				}
				else if (s_selected_task != TASK_REMOTE)
				{
					if (!Tracking_IsRunning())
						stopped = true;
				}

				if (stopped)
				{
					s_last_time_ms = Board_GetTickMs() - s_start_ms;
					s_state        = STATE_READY;
					s_oled_dirty   = true;
				}
			}
			break;
		}

		/* ======== OLED 刷新 ======== */
		Oled_Refresh();

		/* ======== 任务更新 ======== */
		if (s_state != STATE_RUNNING)
			continue;

		switch (s_selected_task)
		{
		case TASK_TRACK_BAL_AB:
		case TASK_TRACK_BAL_LAP_O:
		case TASK_TRACK_BAL_LAP_X:
			{
				uint8_t mask = Grayscale_ReadAll();
				Tracking_Update(Board_GetTickMs(), mask);
				ChassisFF_Update();
			}
			break;

		case TASK_TRACK_ONLY:
			{
				uint8_t mask = Grayscale_ReadAll();
				Tracking_Update(Board_GetTickMs(), mask);
			}
			break;

		case TASK_BAL_STATIC:
			Balance_SeqUpdate();
			break;

		case TASK_REMOTE:
			Remote_Update();
			break;
		}
	}
}
