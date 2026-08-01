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
#include "chassis_config.h"
#include "protocol.h"
#include "protocol_commands.h"
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
	static uint32_t s_rx_frame_cnt;    // 解析成功的总帧数（调试）
	static uint32_t s_ball_frame_cnt;  // 收到的 0x25 球位置帧数（调试）
	static bool     s_task6_capture;    // 任务6：等待捕获首个球位置

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
	s_rx_frame_cnt++;

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

			/* 任务6：捕获启动时球位置作为目标 */
			if (s_task6_capture && conf > 0)
			{
				Balance_SetTarget((float)pos_mm);
				s_task6_capture = false;
			}

			/* 闭环 PD 控制 */
			Balance_Update((float)pos_mm, 0.0f, conf);
			s_debug_ball_pos = pos_mm;
			s_debug_ball_conf = conf;
			s_ball_frame_cnt++;
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
		Tracking_SetSpeedParams(TASK2_SPEED_STRAIGHT,
		                        TASK2_SPEED_ARC,
		                        TASK2_DIFF_GAIN);
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
	OLED_ShowString(0, 1, (uint8_t*)"Tgt:", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K2:Stop", 8);

	Format_Time(0);
	/* 16px 大字居中: 7 字符 x 8px = 56px, (128-56)/2 = 36 */
	OLED_ShowString(36, 2, s_time_buf, 16);

	/* PID 分量 */
	OLED_ShowString(0, 4, (uint8_t*)"P:", 8);
	OLED_ShowString(64, 4, (uint8_t*)"D:", 8);
	OLED_ShowString(0, 5, (uint8_t*)"I:", 8);
	OLED_ShowString(0, 6, (uint8_t*)"V:", 8);
}

/**
 * @brief 运行中刷新 OLED：时间、FF、I 项、目标位置、Pi 位置
 */
static void Running_Show_Time(uint32_t elapsed_ms)
{
	Format_Time(elapsed_ms);
	OLED_ShowString(36, 2, s_time_buf, 16);

	/* PID 分量 */
	{
		int32_t val;
		uint8_t sign;

		/* P 项 (0.1°) line 4 */
		val  = (int32_t)(Balance_GetP() * 10.0f);
		sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(16, 4, sign, 8);
		OLED_ShowNum(24, 4, (uint32_t)val, 3, 8);

		/* D 项 (0.1°) line 4 */
		val  = (int32_t)(Balance_GetD() * 10.0f);
		sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(80, 4, sign, 8);
		OLED_ShowNum(88, 4, (uint32_t)val, 3, 8);

		/* I 项 (0.01°) line 5 */
		val  = (int32_t)(Balance_GetIAccum() * 100.0f);
		sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(16, 5, sign, 8);
		OLED_ShowNum(24, 5, (uint32_t)val, 4, 8);

		/* 球速 (mm/s) line 6 */
		{
			int32_t vel = (int32_t)Balance_GetVel();
			sign = (vel >= 0) ? '+' : '-';
			if (vel < 0) vel = -vel;
			OLED_ShowChar(16, 6, sign, 8);
			OLED_ShowNum(24, 6, (uint32_t)vel, 4, 8);
		}
	}
		/* 目标位置 */
		{
			int32_t tgt = (int32_t)Balance_GetTarget();
			uint8_t sign;

			sign = (tgt >= 0) ? '+' : '-';
			if (tgt < 0) tgt = -tgt;
			OLED_ShowChar(32, 1, sign, 8);
			OLED_ShowNum(40, 1, (uint32_t)tgt, 4, 8);
			OLED_ShowString(72, 1, (uint8_t*)"mm", 8);

			/* 置信度: 0=丢球 1=低 2=正常 */
			OLED_ShowChar(88, 1, (uint8_t)('0' + s_debug_ball_conf), 8);
		}

		/* 接收帧计数（调试）: Rx = 解析成功的总帧数 */
		{
			OLED_ShowString(48, 7, (uint8_t*)"Rx:", 8);
			OLED_ShowNum(66, 7, s_rx_frame_cnt % 10000u, 4, 8);
		}

			/* Pi 球位置调试 */
		{
		int32_t pos = (int32_t)s_debug_ball_pos;
		uint8_t sign;

		sign = (pos >= 0) ? '+' : '-';
		if (pos < 0) pos = -pos;
		OLED_ShowChar(64, 6, sign, 8);
		OLED_ShowNum(72, 6, (uint32_t)pos, 4, 8);
		OLED_ShowString(104, 6, (uint8_t*)"mm", 8);
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
 * @brief 底盘加速度前馈 — 编码器反馈估计加速度，馈入 balance 模块
 *
 * 每 ~20ms 计算一次：编码器速度差分 → 低通滤波 → Balance_ChassisFF()。
 * 车加速/减速时摆杆反向倾斜抵消球的惯性力。
 *
 * 可调参数（balance_config.h）：
 *   FF_ACCEL_GAIN       — 补偿增益 (°/m/s²)
 *   FF_ACCEL_DEADZONE   — 加速度死区 (m/s²)
 *   FF_ACCEL_FILTER     — 低通系数
 */
static void ChassisFF_Update(void)
{
	uint32_t now = Board_GetTickMs();

	/* 与编码器更新频率对齐（50Hz PIT → ~20ms） */
	if (now - s_last_ff_ms < 20)
		return;

	float speed_now = (Motor_GetFilteredSpeed1() + Motor_GetFilteredSpeed2()) * 0.5f;
	float accel_raw = 0.0f;

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

	/* 通知 Pi 开机，开始视觉跟踪（0xFF = 未选任务） */
	{
		uint8_t boot = 0xFF;
		Protocol_SendFrame(CMD_TASK_START, &boot, 1);
	}

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

			if (s_selected_task != TASK_BAL_STATIC)
			{
				switch (s_selected_task)
				{
				case TASK_TRACK_ONLY:
					Tracking_SetSpeedParams(TASK2_SPEED_STRAIGHT,
					                        TASK2_SPEED_ARC,
					                        TASK2_DIFF_GAIN);
					Tracking_SetStopOnCurve(false);
					Tracking_SetSpeedRamp(false);     /* 任务2无需渐加/减速 */
					break;

				case TASK_TRACK_BAL_AB:
					Tracking_SetSpeedParams(TASK4_SPEED_STRAIGHT,
					                        TASK4_SPEED_ARC,
					                        TASK4_DIFF_GAIN);
					Tracking_SetStopOnCurve(true);    /* 首次转弯停车 */
					Balance_SetTarget(0.0f);
					Tracking_SetSpeedRamp(true);
					break;

				case TASK_TRACK_BAL_LAP_O:
					Tracking_SetSpeedParams(TASK5_SPEED_STRAIGHT,
					                        TASK5_SPEED_ARC,
					                        TASK5_DIFF_GAIN);
					Tracking_SetStopOnCurve(false);   /* 停车线停止 */
					Tracking_SetSpeedRamp(true);
					Balance_SetTarget(0.0f);          /* 目标=中心 */
					break;

				case TASK_TRACK_BAL_LAP_X:
					Tracking_SetSpeedParams(TASK6_SPEED_STRAIGHT,
					                        TASK6_SPEED_ARC,
					                        TASK6_DIFF_GAIN);
					Tracking_SetSpeedRamp(true);
					Tracking_SetStopOnCurve(false);   /* 停车线停止 */
					s_task6_capture = true;           /* 等 Pi 帧捕获球位置 */
					break;

				default:
					break;
				}

				Tracking_Start();
				s_last_chassis_speed = 0.0f;
				s_last_ff_ms         = 0;
			}

				/* 通知 Pi 任务开始 */
				{
					uint8_t task_id = (uint8_t)s_selected_task;
					Protocol_SendFrame(CMD_TASK_START, &task_id, 1);
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
					else if (s_selected_task >= TASK_TRACK_BAL_AB)
					{
						Tracking_Stop();
					}
					else
						Tracking_Stop();
				}

				/* 自动停止检测 */
				else if (s_selected_task != TASK_REMOTE)
				{
					if (!Tracking_IsRunning())
						stopped = true;
				}

				if (stopped)
				{
					s_last_time_ms = Board_GetTickMs() - s_start_ms;

					/* 任务4/5/6：摆杆回水平 */
					if (s_selected_task >= TASK_TRACK_BAL_AB)
					{
						Balance_SetAngle(0.0f);
					}

					/* 通知 Pi 任务完成 + 耗时(ms, 大端) */
					{
						uint8_t msg[] = { (uint8_t)(s_last_time_ms >> 8),
						                  (uint8_t)(s_last_time_ms & 0xFF) };
						Protocol_SendFrame(CMD_TASK_STOP, msg, 2);
					}

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
				break;

			case TASK_REMOTE:
			Remote_Update();
			break;
		}
	}
}
