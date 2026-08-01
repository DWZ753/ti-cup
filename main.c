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
#include "menu.h"
#include "balance.h"
#include "balance_config.h"
#include "chassis_config.h"
#include "protocol.h"
#include "motor.h"

/* ========== 遥控命令码（Pi → MSPM0） ========== */

#define CMD_MOTOR        0x30  /**< 电机差速: speed(int8), diff(int8) [-100,100] */
#define CMD_BEAM         0x31  /**< 摆杆倾角: angle(int8) [-10,10] 度 */
#define CMD_BALL_POS     0x25  /**< Pi → MCU 球位置: pos_mm(int16,BE) + confidence(uint8) */
#define CMD_SET_PID      0x26  /**< Pi → MCU 设置PID: param_id(1B) + value(f32 LE,4B) */
#define CMD_PID_ACK      0x27  /**< MCU → Pi PID确认: 同上 */
#define CMD_TASK_START   0x33  /**< MCU → Pi 任务开始: task_id(uint8) */
#define CMD_TASK_STOP    0x34  /**< MCU → Pi 任务结束: time_ms(uint16,BE) */
#define CMD_SET_TARGET   0x28  /**< Pi → MCU 设球目标: target_mm(int16,BE) */
#define CMD_RESUME_LINE  0x2F  /**< 回到循迹: 无载荷 */
#define CMD_STOP         0x32  /**< 立即停止: 无载荷 */

#define REMOTE_MAX_SPEED_MM_S  2136.0f   /**< speed=100 时的线速度 */
#define REMOTE_WATCHDOG_MS     500       /**< 无指令超时自动停车 */


/* ========== 菜单状态 ========== */
static UI_State  s_state         = STATE_MENU;
static UI_TaskMode  s_selected_task = TASK_TRACK_ONLY;

/* ========== 计时 ========== */

static uint32_t s_start_ms;         // 本次启动时刻 (tick)
static uint32_t s_last_time_ms;     // 上次运行用时 (ms)

/* ========== 遥控状态 ========== */

static int8_t   s_rc_speed;       // Pi 下发的速度百分比 [-100, +100]
static int8_t   s_rc_diff;        // Pi 下发的差速百分比 [-100, +100]
static int8_t   s_rc_beam;        // Pi 下发的摆杆倾角 (°)
static uint32_t s_rc_last_ms;     // 最近收到 Pi 指令的时间戳
	/* ========== Pi 球位置调试 ========== */
	static int16_t  s_debug_ball_pos;  // Pi 发来的球位置 (mm)
	static uint8_t  s_debug_ball_conf; // Pi 发来的置信度 0/1/2
	static bool     s_task6_capture;    // 任务6：等待捕获首个球位置

/* ========== 底盘前馈（由 balance 模块内部管理） ========== */

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
					UI_SetDirty();
					s_state        = STATE_RUNNING;
					
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
		}
		break;

	case CMD_SET_PID:
		if (len >= 5)
		{
			uint8_t param_id = payload[0];
			float   value;
			uint8_t ack_buf[5];

			/* 小端 float32 解码 */
			{
				union { float f; uint8_t b[4]; } u;
				u.b[0] = payload[1];
				u.b[1] = payload[2];
				u.b[2] = payload[3];
				u.b[3] = payload[4];
				value = u.f;
			}

			Balance_SetParam((Balance_ParamID)param_id, value);

			/* 回传 ACK */
			ack_buf[0] = param_id;
			value = Balance_GetParam((Balance_ParamID)param_id);
			{
				union { float f; uint8_t b[4]; } u;
				u.f = value;
				ack_buf[1] = u.b[0];
				ack_buf[2] = u.b[1];
				ack_buf[3] = u.b[2];
				ack_buf[4] = u.b[3];
			}
			Protocol_SendFrame(CMD_PID_ACK, ack_buf, 5);
		}
		break;

	case CMD_SET_TARGET:
		if (len >= 2)
		{
			int16_t tgt = (int16_t)(((uint16_t)payload[0] << 8)
			                      | payload[1]);
			Balance_SetTarget((float)tgt);
		}
		break;

	case CMD_STOP:
		Motor_Brake();
		s_rc_speed = 0;
		s_rc_diff  = 0;
		Balance_Rezero();
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
		UI_SetDirty();
		s_state        = STATE_RUNNING;
		
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

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();
UI_Init();

	Tracking_Init();

	/* 初始化 Pi 协议 */
	Protocol_Init(OnPiFrame, Board_GetPiUART());

	/* 遥控初始值 */
	s_rc_speed    = 0;
	s_rc_diff     = 0;
	s_rc_beam     = 0;
	s_rc_last_ms  = Board_GetTickMs();

	/* 显示初始菜单 */
	
	UI_Refresh(s_state, s_selected_task, Board_GetTickMs() - s_start_ms, s_last_time_ms, NULL);

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
				UI_SetDirty();
				
			}

			/* KEY4：确认选择 → 进入就绪，摆杆归零 */
			if (Key_GetFlag(3))
			{
				Balance_Rezero();
				s_state      = STATE_READY;
				UI_SetDirty();
				
			}
			break;

		case STATE_READY:
			/* KEY3：返回菜单，摆杆归零 */
			if (Key_GetFlag(2))
			{
				Balance_Rezero();
				s_state      = STATE_MENU;
				UI_SetDirty();
				
			}

			/* KEY1：启动 + 开始计时 */
			if (Key_GetFlag(0))
			{
				s_start_ms     = Board_GetTickMs();
				UI_SetDirty();

			if (s_selected_task != TASK_BAL_STATIC)
			{
				switch (s_selected_task)
				{
				case TASK_TRACK_ONLY:
					Tracking_SetSpeedParams(TASK2_SPEED_STRAIGHT,
					                        TASK2_SPEED_ARC,
					                        TASK2_DIFF_GAIN);
					Tracking_SetStopOnCurve(false);
					Tracking_SetLapStop(true);        /* 一圈停车：2次弯道→直道 */
					Tracking_SetSpeedRamp(false);     /* 任务2无需渐加/减速 */
					break;

				case TASK_TRACK_BAL_AB:
					Tracking_SetSpeedParams(TASK4_SPEED_STRAIGHT,
					                        TASK4_SPEED_ARC,
					                        TASK4_DIFF_GAIN);
					Tracking_SetStopOnCurve(true);    /* B点停车：进入首个弯道 */
					Tracking_SetLapStop(false);
					Balance_SetTarget(0.0f);
					Tracking_SetSpeedRamp(true);
					break;

				case TASK_TRACK_BAL_LAP_O:
					Tracking_SetSpeedParams(TASK5_SPEED_STRAIGHT,
					                        TASK5_SPEED_ARC,
					                        TASK5_DIFF_GAIN);
					Tracking_SetStopOnCurve(false);
					Tracking_SetLapStop(true);        /* 一圈停车：2次弯道→直道 */
					Tracking_SetSpeedRamp(true);
					Balance_SetTarget(0.0f);          /* 目标=中心 */
					break;

				case TASK_TRACK_BAL_LAP_X:
					Tracking_SetSpeedParams(TASK6_SPEED_STRAIGHT,
					                        TASK6_SPEED_ARC,
					                        TASK6_DIFF_GAIN);
					Tracking_SetSpeedRamp(true);
					Tracking_SetStopOnCurve(false);
					Tracking_SetLapStop(true);        /* 一圈停车：2次弯道→直道 */
					s_task6_capture = true;           /* 等 Pi 帧捕获球位置 */
					break;

				default:
					break;
				}

				Tracking_Start();
				Balance_ResetFF();
			}

				/* 静态平衡任务：启动序列 */
			if (s_selected_task == TASK_BAL_STATIC)
				Balance_Start();

			/* 通知 Pi 任务开始 */
			{
				uint8_t tid = (uint8_t)s_selected_task;
				Protocol_SendFrame(CMD_TASK_START, &tid, 1);
			}

			s_state      = STATE_RUNNING;
				
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

				/* 自动停止检测（循迹任务用，平衡任务不适用） */
				else if (s_selected_task != TASK_REMOTE
				         && s_selected_task != TASK_BAL_STATIC)
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
						Balance_Rezero();
					}

					/* 通知 Pi 任务结束 + 耗时(ms, 大端) */
					{
						uint8_t msg[] = { (uint8_t)(s_last_time_ms >> 8),
						                  (uint8_t)(s_last_time_ms & 0xFF) };
						Protocol_SendFrame(CMD_TASK_STOP, msg, 2);
					}

					s_state        = STATE_READY;
					
				}
			}
			break;
		}

		/* ======== OLED 刷新 ======== */
		{
				UI_DebugValues dbg = {
					.ff_accel = Balance_GetFFAccel(),
					.ff_angle = Balance_GetFFAccel() * FF_ACCEL_GAIN,
					.pid_p = Balance_GetP(),
					.pid_d = Balance_GetD(),
					.pid_i = Balance_GetIAccum(),
					.target_mm = Balance_GetTarget(),
					.pi_ball_pos = s_debug_ball_pos,
				};
				UI_Refresh(s_state, s_selected_task, Board_GetTickMs() - s_start_ms, s_last_time_ms, &dbg);
			}

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

				/* 弯道差速时编码器速度差分的前馈失真 → 取消前馈；直道正常补偿 */
				bool pi_ok = (Board_GetTickMs() - s_rc_last_ms < PI_TIMEOUT_MS);
				Balance_ChassisFF_Update(pi_ok && !Tracking_IsCurve());
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
