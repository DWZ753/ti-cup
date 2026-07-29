/**
 * @file    main.c
 * @brief   26H 车载平衡滚球运动控制系统 — 小车端主程序
 *
 * Phase 1: 纯灰度循迹（差速驱动），上电不动，KEY1 开始/暂停。
 *
 * 控制逻辑照搬 2024h：
 *   - 舵机 PID + 差速驱动（舵量 × ARC_DIFF_GAIN = 轮速差）
 *   - 10ms 循迹周期
 *   - slew rate 限幅替代 D 项
 *   - 丢线时舵机缓慢回中
 *   - 全传感器同时压黑线 = 停车横线检测
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "pid.h"
#include "tracking.h"
#include "chassis_config.h"
#include "grayscale.h"
#include "motor.h"
#include "servo.h"
#include "key.h"

/* ========== 内部状态 ========== */

static PID_Controller s_tracking_pid;
static int32_t        s_tracking_last_servo;   // slew rate 历史
static uint32_t       s_last_tracking;         // 上次循迹时间
static bool           s_running;               // 是否循迹中

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	/* ---- 循迹 PID ---- */
	PID_Init(&s_tracking_pid,
	         TRACKING_KP, TRACKING_KI, TRACKING_KD,
	         TRACKING_INT_LIMIT, TRACKING_OUT_LIMIT);
	PID_SetTarget(&s_tracking_pid, 0.0f);

	/* ---- 初始状态 ---- */
	s_running             = false;
	s_tracking_last_servo = 0;
	s_last_tracking       = 0;

	while (1)
	{
		uint32_t now = Board_GetTickMs();

		/* ======== 按键 ======== */

		/* KEY1：启动/停止循迹 */
		if (Key_GetFlag(0))
		{
			if (s_running)
			{
				Motor_Brake();
				Servo_SetValue(0);
				s_tracking_last_servo = 0;
				PID_Reset(&s_tracking_pid);
				s_running = false;
			}
			else
			{
				PID_Reset(&s_tracking_pid);
				PID_SetTarget(&s_tracking_pid, 0.0f);
				s_tracking_last_servo = 0;
				s_last_tracking       = now;
				s_running             = true;
			}
		}

		if (!s_running)
			continue;

		/* ======== 传感器读取 ======== */
		uint8_t mask    = Grayscale_ReadAll();
		bool    on_line = (mask != 0xFF);

		/* ======== 直道/弯道自适应速度 ======== */
		{
			int32_t abs_servo = (s_tracking_last_servo >= 0)
			                    ? s_tracking_last_servo
			                    : -s_tracking_last_servo;
			float base_speed = (abs_servo > SERVO_CURVE_THRESHOLD)
			                   ? SPEED_ARC
			                   : SPEED_STRAIGHT;

			float diff = (float)s_tracking_last_servo * ARC_DIFF_GAIN;
			float left_speed  = base_speed + diff;
			float right_speed = base_speed - diff;
			Motor_SetSpeedLR(left_speed, right_speed);
		}

		/* ======== 循迹 PID + 舵机（每 10ms） ======== */
		if (now - s_last_tracking >= TRACKING_CTRL_DT_MS)
		{
			s_last_tracking = now;

			float position = Tracking_CalcPosition(mask);

			if (position != 99.0f)
			{
				/* 正常循迹 */
				float   steering  = PID_Compute(&s_tracking_pid, -position);
				int32_t servo_out = (int32_t)steering;

				/* slew rate 限幅 */
				int32_t delta = servo_out - s_tracking_last_servo;
				if (delta > TRACKING_SLEW_MAX)
					servo_out = s_tracking_last_servo + TRACKING_SLEW_MAX;
				else if (delta < -TRACKING_SLEW_MAX)
					servo_out = s_tracking_last_servo - TRACKING_SLEW_MAX;

				s_tracking_last_servo = servo_out;
				Servo_SetValue(servo_out);
			}
			else
			{
				/* 全白丢线：舵机缓慢回中，防止卡在上次纠偏角度 */
				if (s_tracking_last_servo > TRACKING_SLEW_MAX)
					s_tracking_last_servo -= TRACKING_SLEW_MAX;
				else if (s_tracking_last_servo < -TRACKING_SLEW_MAX)
					s_tracking_last_servo += TRACKING_SLEW_MAX;
				else
					s_tracking_last_servo = 0;

				Servo_SetValue(s_tracking_last_servo);
			}
		}

		/* ======== 停车横线检测 ======== */
		/*
		 * A 点停车：所有传感器同时压黑线（mask == 0x00）。
		 * 在弧线段连续检测 STOP_LINE_DEBOUNCE 次确认停车。
		 *
		 * TODO: 启用前先确认赛道确实有横线，且传感器极性正确。
		 */
#if 0
		{
			static uint8_t stop_debounce = 0;

			if (mask == 0x00)
			{
				if (++stop_debounce >= STOP_LINE_DEBOUNCE)
				{
					Motor_Brake();
					Servo_SetValue(0);
					s_tracking_last_servo = 0;
					s_running = false;
					stop_debounce = 0;
				}
			}
			else
			{
				stop_debounce = 0;
			}
		}
#endif
	}
}
