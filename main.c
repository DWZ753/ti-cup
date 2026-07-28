/**
 * @file    main.c
 * @brief   循迹测试 — 上电待命，按 KEY1 开始/暂停
 *
 * 控制周期：20ms（PIT_Control_Tick）
 * 转向方式：舵机（Ackermann），灰度 PID → Servo_SetValue
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "pit_control_tick.h"
#include "grayscale.h"
#include "motor.h"
#include "servo.h"
#include "key.h"
#include "pid.h"
#include "tracking.h"

/* ========== 循迹参数 ========== */

#define TRACK_SPEED_MM_S    500.0f   /**< 循迹前进速度 */
#define TRACK_KP            100.0f   /**< 循迹 P 增益 */
#define TRACK_KI            0.5f     /**< 循迹 I 增益 */
#define TRACK_KD            0.0f     /**< 循迹 D 增益 */
#define TRACK_INTEGRAL_LIMIT 20.0f   /**< 积分限幅 */
#define TRACK_OUTPUT_LIMIT  100.0f   /**< 舵机输出限幅 */
#define TRACK_SLEW_MAX      6        /**< 舵机每帧最大变化量 */

/* ========== 状态 ========== */

static PID_Controller s_track_pid;
static bool           s_running;
static int32_t        s_last_servo;

/* ========== 控制回调（PIT 每 20ms 调用） ========== */

static void Control_Tick(void)
{
	uint8_t mask;
	float   position;
	float   steering;
	int32_t servo_out;
	int32_t delta;

	if (!s_running)
		return;

	mask     = Grayscale_ReadAll();
	position = Tracking_CalcPosition(mask);

	if (position == 99.0f)
	{
		/* 丢线：舵机缓慢回中，低速前进 */
		servo_out = 0;
		Motor_SetSpeed(TRACK_SPEED_MM_S * 0.4f);
	}
	else
	{
		/*
		 * PID 目标 = 0（线居中）
		 * position > 0（线偏右）→ -position < 0 → PID 输出负值 → 舵机左转修正
		 */
		steering  = PID_Compute(&s_track_pid, -position);
		servo_out = (int32_t)steering;
		Motor_SetSpeed(TRACK_SPEED_MM_S);
	}

	/* slew rate 限幅 */
	delta = servo_out - s_last_servo;
	if (delta > TRACK_SLEW_MAX)
		servo_out = s_last_servo + TRACK_SLEW_MAX;
	else if (delta < -TRACK_SLEW_MAX)
		servo_out = s_last_servo - TRACK_SLEW_MAX;

	s_last_servo = servo_out;
	Servo_SetValue(servo_out);
}

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	/* 初始化循迹 PID */
	PID_Init(&s_track_pid,
	         TRACK_KP, TRACK_KI, TRACK_KD,
	         TRACK_INTEGRAL_LIMIT, TRACK_OUTPUT_LIMIT);
	PID_SetTarget(&s_track_pid, 0.0f);

	/* 注册控制回调 */
	PIT_Control_Tick_RegisterCallback(Control_Tick);

	/* 初始状态：待命，电机停止 */
	s_running    = false;
	s_last_servo = 0;

	while (1)
	{
		/*
		 * KEY1：开始/暂停循迹
		 * Key_GetFlag 由 PIT 中断中 Key_TickHandler 置位，主循环轮询
		 */
		if (Key_GetFlag(0))
		{
			s_running = !s_running;
			if (s_running)
			{
				PID_Reset(&s_track_pid);
				s_last_servo = 0;
			}
			else
			{
				Motor_Stop();
				Servo_SetValue(0);
				s_last_servo = 0;
			}
		}
	}
}
