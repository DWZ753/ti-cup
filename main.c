/**
 * @file    main.c
 * @brief   2026 智能物流搬运 — 小车端主程序
 *
 * 双模式：
 *   LINE_FOLLOW — 灰度循迹自治（KEY1 开始/暂停）
 *   GUIDED       — Pi 接管差速控制（CMD_GUIDE）
 *
 * Pi 有线 UART，COBS 协议，命令码见 protocol_commands.h。
 * GUIDED 模式下 200ms 无 Pi 指令 → 自动停车 + 上报 CMD_FAULT(1)。
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
#include "protocol.h"
#include "protocol_commands.h"
#include "gimbal.h"

/* ========== 循迹参数 ========== */

#define TRACK_SPEED_MM_S     200.0f
#define TRACK_KP             100.0f
#define TRACK_KI             0.5f
#define TRACK_KD             0.0f
#define TRACK_INTEGRAL_LIMIT 20.0f
#define TRACK_OUTPUT_LIMIT   100.0f
#define TRACK_SLEW_MAX       6

/* ========== 电磁铁 ========== */

#define MAGNET_PORT  GPIO_MAGNETs_PORT
#define MAGNET_PIN   GPIO_MAGNETs_GPIO_MAGNET_PIN

/* ========== GUIDED 模式 ========== */

#define GUIDE_MAX_SPEED_MM_S  2136.0f  /**< speed=100 → 最大速度（= MOTOR_MAX_SPEED_MM_S） */
#define GUIDE_WATCHDOG_MS     200      /**< 无 Pi 指令超时自动停车 */

/* ========== 模式 ========== */

typedef enum {
	MODE_IDLE,          /**< 待命，电机停止 */
	MODE_LINE_FOLLOW,   /**< 灰度循迹自治 */
	MODE_GUIDED,        /**< Pi 接管差速控制 */
} run_mode_e;

/* ========== 状态 ========== */

static PID_Controller s_track_pid;
static run_mode_e     s_mode;
static int32_t        s_last_servo;
static bool           s_magnet_on;

/* Pi 引导状态 */
static int8_t  s_guide_speed;   /**< 最近一次 CMD_GUIDE speed */
static int8_t  s_guide_diff;    /**< 最近一次 CMD_GUIDE diff */
static uint32_t s_last_pi_ms;   /**< 最近一次收到 Pi 指令的时间戳 */

/* ========== Pi 协议回调 ========== */

/**
 * @brief 收到 Pi 帧时由 Protocol_Update 同步调用
 */
static void on_pi_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	s_last_pi_ms = Board_GetTickMs();

	switch (cmd)
	{
	case CMD_GUIDE:
		if (len >= 2)
		{
			s_guide_speed = (int8_t)payload[0];
			s_guide_diff  = (int8_t)payload[1];
			s_mode = MODE_GUIDED;
		}
		break;

	case CMD_STOP:
		Motor_Stop();
		Servo_SetValue(0);
		s_last_servo = 0;
		/* 留在 GUIDED 模式，等 Pi 发下一个指令 */
		break;

	case CMD_MAGNET_ON:
		DL_GPIO_setPins(MAGNET_PORT, MAGNET_PIN);
		s_magnet_on = true;
		break;

	case CMD_MAGNET_OFF:
		DL_GPIO_clearPins(MAGNET_PORT, MAGNET_PIN);
		s_magnet_on = false;
		break;

	case CMD_GIMBAL:
		if (len >= 2)
		{
			int16_t angle = (int16_t)(((uint16_t)payload[0] << 8)
			                        | payload[1]);
			Gimbal_SetAngle((float)angle);
		}
		break;

	case CMD_RESUME_LINE:
		Motor_Stop();
		Servo_SetValue(0);
		s_last_servo = 0;
		PID_Reset(&s_track_pid);
		s_mode = MODE_LINE_FOLLOW;
		break;

	default:
		break;
	}
}

/* ========== 控制回调（PIT 每 20ms 调用） ========== */

static void Control_Tick(void)
{
	uint8_t mask;
	float   position;
	float   steering;
	int32_t servo_out;
	int32_t delta;
	float   left_speed;
	float   right_speed;

	switch (s_mode)
	{
	case MODE_IDLE:
		return;

	case MODE_LINE_FOLLOW:
		mask     = Grayscale_ReadAll();
		position = Tracking_CalcPosition(mask);

		if (position == 99.0f)
		{
			servo_out = 0;
			Motor_SetSpeed(TRACK_SPEED_MM_S * 0.4f);
		}
		else
		{
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
		break;

	case MODE_GUIDED:
		/*
		 * Pi 控制差速：speed/diff 为百分比 [-100, +100]
		 * left  = speed + diff
		 * right = speed - diff
		 */
		left_speed  = (float)(s_guide_speed + s_guide_diff)
		              * GUIDE_MAX_SPEED_MM_S / 100.0f;
		right_speed = (float)(s_guide_speed - s_guide_diff)
		              * GUIDE_MAX_SPEED_MM_S / 100.0f;
		Motor_SetSpeedLR(left_speed, right_speed);

		/* 前轮舵机回中（GUIDED 下后轮差速转向，不用前轮） */
		Servo_SetValue(0);
		s_last_servo = 0;
		break;
	}
}

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	/* 初始化 Pi 协议 */
	Protocol_Init(on_pi_frame, Board_GetPiUART());

	/* 初始化循迹 PID */
	PID_Init(&s_track_pid,
	         TRACK_KP, TRACK_KI, TRACK_KD,
	         TRACK_INTEGRAL_LIMIT, TRACK_OUTPUT_LIMIT);
	PID_SetTarget(&s_track_pid, 0.0f);

	/* 注册控制回调 */
	PIT_Control_Tick_RegisterCallback(Control_Tick);

	/* 初始状态 */
	s_mode        = MODE_GUIDED;
	s_last_servo  = 0;
	s_magnet_on   = false;
	s_last_pi_ms  = Board_GetTickMs();
	DL_GPIO_clearPins(MAGNET_PORT, MAGNET_PIN);

	while (1)
	{
		uint32_t now = Board_GetTickMs();

		/* 收 Pi 帧 → 回调 on_pi_frame */
		Protocol_Update();

		/* GUIDED 看门狗：200ms 无指令 → 自动停车 */
		if (s_mode == MODE_GUIDED
		 && (now - s_last_pi_ms) > GUIDE_WATCHDOG_MS)
		{
			Motor_Stop();
			Servo_SetValue(0);
			s_last_servo = 0;

			/* 上报故障 */
			uint8_t err = FAULT_TIMEOUT;
			Protocol_SendFrame(CMD_FAULT, &err, 1);

			s_mode = MODE_IDLE;
		}

		/* ---- 按键 ---- */

		/* KEY1：启动循迹 / 停止（全模式有效） */
		if (Key_GetFlag(0))
		{
			if (s_mode == MODE_LINE_FOLLOW)
			{
				Motor_Stop();
				Servo_SetValue(0);
				s_last_servo = 0;
				s_mode = MODE_IDLE;
			}
			else
			{
				PID_Reset(&s_track_pid);
				s_last_servo = 0;
				s_mode = MODE_LINE_FOLLOW;
			}
		}

		/* KEY2：电磁铁翻转（任意模式） */
		if (Key_GetFlag(1))
		{
			s_magnet_on = !s_magnet_on;
			if (s_magnet_on)
				DL_GPIO_setPins(MAGNET_PORT, MAGNET_PIN);
			else
				DL_GPIO_clearPins(MAGNET_PORT, MAGNET_PIN);
		}

		/* KEY3：强制退出 GUIDED → IDLE */
		if (Key_GetFlag(2))
		{
			if (s_mode == MODE_GUIDED)
			{
				Motor_Stop();
				Servo_SetValue(0);
				s_last_servo = 0;
				s_mode = MODE_IDLE;
			}
		}

		/* KEY4：云台角度测试（0° → 20° → 45° 循环） */
		if (Key_GetFlag(3))
		{
			static uint8_t gimbal_step = 0;
			static const float gimbal_angles[] = {0.0f, 20.0f, 45.0f};

			gimbal_step = (gimbal_step + 1) % 3;
			Gimbal_SetAngle(gimbal_angles[gimbal_step]);
		}
	}
}
