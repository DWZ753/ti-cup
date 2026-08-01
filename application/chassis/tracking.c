/**
 * @file    tracking.c
 * @brief   循迹模块实现 — 位置解算 + 循迹控制 + 停车检测
 */

#include "tracking.h"
#include "chassis_config.h"
#include "pid.h"
#include "motor.h"
#include "servo.h"
#include "board.h"

/* ========== 内部状态 ========== */

static PID_Controller s_tracking_pid;
static int32_t        s_tracking_last_servo;   // slew rate 历史
static uint32_t       s_last_tracking;         // 上次循迹时间
static bool           s_running;               // 是否循迹中

/* 运行时速度/差速参数（初始化为默认值，可通过 Tracking_SetSpeedParams 切换） */
static float s_speed_straight = TASK5_SPEED_STRAIGHT;
static float s_speed_arc      = TASK5_SPEED_ARC;
static float s_diff_gain      = TASK5_DIFF_GAIN;

/* 速度斜坡 */
static float    s_current_speed;     // 当前斜坡速度 (mm/s)
static uint32_t s_last_ramp_ms;      // 上次斜坡计算时间
static bool     s_speed_ramp = true; // 渐加速/渐减速开关（默认开）

/* AB段：首次转弯自动停车 */
static bool  s_stop_on_curve  = false;

/* ========== 位置解算 ========== */

/**
 * @brief 加权平均法黑线位置解算
 * @param mask 8 位掩码，bit[i]=1 白/未压线，bit[i]=0 黑/压线
 * @return [-1.0f, +1.0f] 负左正右，0 居中；99.0f = 丢线
 */
float Tracking_CalcPosition(uint8_t mask)
{
	uint8_t bits = ~mask;   /* 取反后 1 = 黑线 */
	float   sum_weight = 0;
	float   sum_index  = 0;

	static const float s_weight[SENSOR_COUNT] = {
		TRACKING_W0, TRACKING_W1, TRACKING_W2, TRACKING_W3,
		TRACKING_W4, TRACKING_W5, TRACKING_W6, TRACKING_W7
	};

	for (uint8_t i = 0; i < SENSOR_COUNT; i++)
	{
		if (bits & (1 << i))
		{
			float w = s_weight[i];
			sum_index  += (float)i * w;
			sum_weight += w;
		}
	}

	if (sum_weight == 0.0f)
		return 99.0f;

	float center   = sum_index / sum_weight;
	float position = (center - SENSOR_CENTER) / SENSOR_CENTER;

	/*
	 * 非线性映射：中段线性，边缘柔和
	 *   POWER=1 → 完全线性
	 *   POWER=2 → 平方（边缘柔和，落差 26%）
	 *   POWER=3 → 立方（过于扁平，落差 37% 容易振荡）
	 */
#if TRACKING_CURVE_POWER >= 2
	{
		float abs_pos = (position >= 0.0f) ? position : -position;
		position = position * abs_pos;               /* power=2: sign * |pos|^2 */
	#if TRACKING_CURVE_POWER >= 3
		position = position * abs_pos;               /* power=3: sign * |pos|^3 */
	#endif
	}
#endif
	return position;
}

/* ========== 循迹控制 API ========== */

/**
 * @brief 初始化循迹模块：配置 PID 参数并复位内部状态
 */
void Tracking_Init(void)
{
	PID_Init(&s_tracking_pid,
	         TRACKING_KP, TRACKING_KI, TRACKING_KD,
	         TRACKING_INT_LIMIT, TRACKING_OUT_LIMIT);
	PID_SetTarget(&s_tracking_pid, 0.0f);

	s_running             = false;
	s_tracking_last_servo = 0;
	s_last_tracking       = 0;
	s_current_speed       = 0.0f;
	s_last_ramp_ms        = 0;
}

/**
 * @brief 启动循迹：复位 PID、清零舵机历史、记录起始时间
 */
void Tracking_Start(void)
{
	PID_Reset(&s_tracking_pid);
	PID_SetTarget(&s_tracking_pid, 0.0f);
	s_tracking_last_servo = 0;
	s_last_tracking       = Board_GetTickMs();
	s_current_speed       = 0.0f;
	s_last_ramp_ms        = Board_GetTickMs();
	s_running             = true;
}

/**
 * @brief 停止循迹：刹车、舵机回中、PID 复位
 */
void Tracking_Stop(void)
{
	Motor_Brake();
	Servo_SetValue(0);
	s_tracking_last_servo = 0;
	PID_Reset(&s_tracking_pid);
	s_running = false;
}

/**
 * @brief 查询当前是否处于循迹运行状态
 * @return true 循迹中，false 已停止
 */
bool Tracking_IsRunning(void)
{
	return s_running;
}

/**
 * @brief 运行时切换速度/差速参数
 * @param straight  直道速度 (mm/s)
 * @param arc       弯道速度 (mm/s)
 * @param diff_gain 差速增益 (mm/s per servo unit)
 */
void Tracking_SetSpeedParams(float straight, float arc, float diff_gain)
{
	s_speed_straight = straight;
	s_speed_arc      = arc;
	s_diff_gain      = diff_gain;
}

/**
 * @brief AB段模式：首次转弯自动停车
 * @param enable true=检测到首次转弯时自动 Tracking_Stop()
 */
void Tracking_SetStopOnCurve(bool enable)
{
	s_stop_on_curve = enable;
}

/**
 * @brief 渐加速/渐减速开关
 * @param enable true=速度斜坡生效，false=直接设目标速度
 */
void Tracking_SetSpeedRamp(bool enable)
{
	s_speed_ramp = enable;
}

/**
 * @brief 循迹主更新函数，每主循环周期调用一次
 *
 * 内部按需执行三项任务：
 *   1. 自适应速度 + 差速驱动（每次调用）
 *   2. 循迹 PID + 舵机控制（每 TRACKING_CTRL_DT_MS）
 *   3. 停车横线检测 + 防抖
 *
 * @param now_ms 当前系统时间戳 (ms)，来自 Board_GetTickMs()
 * @param mask   灰度传感器原始读数，来自 Grayscale_ReadAll()
 */
void Tracking_Update(uint32_t now_ms, uint8_t mask)
{
	if (!s_running)
		return;

	/* ======== 直道/弯道自适应速度 + 渐加/减速斜坡 ======== */
	{
		int32_t abs_servo = (s_tracking_last_servo >= 0)
		                    ? s_tracking_last_servo
		                    : -s_tracking_last_servo;
		float target_speed = (abs_servo > SERVO_CURVE_THRESHOLD)
		                     ? s_speed_arc
		                     : s_speed_straight;

		/* 速度斜坡：渐加速/渐减速 */
		if (s_speed_ramp)
		{
			float dt = (float)(now_ms - s_last_ramp_ms) * 0.001f;
			if (dt > 0.0f && dt < 0.5f)
			{
				if (target_speed > s_current_speed)
				{
					s_current_speed += TRACK_SPEED_RAMP_UP * dt;
					if (s_current_speed > target_speed)
						s_current_speed = target_speed;
				}
				else if (target_speed < s_current_speed)
				{
					s_current_speed -= TRACK_SPEED_RAMP_DOWN * dt;
					if (s_current_speed < target_speed)
						s_current_speed = target_speed;
				}
			}
			else
			{
				s_current_speed = target_speed;  /* 首次或异常：直设目标 */
			}
			s_last_ramp_ms = now_ms;
		}
		else
		{
			s_current_speed = target_speed;  /* 无斜坡：直设目标 */
		}

		float diff = (float)s_tracking_last_servo * s_diff_gain;
		float left_speed  = s_current_speed + diff;
		float right_speed = s_current_speed - diff;
		Motor_SetSpeedLR(left_speed, right_speed);

		/* AB段：首次转弯 → 自动停车 */
		if (s_stop_on_curve && abs_servo > SERVO_CURVE_THRESHOLD)
		{
			Tracking_Stop();
			return;
		}
	}

	/* ======== 循迹 PID + 舵机（每 10ms） ======== */
	if (now_ms - s_last_tracking >= TRACKING_CTRL_DT_MS)
	{
		s_last_tracking = now_ms;

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
	 * A 点停车：所有传感器同时压黑线((mask & 0xFC) == 0x00)。
	 * 在弧线段连续检测 STOP_LINE_DEBOUNCE 次确认停车。
	 *
	 * TODO: 启用前先确认赛道确实有横线，且传感器极性正确。
	 */
	static uint8_t stop_debounce = 0;

	if ((mask & 0xFC) == 0x00)
	{
		if (++stop_debounce >= STOP_LINE_DEBOUNCE)
		{
			Tracking_Stop();
			stop_debounce = 0;
		}
	}
	else
	{
		stop_debounce = 0;
	}
}
