/**
 * @file    gimbal.c
 * @brief   单轴云台控制实现 — ZDT 闭环步进电机封装
 *
 * 电机地址：1（默认 ID）
 * 通信接口：UART（ZDT_Motor_Init 自注册，见 zdt_motor.h）
 */

#include "gimbal.h"
#include "zdt_motor.h"
#include "delay.h"

/* ========== 内部状态 ========== */

#define GIMBAL_MOTOR_ID  1

static float s_angle;   /**< 当前目标角度（°），QPos 为相对运动，需软件记位置 */

/* ========== 初始化 ========== */

void Gimbal_Init(void)
{
	/* 1. 初始化 ZDT 电机通信（内部 UART 自注册 + 500ms 等待驱动板启动） */
	ZDT_Motor_Init();

	/* 2. 配置硬停回零参数（不存储到 flash，仅本次运行有效）
	 *    mode=2: 硬停回零（电机转动直到机械碰撞触发停转）
	 *    o_dir: 方向待实测确认（0=CW, 1=CCW） */
	ZDT_Motor_Origin_Modify_Params(GIMBAL_MOTOR_ID, false,
	                               2,    /* o_mode = 硬停回零 */
	                               0,    /* o_dir  = 待实测 */
	                               GIMBAL_ZERO_VEL,
	                               10000,/* o_tm = 10s 超时 */
	                               GIMBAL_ZERO_VEL,
	                               GIMBAL_ZERO_CUR_MA,
	                               GIMBAL_ZERO_TIME_MS,
	                               false /* potF = 不上电自动回零 */);

	/* 3. 配置过流保护（不存储） */
	ZDT_Motor_Modify_Otocp(GIMBAL_MOTOR_ID, false,
	                       100,             /* otp = 100°C */
	                       GIMBAL_OCP_MA,
	                       GIMBAL_OCP_TIME_MS);

	/* 4. 使能电机 */
	ZDT_Motor_En_Control(GIMBAL_MOTOR_ID, true, false);

	/* 5. 预设快速位置模式参数（之后用 QPos_Control 只需发脉冲数） */
	ZDT_Motor_Set_QPos_Params(GIMBAL_MOTOR_ID,
	                          GIMBAL_WORK_VEL,
	                          5,    /* acc = 5，轻微加减速 */
	                          1,    /* raF = 绝对位置运动 */
	                          false /* snF = 不同步 */);

	/* 6. 触发硬停回零 */
	Gimbal_Rehome();
}

/* ========== 角度控制 ========== */

void Gimbal_SetAngle(float angle_deg)
{
	float   delta_deg;
	int32_t delta_pulses;

	/* 钳位到安全范围 */
	if (angle_deg < 0.0f)
		angle_deg = 0.0f;
	if (angle_deg > GIMBAL_MAX_ANGLE_DEG)
		angle_deg = GIMBAL_MAX_ANGLE_DEG;

	/* QPos 是相对运动 → 计算目标与当前角度的差值 */
	delta_deg    = angle_deg - s_angle;
	delta_pulses = (int32_t)(delta_deg * GIMBAL_PULSE_PER_DEG);

	if (delta_pulses != 0)
	{
		ZDT_Motor_QPos_Control(GIMBAL_MOTOR_ID, delta_pulses);
		s_angle = angle_deg;
	}
}

/* ========== 状态查询 ========== */

bool Gimbal_IsIdle(void)
{
	/* 读取位置误差，非 0 表示仍在运动中 */
	ZDT_Motor_Read_Sys_Params(GIMBAL_MOTOR_ID, ZDT_PARAM_PERR);
	/*
	 * TODO: 需要解析 ZDT 返回数据判断 PERR 是否为 0。
	 * 当前 ZDT 模块的 rx_cmd[] 接收完成后需手动解析，
	 * 协议格式：
	 *   响应: [addr] [cmd=0x37] [perr_bit31..24] [23..16] [15..8] [7..0] [0x6B]
	 *   解析为 int32_t，== 0 表示已到位。
	 */
	return false;
}

/* ========== 紧急停止 ========== */

void Gimbal_Stop(void)
{
	ZDT_Motor_Stop_Now(GIMBAL_MOTOR_ID, false);
}

/* ========== 重新归零 ========== */

void Gimbal_Rehome(void)
{
	/*
	 * 触发硬停回零：电机下转直到机械臂碰地面 → 电流尖峰 → 自动停止
	 * 阻塞等待回零完成（硬停回零自动检测碰撞停转，无需手动轮询）
	 */
	ZDT_Motor_Origin_Trigger_Return(GIMBAL_MOTOR_ID, 2, false);

	/* 等待回零完成（硬停回零自动停转，延时需实测）
	 * 超时保护：最多等 10 秒 */
	delay_ms(3000);

		/* 将当前位置设为原点 */
	ZDT_Motor_Reset_CurPos_To_Zero(GIMBAL_MOTOR_ID);
	s_angle = 0.0f;
}
