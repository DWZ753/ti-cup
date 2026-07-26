#ifndef __GIMBAL_H
#define __GIMBAL_H

#include <stdint.h>
#include <stdbool.h>

/**********************************************************
*** 云台双环 PID 控制模块
***
*** 外环（位置环）：target_angle → PID → vel_cmd
*** 内环（速度环）：vel_cmd       → PID → Vel_Control()
***
*** PID 控制器为模块内部静态变量（参照 Angle 模块模式），
*** 调参通过 GimbalAxis_TunePosPID() / GimbalAxis_TuneVelPID()。
***
*** 反馈源：ZDT 电机的实时位置/转速（RS485 查询）
*** 输出：  ZDT_Motor_Vel_Control() 或 Stop_Now()
**********************************************************/

/* ========== 云台电机 ID ========== */

#define GIMBAL_ROLL_ID    1    /* 云台1（上方电机）— Roll 轴 */
#define GIMBAL_YAW_ID     2    /* 云台2（下方电机）— Yaw 轴 */

/* ========== 安全速度限制 ========== */

#define GIMBAL_SPEED_SLOW     50    /* 慢速测试 (RPM) */
#define GIMBAL_SPEED_NORMAL   80    /* 正常速度 (RPM) */
#define GIMBAL_ACCEL          10    /* 加速度（平滑启停） */

/* ========== 角度-脉冲换算（16 细分，3200 脉冲/圈） ========== */

#define GIMBAL_DEG_TO_PULSE(deg) \
    ((int32_t)((float)(deg) * 3200.0f / 360.0f))

/* ========== 默认 PID 参数（上赛道后实测调整） ========== */

#define GC_POS_KP_DEFAULT      8.0f
#define GC_POS_KI_DEFAULT      0.1f
#define GC_POS_KD_DEFAULT      0.0f
#define GC_POS_INT_LIMIT       50.0f
#define GC_POS_OUT_LIMIT       200.0f

#define GC_VEL_KP_DEFAULT      1.5f
#define GC_VEL_KI_DEFAULT      0.3f
#define GC_VEL_KD_DEFAULT      0.0f
#define GC_VEL_INT_LIMIT       100.0f
#define GC_VEL_OUT_LIMIT       800.0f

/* ========== 通用参数 ========== */

#define GC_POS_TOLERANCE_DEG   0.5f
#define GC_VEL_DEADBAND_RPM    5.0f

/* ========== 单轴控制结构体 ========== */

typedef struct {
    uint8_t  motor_id;
    bool     enabled;

    /* ---- 反馈（只读，Update 后更新） ---- */
    float    current_angle;
    float    current_vel;

    /* ---- 状态（只读，Update 后更新） ---- */
    float    target_angle;
    float    pos_error;
    float    vel_cmd;
    float    vel_error;
    float    motor_output;

    /* ---- 可调参数 ---- */
    uint16_t max_vel;
    float    pos_tolerance;
    uint16_t control_period_ms;

    /* ---- 模块内部（外部勿动） ---- */
    uint8_t  _slot;
} GimbalAxis;

/* ========== API ========== */

void GimbalAxis_Init(GimbalAxis *axis, uint8_t motor_id);

/**
 * @brief 实时调整位置环 PID 参数（改后立即生效，内部清零积分）
 */
void GimbalAxis_TunePosPID(GimbalAxis *axis, float kp, float ki, float kd);

/**
 * @brief 实时调整速度环 PID 参数
 */
void GimbalAxis_TuneVelPID(GimbalAxis *axis, float kp, float ki, float kd);

/**
 * @brief 读取当前生效的位置环 PID 参数
 */
void GimbalAxis_GetPosPID(GimbalAxis *axis, float *kp, float *ki, float *kd);

/**
 * @brief 读取当前生效的速度环 PID 参数
 */
void GimbalAxis_GetVelPID(GimbalAxis *axis, float *kp, float *ki, float *kd);

void GimbalAxis_SetTarget(GimbalAxis *axis, float angle_deg);
void GimbalAxis_Enable(GimbalAxis *axis, bool en);
void GimbalAxis_Update(GimbalAxis *axis);
bool GimbalAxis_IsAtTarget(GimbalAxis *axis);
void GimbalAxis_Stop(GimbalAxis *axis);
void GimbalAxis_Reset(GimbalAxis *axis);

/**
 * @brief 云台综合测试——按顺序测试 Roll/Yaw 两轴所有常用功能
 * @note  速度已限制在安全范围（≤100 RPM），小角度运动（≤45°）
 *        测试包含：使能、位置控制、快速位置、同步运动、
 *        读取位置/速度、急停、回零等功能
 */
void GimbalTest_Run(void);

#endif /* __GIMBAL_H */
