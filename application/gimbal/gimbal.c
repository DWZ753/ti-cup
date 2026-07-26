#include "gimbal.h"
#include "pid.h"
#include "zdt_motor.h"
#include "delay.h"

/**********************************************************
*** 云台双环 PID 控制模块
***
*** 参照 Angle 模块模式：PID_Controller 为模块内部静态数组，
*** 外部通过 GimbalAxis_TunePosPID() / GimbalAxis_TuneVelPID()
*** 直接操作内部 PID 池的 Kp/Ki/Kd，不经过结构体字段中转。
**********************************************************/

#define GC_MAX_AXES  4

static PID_Controller s_pos_pid[GC_MAX_AXES];
static PID_Controller s_vel_pid[GC_MAX_AXES];
static uint8_t s_next_slot;

/* ========== 内部辅助 ========== */

static float angle_error(float target, float current)
{
    float err = target - current;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

static int read_angle(uint8_t addr, float *deg)
{
    uint32_t raw;

    ZDT_Motor_Read_Sys_Params(addr, ZDT_PARAM_CPOS);
    delay_ms(15);
    ZDT_Motor_GetRxCmd();

    if (zdt_motor_rx_cmd[0] != addr
        || zdt_motor_rx_cmd[1] != 0x36
        || zdt_motor_rx_count != 8)
        {
        return -1;
    }

    raw = ((uint32_t)zdt_motor_rx_cmd[3] << 24)
        | ((uint32_t)zdt_motor_rx_cmd[4] << 16)
        | ((uint32_t)zdt_motor_rx_cmd[5] << 8)
        | ((uint32_t)zdt_motor_rx_cmd[6] << 0);

    *deg = (float)raw * 360.0f / 65536.0f;
    if (zdt_motor_rx_cmd[2]) *deg = -*deg;
    return 0;
}

static int read_velocity(uint8_t addr, float *rpm)
{
    uint16_t raw;

    ZDT_Motor_Read_Sys_Params(addr, ZDT_PARAM_VEL);
    delay_ms(15);
    ZDT_Motor_GetRxCmd();

    if (zdt_motor_rx_cmd[0] != addr
        || zdt_motor_rx_cmd[1] != 0x35
        || zdt_motor_rx_count != 6)
        {
        return -1;
    }

    raw = ((uint16_t)zdt_motor_rx_cmd[3] << 8)
        | ((uint16_t)zdt_motor_rx_cmd[4] << 0);

    *rpm = (float)raw;
    if (zdt_motor_rx_cmd[2]) *rpm = -*rpm;
    return 0;
}

/** 参照 pid.md Angle 模块用法：target - error 作为等价反馈 */
static float pos_pid_compute(uint8_t slot, float target_deg, float current_deg)
{
    float err = angle_error(target_deg, current_deg);
    float feedback = target_deg - err;

    PID_SetTarget(&s_pos_pid[slot], target_deg);
    return PID_Compute(&s_pos_pid[slot], feedback);
}

static float vel_pid_compute(uint8_t slot, float target_vel, float current_vel)
{
    PID_SetTarget(&s_vel_pid[slot], target_vel);
    return PID_Compute(&s_vel_pid[slot], current_vel);
}

/* ======================================================================== */
/*  API                                                                     */
/* ======================================================================== */

void GimbalAxis_Init(GimbalAxis *axis, uint8_t motor_id)
{
    if (s_next_slot >= GC_MAX_AXES)
    {
        axis->_slot = 0xFF;
        return;
    }

    axis->motor_id = motor_id;
    axis->enabled  = false;
    axis->_slot    = s_next_slot;
    ++s_next_slot;

    axis->current_angle = 0.0f;
    axis->current_vel   = 0.0f;
    axis->target_angle  = 0.0f;
    axis->pos_error     = 0.0f;
    axis->vel_cmd       = 0.0f;
    axis->vel_error     = 0.0f;
    axis->motor_output  = 0.0f;
    axis->max_vel       = 200;
    axis->pos_tolerance = GC_POS_TOLERANCE_DEG;
    axis->control_period_ms = 40;

    /* 向内部 PID 池注册（使用默认参数） */
    PID_Init(&s_pos_pid[axis->_slot],
             GC_POS_KP_DEFAULT, GC_POS_KI_DEFAULT, GC_POS_KD_DEFAULT,
             GC_POS_INT_LIMIT, GC_POS_OUT_LIMIT);
    PID_SetTarget(&s_pos_pid[axis->_slot], 0.0f);

    PID_Init(&s_vel_pid[axis->_slot],
             GC_VEL_KP_DEFAULT, GC_VEL_KI_DEFAULT, GC_VEL_KD_DEFAULT,
             GC_VEL_INT_LIMIT, GC_VEL_OUT_LIMIT);
    PID_SetTarget(&s_vel_pid[axis->_slot], 0.0f);
}

void GimbalAxis_TunePosPID(GimbalAxis *axis, float kp, float ki, float kd)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF) return;

    s_pos_pid[slot].Kp = kp;
    s_pos_pid[slot].Ki = ki;
    s_pos_pid[slot].Kd = kd;
    PID_Reset(&s_pos_pid[slot]);
}

void GimbalAxis_TuneVelPID(GimbalAxis *axis, float kp, float ki, float kd)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF) return;

    s_vel_pid[slot].Kp = kp;
    s_vel_pid[slot].Ki = ki;
    s_vel_pid[slot].Kd = kd;
    PID_Reset(&s_vel_pid[slot]);
}

void GimbalAxis_GetPosPID(GimbalAxis *axis, float *kp, float *ki, float *kd)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF)
    {
        *kp = 0.0f;
        *ki = 0.0f;
        *kd = 0.0f;
        return;
    }
    *kp = s_pos_pid[slot].Kp;
    *ki = s_pos_pid[slot].Ki;
    *kd = s_pos_pid[slot].Kd;
}

void GimbalAxis_GetVelPID(GimbalAxis *axis, float *kp, float *ki, float *kd)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF)
    {
        *kp = 0.0f;
        *ki = 0.0f;
        *kd = 0.0f;
        return;
    }
    *kp = s_vel_pid[slot].Kp;
    *ki = s_vel_pid[slot].Ki;
    *kd = s_vel_pid[slot].Kd;
}

void GimbalAxis_SetTarget(GimbalAxis *axis, float angle_deg)
{
    axis->target_angle = angle_deg;
    while (axis->target_angle > 180.0f)  axis->target_angle -= 360.0f;
    while (axis->target_angle < -180.0f) axis->target_angle += 360.0f;
}

void GimbalAxis_Enable(GimbalAxis *axis, bool en)
{
    if (axis->_slot == 0xFF) return;
    if (en != axis->enabled)
    {
        ZDT_Motor_En_Control(axis->motor_id, en, false);
        delay_ms(15);
        ZDT_Motor_GetRxCmd();
        if (!en)
        {
            ZDT_Motor_Stop_Now(axis->motor_id, false);
            delay_ms(15);
            ZDT_Motor_GetRxCmd();
        }
        axis->enabled = en;
    }
}

void GimbalAxis_Update(GimbalAxis *axis)
{
    float pos_err, vel_cmd, output;
    float abs_output;
    uint8_t dir;
    uint8_t slot = axis->_slot;

    if (slot == 0xFF || !axis->enabled) return;

    /* ---- 1. 读取反馈 ---- */
    if (read_angle(axis->motor_id, &axis->current_angle) != 0) return;
    if (read_velocity(axis->motor_id, &axis->current_vel) != 0) return;

    /* ---- 2. 外环：位置 PID（参照 pid.md Angle 模块用法） ---- */
    pos_err = angle_error(axis->target_angle, axis->current_angle);
    axis->pos_error = pos_err;

    vel_cmd = pos_pid_compute(slot, axis->target_angle,
                              axis->current_angle);

    if (vel_cmd > (float)axis->max_vel)       vel_cmd = (float)axis->max_vel;
    else if (vel_cmd < -(float)axis->max_vel) vel_cmd = -(float)axis->max_vel;
    axis->vel_cmd = vel_cmd;

    /* ---- 3. 内环：速度 PID ---- */
    axis->vel_error = vel_cmd - axis->current_vel;

    output = vel_pid_compute(slot, vel_cmd, axis->current_vel);

    if (output > (float)axis->max_vel)       output = (float)axis->max_vel;
    else if (output < -(float)axis->max_vel) output = -(float)axis->max_vel;
    axis->motor_output = output;

    /* ---- 4. 死区与输出 ---- */
    abs_output = ABS(output);

    if (abs_output < GC_VEL_DEADBAND_RPM)
    {
        if (ABS(pos_err) < axis->pos_tolerance)
        {
            ZDT_Motor_Stop_Now(axis->motor_id, false);
            PID_Reset(&s_vel_pid[slot]);
        } else
        {
            dir = (pos_err > 0.0f) ? 0 : 1;
            ZDT_Motor_Vel_Control(axis->motor_id, dir,
                                  (uint16_t)GC_VEL_DEADBAND_RPM,
                                  0, false);
        }
    } else
    {
        dir = (output > 0.0f) ? 0 : 1;
        ZDT_Motor_Vel_Control(axis->motor_id, dir,
                              (uint16_t)abs_output, 0, false);
    }

    delay_ms(10);
    ZDT_Motor_GetRxCmd();
}

bool GimbalAxis_IsAtTarget(GimbalAxis *axis)
{
    return ABS(axis->pos_error) < axis->pos_tolerance;
}

void GimbalAxis_Stop(GimbalAxis *axis)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF) return;

    ZDT_Motor_Stop_Now(axis->motor_id, false);
    delay_ms(10);
    ZDT_Motor_GetRxCmd();
    PID_Reset(&s_pos_pid[slot]);
    PID_Reset(&s_vel_pid[slot]);
    axis->motor_output = 0.0f;
}

void GimbalAxis_Reset(GimbalAxis *axis)
{
    uint8_t slot = axis->_slot;
    if (slot == 0xFF) return;

    PID_Reset(&s_pos_pid[slot]);
    PID_Reset(&s_vel_pid[slot]);
    axis->pos_error    = 0.0f;
    axis->vel_cmd      = 0.0f;
    axis->vel_error    = 0.0f;
    axis->motor_output = 0.0f;
}

/* ======================================================================== */
/*  综合测试                                                                */
/* ======================================================================== */

/**
 * @brief 等待电机响应并清除 FIFO
 */
static void test_flush_rx(void)
{
    delay_ms(15);
    ZDT_Motor_GetRxCmd();
}

/**
 * @brief 安全位置移动：发送命令 → 等待到位 → 读取确认
 * @param addr   电机 ID
 * @param dir    方向（0=CW, 1=CCW）
 * @param deg    目标角度（绝对值，内部转脉冲）
 * @param speed  速度(RPM)
 */
static void test_safe_move_to(uint8_t addr, uint8_t dir, float deg,
                              uint16_t speed)
{
    uint32_t pulses;
    uint32_t wait_ms;
    float actual_deg;

    pulses = (uint32_t)GIMBAL_DEG_TO_PULSE(deg);

    ZDT_Motor_Pos_Control(addr, dir, speed, GIMBAL_ACCEL,
                          pulses, 0, 0);
    test_flush_rx();

    /* 估算等待时间 */
    wait_ms = (uint32_t)(deg * 60.0f * 1000.0f / ((float)speed * 360.0f));
    wait_ms += 500;
    if (wait_ms < 1000) wait_ms = 1000;
    delay_ms(wait_ms);

    /* 读取确认 */
    if (read_angle(addr, &actual_deg) == 0)
    {
        (void)actual_deg;
    }
}

void GimbalTest_Run(void)
{
    float deg1, deg2;
    float rpm1;

    /* ---- 阶段 0：读取电机配置（验证通信正常） ---- */
    ZDT_Motor_Read_Motor_Conf_Params(GIMBAL_ROLL_ID);
    test_flush_rx();
    ZDT_Motor_Read_Motor_Conf_Params(GIMBAL_YAW_ID);
    test_flush_rx();
    ZDT_Motor_Read_System_State_Params(GIMBAL_ROLL_ID);
    test_flush_rx();
    ZDT_Motor_Read_System_State_Params(GIMBAL_YAW_ID);
    test_flush_rx();

    /* ---- 阶段 1：使能两轴 ---- */
    ZDT_Motor_En_Control(GIMBAL_ROLL_ID, true, false);
    test_flush_rx();
    ZDT_Motor_En_Control(GIMBAL_YAW_ID, true, false);
    test_flush_rx();

    /* ---- 阶段 2：位置归零 ---- */
    ZDT_Motor_Reset_CurPos_To_Zero(GIMBAL_ROLL_ID);
    test_flush_rx();
    ZDT_Motor_Reset_CurPos_To_Zero(GIMBAL_YAW_ID);
    test_flush_rx();

    /* ---- 阶段 3：Roll 轴单轴测试 ---- */
    test_safe_move_to(GIMBAL_ROLL_ID, 0, 15.0f, GIMBAL_SPEED_SLOW);
    read_angle(GIMBAL_ROLL_ID, &deg1);
    test_safe_move_to(GIMBAL_ROLL_ID, 1, 15.0f, GIMBAL_SPEED_SLOW);
    test_safe_move_to(GIMBAL_ROLL_ID, 1, 15.0f, GIMBAL_SPEED_SLOW);
    read_velocity(GIMBAL_ROLL_ID, &rpm1);
    test_safe_move_to(GIMBAL_ROLL_ID, 0, 15.0f, GIMBAL_SPEED_SLOW);

    /* ---- 阶段 4：Yaw 轴单轴测试 ---- */
    test_safe_move_to(GIMBAL_YAW_ID, 0, 20.0f, GIMBAL_SPEED_SLOW);
    read_angle(GIMBAL_YAW_ID, &deg2);
    test_safe_move_to(GIMBAL_YAW_ID, 1, 20.0f, GIMBAL_SPEED_SLOW);
    test_safe_move_to(GIMBAL_YAW_ID, 1, 20.0f, GIMBAL_SPEED_SLOW);
    test_safe_move_to(GIMBAL_YAW_ID, 0, 20.0f, GIMBAL_SPEED_SLOW);

    /* ---- 阶段 5：双轴同步运动 ---- */
    ZDT_Motor_Pos_Control(GIMBAL_ROLL_ID, 0, GIMBAL_SPEED_SLOW,
                          GIMBAL_ACCEL,
                          GIMBAL_DEG_TO_PULSE(30.0f), 0, 1);
    test_flush_rx();
    ZDT_Motor_Pos_Control(GIMBAL_YAW_ID, 0, GIMBAL_SPEED_SLOW,
                          GIMBAL_ACCEL,
                          GIMBAL_DEG_TO_PULSE(30.0f), 0, 1);
    test_flush_rx();
    ZDT_Motor_Synchronous_motion(0);
    test_flush_rx();
    delay_ms(2500);
    read_angle(GIMBAL_ROLL_ID, &deg1);
    read_angle(GIMBAL_YAW_ID, &deg2);

    /* 同步回中 */
    ZDT_Motor_Pos_Control(GIMBAL_ROLL_ID, 1, GIMBAL_SPEED_SLOW,
                          GIMBAL_ACCEL,
                          GIMBAL_DEG_TO_PULSE(30.0f), 0, 1);
    test_flush_rx();
    ZDT_Motor_Pos_Control(GIMBAL_YAW_ID, 1, GIMBAL_SPEED_SLOW,
                          GIMBAL_ACCEL,
                          GIMBAL_DEG_TO_PULSE(30.0f), 0, 1);
    test_flush_rx();
    ZDT_Motor_Synchronous_motion(0);
    test_flush_rx();
    delay_ms(2500);
    read_angle(GIMBAL_ROLL_ID, &deg1);
    read_angle(GIMBAL_YAW_ID, &deg2);

    /* ---- 阶段 6：快速位置模式测试 ---- */
    ZDT_Motor_Set_QPos_Params(GIMBAL_ROLL_ID, GIMBAL_SPEED_SLOW,
                              GIMBAL_ACCEL, 0, 0);
    test_flush_rx();
    ZDT_Motor_QPos_Control(GIMBAL_ROLL_ID,
                           GIMBAL_DEG_TO_PULSE(10.0f));
    test_flush_rx();
    delay_ms(800);
    read_angle(GIMBAL_ROLL_ID, &deg1);

    ZDT_Motor_QPos_Control(GIMBAL_ROLL_ID,
                           GIMBAL_DEG_TO_PULSE(-10.0f));
    test_flush_rx();
    delay_ms(800);

    ZDT_Motor_QPos_Control(GIMBAL_ROLL_ID,
                           GIMBAL_DEG_TO_PULSE(-10.0f));
    test_flush_rx();
    delay_ms(800);

    ZDT_Motor_QPos_Control(GIMBAL_ROLL_ID,
                           GIMBAL_DEG_TO_PULSE(10.0f));
    test_flush_rx();
    delay_ms(800);

    ZDT_Motor_Set_QPos_Params(GIMBAL_YAW_ID, GIMBAL_SPEED_SLOW,
                              GIMBAL_ACCEL, 0, 0);
    test_flush_rx();
    ZDT_Motor_QPos_Control(GIMBAL_YAW_ID,
                           GIMBAL_DEG_TO_PULSE(15.0f));
    test_flush_rx();
    delay_ms(1000);
    ZDT_Motor_QPos_Control(GIMBAL_YAW_ID,
                           GIMBAL_DEG_TO_PULSE(-15.0f));
    test_flush_rx();
    delay_ms(1000);

    /* ---- 阶段 7：MMCL 多命令队列 ---- */
    ZDT_Motor_MMCL_Pos_Control(GIMBAL_ROLL_ID, 0, GIMBAL_SPEED_SLOW,
                               GIMBAL_ACCEL,
                               GIMBAL_DEG_TO_PULSE(10.0f), 0, 0);
    ZDT_Motor_MMCL_Pos_Control(GIMBAL_YAW_ID, 0, GIMBAL_SPEED_SLOW,
                               GIMBAL_ACCEL,
                               GIMBAL_DEG_TO_PULSE(10.0f), 0, 0);
    ZDT_Motor_Multi_Motor_Cmd(0);
    test_flush_rx();
    delay_ms(1500);
    read_angle(GIMBAL_ROLL_ID, &deg1);
    read_angle(GIMBAL_YAW_ID, &deg2);

    /* MMCL 回中 */
    ZDT_Motor_MMCL_Pos_Control(GIMBAL_ROLL_ID, 1, GIMBAL_SPEED_SLOW,
                               GIMBAL_ACCEL,
                               GIMBAL_DEG_TO_PULSE(10.0f), 0, 0);
    ZDT_Motor_MMCL_Pos_Control(GIMBAL_YAW_ID, 1, GIMBAL_SPEED_SLOW,
                               GIMBAL_ACCEL,
                               GIMBAL_DEG_TO_PULSE(10.0f), 0, 0);
    ZDT_Motor_Multi_Motor_Cmd(0);
    test_flush_rx();
    delay_ms(1500);

    /* ---- 阶段 8：急停测试 ---- */
    ZDT_Motor_Vel_Control(GIMBAL_ROLL_ID, 0, GIMBAL_SPEED_NORMAL,
                          GIMBAL_ACCEL, 0);
    test_flush_rx();
    delay_ms(500);
    ZDT_Motor_Stop_Now(GIMBAL_ROLL_ID, 0);
    test_flush_rx();

    ZDT_Motor_Vel_Control(GIMBAL_YAW_ID, 0, GIMBAL_SPEED_NORMAL,
                          GIMBAL_ACCEL, 0);
    test_flush_rx();
    delay_ms(500);
    ZDT_Motor_Stop_Now(GIMBAL_YAW_ID, 0);
    test_flush_rx();
    delay_ms(300);

    /* ---- 阶段 9：最终读取 + 关闭使能 ---- */
    read_angle(GIMBAL_ROLL_ID, &deg1);
    read_angle(GIMBAL_YAW_ID, &deg2);

    ZDT_Motor_Reset_CurPos_To_Zero(GIMBAL_ROLL_ID);
    test_flush_rx();
    ZDT_Motor_Reset_CurPos_To_Zero(GIMBAL_YAW_ID);
    test_flush_rx();

    ZDT_Motor_En_Control(GIMBAL_ROLL_ID, false, false);
    test_flush_rx();
    ZDT_Motor_En_Control(GIMBAL_YAW_ID, false, false);
    test_flush_rx();
}
