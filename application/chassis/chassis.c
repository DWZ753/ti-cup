/**
 * @file    chassis.c
 * @brief   底盘控制子系统实现 — 从 main.c 提取的控制逻辑
 *
 * 内部所有状态均为 static，外部仅通过 Chassis_Init() / Chassis_Task() 操作。
 * 可调参数集中在 chassis_config.h，改参数无需进此文件。
 */

#include <stdio.h>
#include <stdbool.h>
#include "board.h"
#include "chassis.h"
#include "chassis_config.h"
#include "pid.h"
#include "angle.h"
#include "tracking.h"
#include "state_machine.h"
#include "grayscale.h"
#include "motor.h"
#include "imu.h"
#include "servo.h"
#include "oled.h"
#include "uart.h"

/* ======================================================================== */
/*  内部状态（全部 static）                                                 */
/* ======================================================================== */

/* -- PID -- */
static PID_Controller s_tracking_pid;

/* -- 时序 -- */
static uint32_t s_last_imu;
static uint32_t s_last_angle_pid;
static uint32_t s_last_tracking;
static uint32_t s_last_output;

/* -- 状态跟踪 -- */
static QuestionState_t s_last_state = STATE_IDLE;
static SegmentType_t   s_last_seg   = SEG_STOP;
static uint8_t         s_lost_debounce;
static int32_t         s_tracking_prev_servo;   // slew rate 历史

/* -- 遥测缓冲 -- */
static uint8_t s_telem_buf[256];

/* ======================================================================== */
/*  内部辅助                                                               */
/* ======================================================================== */

/**
 * @brief 弧线段控制：循迹 PID → 差速 + 舵机
 */
static void control_arc(uint32_t now)
{
    uint8_t mask    = Grayscale_ReadAll();
    bool    on_line = (mask != 0xFF);

    /* 差速控制：舵机偏向越大，两轮速度差越大 */
    float diff        = (float)s_tracking_prev_servo * ARC_DIFF_GAIN;
    float left_speed  = SPEED_ARC + diff;
    float right_speed = SPEED_ARC - diff;
    Motor_SetSpeedLR(left_speed, right_speed);

    /* 刚切入弧线段时复位 PID */
    if (s_last_seg != SEG_ARC)
    {
        PID_Reset(&s_tracking_pid);
        PID_SetTarget(&s_tracking_pid, 0.0f);
        s_tracking_prev_servo = 0;
    }

    /* 定周期计算循迹 PID（10ms） */
    if (now - s_last_tracking >= 10)
    {
        s_last_tracking = now;

        float position = Tracking_CalcPosition(mask);

        if (position != 99.0f)
        {
            /* 正常循迹 */
            float   steering  = PID_Compute(&s_tracking_pid, -position);
            int32_t servo_out = (int32_t)steering;

            /* slew rate 限幅 */
            int32_t delta = servo_out - s_tracking_prev_servo;
            if (delta > TRACKING_SLEW_MAX)
            {
                servo_out = s_tracking_prev_servo + TRACKING_SLEW_MAX;
            }
            else if (delta < -TRACKING_SLEW_MAX)
            {
                servo_out = s_tracking_prev_servo - TRACKING_SLEW_MAX;
            }

            s_tracking_prev_servo = servo_out;
            Servo_SetValue(servo_out);
        }
        else
        {
            /* 全白丢线：舵机缓慢回中，防止卡在上次纠偏角度 */
            if (s_tracking_prev_servo > TRACKING_SLEW_MAX)
            {
                s_tracking_prev_servo -= TRACKING_SLEW_MAX;
            }
            else if (s_tracking_prev_servo < -TRACKING_SLEW_MAX)
            {
                s_tracking_prev_servo += TRACKING_SLEW_MAX;
            }
            else
            {
                s_tracking_prev_servo = 0;
            }
            Servo_SetValue(s_tracking_prev_servo);
        }
    }

    /* -- 弧线段换段检测：丢线防抖 -- */
    if (!on_line)
    {
        if (++s_lost_debounce >= LINE_LOST_DEBOUNCE)
        {
            s_lost_debounce = 0;
            StateMachine_SegmentDone();
        }
    }
    else
    {
        s_lost_debounce = 0;
    }
}

/**
 * @brief 直线段控制：角度保持 + 阿克曼固定速度 + 差速辅助
 */
static void control_straight(uint32_t now)
{
    uint8_t mask    = Grayscale_ReadAll();
    bool    on_line = (mask != 0xFF);

    /* 进入直线段时以当前 yaw 为基准 + 相对偏转角 */
    if (!Angle_IsEnabled())
    {
        Angle_Enable(true);
        Angle_SetTargetRelative(StateMachine_GetDeltaDeg());
    }

    /* 差速辅助转向：舵偏越大两轮速差越大，加速回正 */
    int32_t servo      = Angle_GetServoValue();
    float   diff       = (float)servo * ARC_DIFF_GAIN;
    float   left_speed  = SPEED_STRAIGHT + diff;
    float   right_speed = SPEED_STRAIGHT - diff;
    Motor_SetSpeedLR(left_speed, right_speed);

    /* -- 直线段换段检测：离-回线 -- */
    if (on_line)
    {
        if (!StateMachine_NeedLeaveFirst())
        {
            StateMachine_SegmentDone();
        }
    }
    else
    {
        if (StateMachine_NeedLeaveFirst())
        {
            StateMachine_LeftLine();
        }
        s_lost_debounce = 0;
    }
}

/**
 * @brief 遥测输出：UART + OLED
 */
static void telemetry(uint32_t now)
{
    if (now - s_last_output < TELEMETRY_DT_MS) return;
    s_last_output = now;

    float roll, pitch, yaw;
    IMU_GetEuler(&roll, &pitch, &yaw);

    /* OLED */
    snprintf((char *)s_telem_buf, sizeof(s_telem_buf), "yaw %.1f", yaw);
    OLED_ShowString(32, 0, s_telem_buf, 16);

    snprintf((char *)s_telem_buf, sizeof(s_telem_buf), "yaw_t %.1f", Angle_GetTarget());
    OLED_ShowString(32, 2, s_telem_buf, 16);

    snprintf((char *)s_telem_buf, sizeof(s_telem_buf), "state %d",
             StateMachine_GetState());
    OLED_ShowString(32, 4, s_telem_buf, 16);

    /* UART */
    uint8_t mask     = Grayscale_ReadAll();
    bool    on_line  = (mask != 0xFF);
    float   pos      = Tracking_CalcPosition(mask);
    float   trk_out  = s_tracking_pid.output;
    SegmentType_t seg = StateMachine_GetCurrentSegment();

    UART_Printf(Board_GetUART(),
        "%d, %.1f, %.1f, %.1f, %ld, %d, %d, %d, %.2f, %.1f, %ld\n",
        seg,
        yaw,
        Angle_GetTarget(),
        Angle_GetPIDOutput(),
        (long)Angle_GetServoValue(),
        on_line,
        s_lost_debounce,
        mask,
        pos,
        trk_out,
        (long)s_tracking_prev_servo);
}

/* ======================================================================== */
/*  公共 API                                                                */
/* ======================================================================== */

void Chassis_Init(void)
{
    /* ---- 循迹 PID ---- */
    PID_Init(&s_tracking_pid,
             TRACKING_KP, TRACKING_KI, TRACKING_KD,
             TRACKING_INT_LIMIT, TRACKING_OUT_LIMIT);
    PID_SetTarget(&s_tracking_pid, 0.0f);

    /* ---- 角度保持 ---- */
    Angle_Init();

    /* ---- 状态机 ---- */
    StateMachine_Init();

    /* ---- 状态复位 ---- */
    s_last_state        = STATE_IDLE;
    s_last_seg          = SEG_STOP;
    s_lost_debounce     = 0;
    s_tracking_prev_servo  = 0;
    s_last_imu          = 0;
    s_last_angle_pid    = 0;
    s_last_tracking     = 0;
    s_last_output       = 0;
}

void Chassis_Task(void)
{
    uint32_t now = Board_GetTickMs();

    /* ======== IMU 姿态更新 ======== */
    if (now - s_last_imu >= IMU_UPDATE_DT_MS)
    {
        s_last_imu = now;
        IMU_Update();
    }

    /* ======== 按键检测 & 任务启动 ======== */
    QuestionState_t st = StateMachine_GetState();
    if (st != s_last_state)
    {
        s_last_state = st;
        if (st != STATE_IDLE)
        {
            StateMachine_StartTask(st);
            s_lost_debounce = 0;
        }
        else
        {
            Motor_Brake();
        }
    }

    /* ======== 当前段类型 → 控制模式 ======== */
    SegmentType_t seg = StateMachine_GetCurrentSegment();

    switch (seg)
    {
        case SEG_ARC:
            Angle_Enable(false);
            control_arc(now);
            break;

        case SEG_STRAIGHT:
            control_straight(now);
            break;

        default:   /* SEG_STOP */
            Angle_Enable(false);
            Motor_Brake();
            break;
    }

    /* ======== 角度 PID 定时计算 ======== */
    if (now - s_last_angle_pid >= ANGLE_PID_DT_MS)
    {
        s_last_angle_pid = now;
        Angle_Compute();
    }

    /* ======== 终点停车 ======== */
    if (StateMachine_IsFinished())
    {
        Motor_Brake();
    }

    s_last_seg = seg;

    /* ======== 遥测 ======== */
    telemetry(now);
}
