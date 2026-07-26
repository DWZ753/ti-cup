#include "board.h"
#include "state_machine.h"
#include "pid.h"
#include "angle.h"
#include "tracking.h"

/* ========== 控制周期（ms） ========== */
#define IMU_UPDATE_DT_MS      10
#define ANGLE_PID_DT_MS       80

/* ========== 丢线防抖 ========== */
#define LINE_LOST_DEBOUNCE     5

/* ========== 固定速度（开环） ========== */
#define SPEED_ARC          1000.0f
#define SPEED_STRAIGHT     1000.0f
#define ARC_DIFF_GAIN        7.3f   // 差速增益：servo × gain = 两轮速度差 (mm/s)

/* ========== 循迹 PID 参数 ========== */
// position ∈ [-1, 1]，OUT_LIMIT=40 对应满偏 40%
// KP=40 → 最边缘传感器单独压线时输出 ±40
#define TRACKING_KP         100.0f
#define TRACKING_KI          0.5f
#define TRACKING_KD          0.0f   // 用 slew rate 替代 D 项
#define TRACKING_INT_LIMIT  20.0f
#define TRACKING_OUT_LIMIT  100.0f
#define TRACKING_SLEW_MAX     4     // 每次调用最大变化量

int main(void)
{
    SYSCFG_DL_init();
    Board_Init();
    StateMachine_Init();

    /* ---- 循迹 PID（弧线段使用） ---- */
    PID_Controller tracking_pid;
    PID_Init(&tracking_pid,
             TRACKING_KP, TRACKING_KI, TRACKING_KD,
             TRACKING_INT_LIMIT, TRACKING_OUT_LIMIT);
    PID_SetTarget(&tracking_pid, 0.0f);
    int32_t tracking_last_servo = 0;   // slew rate 历史

    /* ---- 角度 PID（直线段使用） ---- */
    Angle_Init();

    /* ---- 时序变量 ---- */
    uint32_t last_imu        = 0;
    uint32_t last_angle_pid  = 0;
    uint32_t last_tracking   = 0;
    uint32_t last_output     = 0;

    /* ---- 状态机变化跟踪 ---- */
    QuestionState_t last_state = STATE_IDLE;
    SegmentType_t   last_seg  = SEG_STOP;   // 用于检测段切换

    /* ---- 丢线防抖计数器 ---- */
    uint8_t lost_debounce = 0;

    uint8_t buf[16];

    while (1)
    {
        uint32_t now = Board_GetTickMs();

        /* ======== IMU 姿态更新 ======== */
        if (now - last_imu >= IMU_UPDATE_DT_MS)
        {
            last_imu = now;
            IMU_Update();
        }

        /* ======== 按键检测 & 任务启动 ======== */
        QuestionState_t st = StateMachine_GetState();
        if (st != last_state)
        {
            last_state = st;
            if (st != STATE_IDLE)
            {
                StateMachine_StartTask(st);
                lost_debounce = 0;
            }
            else
            {
                Motor_Brake();
            }
        }

        /* ======== 当前段类型 → 控制模式 ======== */
        SegmentType_t seg = StateMachine_GetCurrentSegment();
        uint8_t mask = Grayscale_ReadAll();
        bool on_line = (mask != 0xFF);

        if (seg == SEG_ARC)
        {
            /* 循迹模式 */
            Angle_Enable(false);

            /* 差速控制：舵机偏向越大，两轮速度差越大 */
            {
                float diff = tracking_last_servo * ARC_DIFF_GAIN;
                float left_speed  = SPEED_ARC + diff;
                float right_speed = SPEED_ARC - diff;
                Motor_SetSpeedLR(left_speed, right_speed);
            }

            /* 刚切入弧线段时复位 PID */
            if (last_seg != SEG_ARC)
            {
                PID_Reset(&tracking_pid);
                PID_SetTarget(&tracking_pid, 0.0f);
                tracking_last_servo = 0;
            }

            /* 定周期计算循迹 PID（10ms） */
            if (now - last_tracking >= 10)
            {
                last_tracking = now;
                float position = Tracking_CalcPosition(mask);
                if (position != 99.0f)
                {
                    float steering = PID_Compute(&tracking_pid, -position);
                    int32_t servo_out = (int32_t)steering;

                    /* slew rate 限幅 */
                    int32_t delta = servo_out - tracking_last_servo;
                    if (delta > TRACKING_SLEW_MAX)
                        servo_out = tracking_last_servo + TRACKING_SLEW_MAX;
                    else if (delta < -TRACKING_SLEW_MAX)
                        servo_out = tracking_last_servo - TRACKING_SLEW_MAX;

                    tracking_last_servo = servo_out;
                    Servo_SetValue(servo_out);
                }
                else
                {
                    /* 全白丢线：舵机缓慢回中，防止卡在上次纠偏角度 */
                    if (tracking_last_servo > TRACKING_SLEW_MAX)
                        tracking_last_servo -= TRACKING_SLEW_MAX;
                    else if (tracking_last_servo < -TRACKING_SLEW_MAX)
                        tracking_last_servo += TRACKING_SLEW_MAX;
                    else
                        tracking_last_servo = 0;
                    Servo_SetValue(tracking_last_servo);
                }
            }
        }
        else if (seg == SEG_STRAIGHT)
        {
            /* 角度保持模式：进入时以当前 yaw 为基准 + 相对偏转角 */
            if (!Angle_IsEnabled())
            {
                Angle_Enable(true);
                Angle_SetTargetRelative(StateMachine_GetDeltaDeg());
            }
            /* 差速辅助转向：舵偏越大两轮速差越大，加速回正 */
            {
                int32_t servo = Angle_GetServoValue();
                float diff = servo * ARC_DIFF_GAIN;
                float left_speed  = SPEED_STRAIGHT + diff;
                float right_speed = SPEED_STRAIGHT - diff;
                Motor_SetSpeedLR(left_speed, right_speed);
            }
        }
        else  // SEG_STOP
        {
            Angle_Enable(false);
            Motor_Brake();
        }

        /* ======== 角度 PID 定时计算 ======== */
        if (now - last_angle_pid >= ANGLE_PID_DT_MS)
        {
            last_angle_pid = now;
            Angle_Compute();
        }

        /* ======== 段切换检测 ======== */
        if (seg == SEG_ARC)
        {
            if (!on_line)
            {
                if (++lost_debounce >= LINE_LOST_DEBOUNCE)
                {
                    lost_debounce = 0;
                    StateMachine_SegmentDone();
                }
            }
            else
            {
                lost_debounce = 0;
            }
        }
        else if (seg == SEG_STRAIGHT)
        {
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
                lost_debounce = 0;
            }
        }

        /* ======== 终点停车 ======== */
        if (StateMachine_IsFinished())
        {
            Motor_Brake();
        }

        last_seg = seg;

        /* ======== UART 遥测（每 100ms） ======== */
        if (now - last_output >= 100)
        {
            last_output = now;

            float roll, pitch, yaw;
            IMU_GetEuler(&roll, &pitch, &yaw);

            sprintf(buf, "yaw %.1f", yaw);
            OLED_ShowString(32, 0, buf, 16);
            sprintf(buf, "yaw_t %.1f", Angle_GetTarget());
            OLED_ShowString(32, 2, buf, 16);

            float pos = Tracking_CalcPosition(mask);
            float trk_out = tracking_pid.output;

            UART_Printf(Board_GetUART(),
                "%d, %.1f, %.1f, %.1f, %ld, %d, %d, %d, %.2f, %.1f, %ld\n",
                seg,
                yaw,
                Angle_GetTarget(),
                Angle_GetPIDOutput(),
                (long)Angle_GetServoValue(),
                on_line,
                lost_debounce,
                mask,
                pos,
                trk_out,
                (long)tracking_last_servo);
        }
    }
}
