#include "angle.h"
#include "imu.h"
#include "servo.h"

/* ========== 内部静态状态 ========== */

static PID_Controller s_angle_pid;    // 复用 application/pid 的 PID 结构体
static bool           s_enabled;      // 当前是否启用角度保持
static int32_t        s_last_servo;   // 上次舵机输出值（用于 slew rate 限幅）

/* ========== 公共 API ========== */

void Angle_Init(void)
{
    PID_Init(&s_angle_pid,
             ANGLE_KP, ANGLE_KI, ANGLE_KD,
             ANGLE_INTEGRAL_LIMIT, ANGLE_OUTPUT_LIMIT);
    s_enabled    = false;
    s_last_servo = 0;
}

void Angle_Enable(bool enable)
{
    if (enable && !s_enabled)
    {
        // 从关闭切换到启用：只清 PID 积分/误差，舵机保持当前值平滑过渡
        PID_Reset(&s_angle_pid);
    }
    else if (!enable && s_enabled)
    {
        // 从启用切换到关闭：舵机归零
        Servo_SetValue(0);
        s_last_servo = 0;
        PID_Reset(&s_angle_pid);
    }
    s_enabled = enable;
}

bool Angle_IsEnabled(void)
{
    return s_enabled;
}

void Angle_SetTarget(float heading_deg)
{
    s_angle_pid.target = heading_deg;
}

static float wrap_180(float error);

/**
 * @brief 以当前 yaw 为基准，偏转 delta° 后作为目标航向
 *
 * 用于相对角度模式：每次进入直线段时调用一次，锁定"当前朝向 + 偏转量"。
 * yaw 的绝对漂移不影响相对偏转精度。
 */
void Angle_SetTargetRelative(float delta_deg)
{
    float roll, pitch, yaw;
    IMU_GetEuler(&roll, &pitch, &yaw);
    s_angle_pid.target = wrap_180(yaw + delta_deg);
}

/**
 * @brief 将角度误差归一化到 [-180°, 180°]
 */
static float wrap_180(float error)
{
    while (error > 180.0f)  error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    return error;
}

void Angle_Compute(void)
{
    if (!s_enabled)
        return;

    /* ---- 1. 读取当前偏航角 ---- */
    float roll, pitch, yaw;
    IMU_GetEuler(&roll, &pitch, &yaw);

    /* ---- 2. 角度缠绕归一化 ---- */
    float error = wrap_180(yaw - s_angle_pid.target);

    /* ---- 3. 死区检查 ---- */
    int32_t servo_out;

    if (error < ANGLE_DEADBAND_DEG && error > -ANGLE_DEADBAND_DEG)
    {
        // 死区内：舵机归零，清零积分，手动推进误差历史（为将来 D 项保持连续性）
        servo_out = 0;
        s_angle_pid.integral   = 0.0f;
        s_angle_pid.prev_error = s_angle_pid.last_error;
        s_angle_pid.last_error = error;
    }
    else
    {
        /* ---- 4. PID 计算 ---- */
        // current_value = target - error，经缠绕修正后的等价当前值
        float current_value = s_angle_pid.target - error;
        float output = PID_Compute(&s_angle_pid, current_value);
        servo_out = (int32_t)output;
    }

    /* ---- 5. Slew rate 限幅 ---- */
    int32_t delta = servo_out - s_last_servo;
    if (delta > ANGLE_SLEW_MAX)
        servo_out = s_last_servo + ANGLE_SLEW_MAX;
    else if (delta < -ANGLE_SLEW_MAX)
        servo_out = s_last_servo - ANGLE_SLEW_MAX;

    s_last_servo = servo_out;

    /* ---- 6. 输出到舵机 ---- */
    Servo_SetValue(servo_out);
}

void Angle_Reset(void)
{
    PID_Reset(&s_angle_pid);
    s_last_servo = 0;
    Servo_SetValue(0);
}

/* ========== 遥测 Getter ========== */

float Angle_GetTarget(void)
{
    return s_angle_pid.target;
}

float Angle_GetPIDOutput(void)
{
    return s_angle_pid.output;
}

int32_t Angle_GetServoValue(void)
{
    return s_last_servo;
}
