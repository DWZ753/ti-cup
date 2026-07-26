#include "pid.h"

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
              float integral_limit, float output_limit)
{
    pid->Kp             = kp;
    pid->Ki             = ki;
    pid->Kd             = kd;
    pid->target         = 0.0f;
    pid->error          = 0.0f;
    pid->last_error     = 0.0f;
    pid->prev_error     = 0.0f;
    pid->integral       = 0.0f;
    pid->integral_limit = integral_limit;
    pid->output_limit   = output_limit;
    pid->output         = 0.0f;
}

float PID_Compute(PID_Controller *pid, float current_value)
{
    pid->error = pid->target - current_value;

    float p_out = pid->Kp * pid->error;

    pid->integral += pid->error;
    if (pid->integral > pid->integral_limit)
        pid->integral = pid->integral_limit;
    else if (pid->integral < -pid->integral_limit)
        pid->integral = -pid->integral_limit;
    float i_out = pid->Ki * pid->integral;

    float d_out = pid->Kd * (pid->error - 2.0f * pid->last_error + pid->prev_error);

    pid->output = p_out + i_out + d_out;

    if (pid->output > pid->output_limit)
        pid->output = pid->output_limit;
    else if (pid->output < -pid->output_limit)
        pid->output = -pid->output_limit;

    pid->prev_error = pid->last_error;
    pid->last_error = pid->error;

    return pid->output;
}

void PID_SetTarget(PID_Controller *pid, float target)
{
    pid->target = target;
}

void PID_Reset(PID_Controller *pid)
{
    pid->error      = 0.0f;
    pid->last_error = 0.0f;
    pid->prev_error = 0.0f;
    pid->integral   = 0.0f;
    pid->output     = 0.0f;
}
