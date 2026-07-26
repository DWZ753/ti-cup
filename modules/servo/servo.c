#include "servo.h"
#include "ti_msp_dl_config.h"

void Servo_Init(void)
{
    DL_Timer_startCounter(SERVO_PWM_INST);
    Servo_SetValue(0);
}

int32_t Servo_LimitValue(int32_t value)
{
    if (value > 100)
    {
        value = 100;
    }
    if (value < -100)
    {
        value = -100;
    }

    return value;
}

void Servo_SetValue(int32_t value)
{
    uint32_t duty_us;
    uint32_t duty_counts;

    value = Servo_LimitValue(value);

    // 脉宽微秒值（线性映射）
    duty_us = (uint32_t)((int32_t)SERVO_PULSE_CENTER_US +
        (value * (int32_t)(SERVO_PULSE_MAX_US - SERVO_PULSE_MIN_US) / 200));

    // 微秒 → 定时器比较值: counts = us * (clk_freq / 1,000,000)
    duty_counts = duty_us * (SERVO_PWM_CLK_FREQ / 1000000U);

    DL_Timer_setCaptureCompareValue(SERVO_PWM_INST, duty_counts, SERVO_PWM_CHANNEL);
}
