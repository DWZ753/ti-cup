#include "tb6612.h"
#include "ti_msp_dl_config.h"


void TB6612_Init(void)
{
   DL_Timer_startCounter(TB6612_PWM_TIMER);
}


uint32_t TB6612_LimitPWM(uint32_t period_count)
{
    if (period_count > TB6612_PWM_PERIOD_COUNT)
    {
        period_count = TB6612_PWM_PERIOD_COUNT;
    }

    return period_count;
}


void TB6612_A_Forward(uint32_t duty)
{
    duty = TB6612_LimitPWM(duty);

    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN1_PIN);
    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(TB6612_PWM_TIMER, duty, TB6612_PWM_A_PIN);
}


void TB6612_A_Backward(uint32_t duty)
{ 
    duty = TB6612_LimitPWM(duty);

    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN1_PIN);
    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN2_PIN);
    DL_Timer_setCaptureCompareValue(TB6612_PWM_TIMER, duty, TB6612_PWM_A_PIN);
}


void TB6612_A_Brake(void)
{
    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN1_PIN | TB6612_GPIO_AIN2_PIN);
}


void TB6612_A_Stop(void)
{
    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_AIN1_PIN | TB6612_GPIO_AIN2_PIN);
}


void TB6612_B_Forward(uint32_t duty)
{ 
    duty = TB6612_LimitPWM(duty);

    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN1_PIN);
    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(TB6612_PWM_TIMER, duty, TB6612_PWM_B_PIN);
}


void TB6612_B_Backward(uint32_t duty)
{ 
    duty = TB6612_LimitPWM(duty);

    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN1_PIN);
    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN2_PIN);
    DL_Timer_setCaptureCompareValue(TB6612_PWM_TIMER, duty, TB6612_PWM_B_PIN);
}


void TB6612_B_Brake(void)
{
    DL_GPIO_setPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN1_PIN | TB6612_GPIO_BIN2_PIN);
}


void TB6612_B_Stop(void)
{
    DL_GPIO_clearPins(TB6612_GPIO_PORT, TB6612_GPIO_BIN1_PIN | TB6612_GPIO_BIN2_PIN);
}
