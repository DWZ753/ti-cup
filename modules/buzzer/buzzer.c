#include "buzzer.h"
#include "ti_msp_dl_config.h"
#include "pit_fast_tick.h"
#include <stdbool.h>

static volatile uint32_t s_beep_time = 0;   // 蜂鸣器鸣叫时间
static bool s_is_active = false;            // 蜂鸣器是否激活


void Buzzer_Beep(uint32_t time_ms)
{
    s_beep_time = time_ms;
    s_is_active = true;
}


void Buzzer_Stop(void)
{
    s_is_active = false;
    s_beep_time = 0;
    DL_GPIO_setPins(GPIO_BUZZERs_PORT, GPIO_BUZZERs_GPIO_BUZZER_PIN);
}


/**
 * @brief 蜂鸣器定时器tick处理回调函数
 * 
 * 该函数由PIT自定义tick定时器周期性调用，用于控制蜂鸣器的鸣叫时长。
 * 在蜂鸣器激活状态下，每次调用会递减剩余鸣叫时间并控制GPIO引脚状态。
 * 当鸣叫时间耗尽时，自动停止蜂鸣器。
 * 
 * @return 无
 * 
 * @note 此函数为静态私有函数，仅作为定时器回调使用
 * 
 */
static void Buzzer_TickHandler(void)
{
    if (!s_is_active) return;

    if (s_beep_time > 0)
    {
        DL_GPIO_clearPins(GPIO_BUZZERs_PORT, GPIO_BUZZERs_GPIO_BUZZER_PIN);
        s_beep_time--;
    }
    else
    {
        Buzzer_Stop();
    }
}


void Buzzer_Init(void)
{
    PIT_Fast_Tick_RegisterCallback(Buzzer_TickHandler);
}
