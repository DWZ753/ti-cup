#include <stdint.h>
#include "pit_fast_tick.h"
#include "ti_msp_dl_config.h"

#define MAX_PIT_FAST_TASKS 8 // 最多同时支持8个任务

static PIT_Fast_Callback_t  s_tick_tasks[MAX_PIT_FAST_TASKS]; // 任务列表数组
static uint8_t s_task_count = 0;    // 任务数量


void PIT_Fast_Tick_Init(void)
{
    NVIC_EnableIRQ(PIT_FAST_TICK_INST_INT_IRQN);
}


bool PIT_Fast_Tick_RegisterCallback(PIT_Fast_Callback_t callback)
{
    if (s_task_count >= MAX_PIT_FAST_TASKS || callback == NULL)
    {
        return false;
    }
    s_tick_tasks[s_task_count++] = callback;
    return true;
}


/**
 * @brief PIT快速滴答实例中断服务程序
 *
 * 当PIT定时器产生LOAD中断时，依次执行所有已注册的回调任务。
 * 该函数由硬件中断自动调用，用于处理周期性定时任务。
 *
 * @return 无
 *
 * @note 此函数在中断上下文中执行，回调函数应尽量简短以避免影响系统实时性
 */
void PIT_FAST_TICK_INST_IRQHandler(void)
{
    switch(DL_TimerG_getPendingInterrupt(PIT_FAST_TICK_INST))
    {
        case DL_TIMER_IIDX_LOAD:
        // 遍历所有注册的任务并执行
        for (uint8_t i = 0; i < s_task_count; i++)
        {
            if (s_tick_tasks[i] != NULL)
            {
                s_tick_tasks[i]();
            }
        }
        break;
    }
}
