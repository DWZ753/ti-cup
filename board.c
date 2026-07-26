/**
 * @file board.c
 * @brief 板级初始化（小车端）
 *
 * 初始化顺序：SysTick → 定时器 → 执行器 → 输入 → 通信 → 传感器
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "pit_fast_tick.h"
#include "pit_control_tick.h"

static volatile uint32_t s_tick_ms;

static void tick_cb(void)
{
	s_tick_ms++;
}

void Board_Init(void)
{
	SYSCFG_DL_init();
	PIT_Fast_Tick_Init();
	PIT_Fast_Tick_RegisterCallback(tick_cb);
	PIT_Control_Tick_Init();
}

uint32_t Board_GetTickMs(void)
{
	return s_tick_ms;
}
