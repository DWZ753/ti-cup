/**
 * @file board.c
 * @brief 板级初始化模板（公共驱动基线）
 *
 * 赛题分支基于此文件扩展，添加各自外设的初始化。
 * 编译前需确保 empty.syscfg 已配置所需外设。
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
