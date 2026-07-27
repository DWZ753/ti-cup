/**
 * @file board.c
 * @brief 板级初始化（计分显示装置端）
 *
 * 初始化顺序：SysTick → 定时器 → 外设 → 通信
 */
#include "board.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_tick_ms;

static void tick_cb(void)
{
	s_tick_ms++;
}

void Board_Init(void)
{
	/* ---- 系统滴答 ---- */
	PIT_Fast_Tick_Init();
	PIT_Fast_Tick_RegisterCallback(tick_cb);
	PIT_Control_Tick_Init();

	/* ---- 外设初始化 ---- */
	Key_Init();
	TM1637_Init();
	Buzzer_Init();
	OLED_Init();
	Servo_Init();

}

uint32_t Board_GetTickMs(void)
{
	return s_tick_ms;
}
