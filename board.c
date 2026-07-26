#include "board.h"

/* ========== 系统滴答时钟 ========== */

static volatile uint32_t g_sys_tick_ms;

uint32_t Board_GetTickMs(void)
{
	return g_sys_tick_ms;
}

/* ========== 板级初始化 ========== */

void Board_Init(void)
{
	/* ---- 系统滴答（SysTick 1ms） ---- */
	g_sys_tick_ms = 0;
	SysTick_Config(CPUCLK_FREQ / 1000UL);

	/* ---- 显示（OLED，自注册 I2C） ---- */
	OLED_Init();

	/* ---- 传感器（IMU，自注册 SPI） ---- */
	IMU_Init();
}

/* ========== SysTick 中断 ========== */

void SysTick_Handler(void)
{
	++g_sys_tick_ms;
}
