/**
 * @file board.c
 * @brief 板级初始化（计分显示装置端）
 */
#include "board.h"
#include "ti_msp_dl_config.h"

static volatile uint32_t s_tick_ms;
static UART_Handle      *s_uart_bt;

static void tick_cb(void)
{
	s_tick_ms++;
}

void Board_Init(void)
{
	/* 系统滴答 */
	PIT_Fast_Tick_Init();
	PIT_Fast_Tick_RegisterCallback(tick_cb);
	PIT_Control_Tick_Init();

	/* 外设 */
	Key_Init();
	TM1637_Init();
	Buzzer_Init();
	OLED_Init();
	Servo_Init();

	/* 蓝牙 UART（UART1: PB6/TX, PB7/RX, 115200） */
	UART_Config uart_cfg = {
		.uart         = BT24_INST,
		.irqNum       = BT24_INST_INT_IRQN,
		.dmaTxChanId  = 0,
		.dmaTxTrigger = 0,
	};
	s_uart_bt = UART_Init(&uart_cfg);
}

uint32_t Board_GetTickMs(void)
{
	return s_tick_ms;
}

UART_Handle* Board_GetUART_BT(void)
{
	return s_uart_bt;
}
