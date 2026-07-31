/**
 * @file board.c
 * @brief 板级初始化（小车端）
 *
 * 初始化顺序：SysTick → 定时器 → 执行器 → 输入 → 通信 → 显示
 */
#include "board.h"
#include "ti_msp_dl_config.h"
#include "pit_fast_tick.h"
#include "pit_control_tick.h"
#include "tb6612.h"
#include "servo.h"
#include "motor.h"
#include "grayscale.h"
#include "key.h"
#include "oled.h"
#include "balance.h"

static volatile uint32_t s_tick_ms;
static UART_Handle      *s_pi_uart;

static void tick_cb(void)
{
	s_tick_ms++;
}

void Board_Init(void)
{
	/* 1. 系统滴答 */
	PIT_Fast_Tick_Init();
	PIT_Fast_Tick_RegisterCallback(tick_cb);
	PIT_Control_Tick_Init();

	/* 2. 执行器 */
	TB6612_Init();
	Servo_Init();
	Motor_Init();
	Balance_Init();

	/* 3. 输入 */
	Grayscale_Init();
	Key_Init();

	/* 4. 显示 */
	OLED_Init();

	/* 5. 通信：Pi 有线 UART */
	{
		UART_Config pi_cfg = {
			.uart         = (UART_Regs *)UART_PI_INST,
			.irqNum       = UART_PI_INST_INT_IRQN,
			.dmaTxChanId  = 0,
			.dmaTxTrigger = 0,
		};

		s_pi_uart = UART_Init(&pi_cfg);
	}
}

uint32_t Board_GetTickMs(void)
{
	return s_tick_ms;
}

UART_Handle* Board_GetPiUART(void)
{
	return s_pi_uart;
}
