/**
 * @file main.c
 * @brief 智能物流搬运系统 — 无线计分显示装置
 *
 * 搭载：数码管、按键、LED 指示灯、蜂鸣器、无线通信模块
 */
#include "ti_msp_dl_config.h"
#include "board.h"

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	while (1)
	{
	}
}
