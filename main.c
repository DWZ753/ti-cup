/**
 * @file main.c
 * @brief 智能物流搬运系统 — 移动搬运小车
 *
 * 搭载：摄像头、电磁铁、循迹传感器、无线通信模块、电机驱动
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
