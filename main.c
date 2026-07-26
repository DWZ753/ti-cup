/**
 * @file    main.c
 * @brief   TI Cup 2024 赛题主入口 — 差速循迹 + 状态机
 *
 * 控制逻辑全部封装在 Chassis_Init() / Chassis_Task() 中。
 * 可调参数见 application/chassis/chassis_config.h。
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "chassis.h"

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();
	Chassis_Init();

	while (1)
	{
		Chassis_Task();
	}
}
