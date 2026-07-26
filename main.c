/**
 * @file main.c
 * @brief 主入口模板（公共驱动基线）
 *
 * 赛题分支基于此文件扩展。完整赛题逻辑见各分支的 main.c。
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
