/**
 * @file    main.c
 * @brief   26H 车载平衡滚球运动控制系统 — 小车端主程序
 *
 * 循迹控制逻辑封装在 tracking.c，main.c 仅负责调度：
 *   - KEY1 启动循迹
 *   - KEY2 停止循迹
 *   - 循环调用 Tracking_Update()
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "tracking.h"
#include "grayscale.h"
#include "key.h"

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	/* ---- 循迹模块 ---- */
	Tracking_Init();

	while (1)
	{
		/* ======== 按键 ======== */

		/* KEY2：停止循迹 */
		if (Key_GetFlag(1) && Tracking_IsRunning())
			Tracking_Stop();

		/* KEY1：启动循迹 */
		if (Key_GetFlag(0) && !Tracking_IsRunning())
			Tracking_Start();

		if (!Tracking_IsRunning())
			continue;

		/* ======== 循迹更新 ======== */
		uint8_t mask = Grayscale_ReadAll();
		Tracking_Update(Board_GetTickMs(), mask);
	}
}
