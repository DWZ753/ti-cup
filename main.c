/**
 * @file    main.c
 * @brief   26H 车载平衡滚球运动控制系统 — 小车端主程序
 *
 * 循迹控制逻辑封装在 tracking.c，main.c 仅负责调度：
 *   - KEY3 选择题目，KEY4 确认选择
 *   - KEY1 启动，KEY2 停止
 *   - OLED 显示菜单和状态
 *   - 循环调用 Tracking_Update()
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "tracking.h"
#include "grayscale.h"
#include "key.h"
#include "oled.h"

/* ========== 题目模式（要求2-6） ========== */

typedef enum {
	TASK_TRACK_ONLY     = 0,  // 要求2：纯循迹一圈，A点停车
	TASK_BAL_STATIC,          // 要求3：静态平衡（车不动，球 O→±5cm）
	TASK_TRACK_BAL_AB,        // 要求4：循迹+O点平衡，AB段
	TASK_TRACK_BAL_LAP_O,     // 要求5：循迹+O点平衡，一圈
	TASK_TRACK_BAL_LAP_X,     // 要求6：循迹+任意位置平衡，一圈
	TASK_COUNT
} TaskMode;

static const uint8_t *s_task_names[TASK_COUNT] = {
	(uint8_t*)"2.Track Only",
	(uint8_t*)"3.Bal Static",
	(uint8_t*)"4.T+B AB",
	(uint8_t*)"5.T+B Lap@O",
	(uint8_t*)"6.T+B Lap@X",
};

/* ========== 菜单状态 ========== */

typedef enum {
	STATE_MENU,          // 选择题目：KEY3 切换，KEY4 确认
	STATE_READY,         // 已确认，等待启动：KEY1 启动，KEY3 返回菜单
	STATE_RUNNING,       // 运行中：KEY2 停止
} AppState;

static AppState  s_state         = STATE_MENU;
static TaskMode  s_selected_task = TASK_TRACK_ONLY;
static bool      s_oled_dirty    = true;

/* ========== OLED 显示 ========== */

static void Menu_Show(void)
{
	uint8_t i;

	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"=== Task Select ===", 8);

	for (i = 0; i < TASK_COUNT; i++)
	{
		uint8_t row = 1 + i;

		if (i == s_selected_task)
			OLED_ShowString(0, row, (uint8_t*)">", 8);
		OLED_ShowString(12, row, s_task_names[i], 8);
	}

	OLED_ShowString(0, 7, (uint8_t*)"K3:Next K4:OK", 8);
}

static void Ready_Show(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"Task:", 8);
	OLED_ShowString(0, 2, s_task_names[s_selected_task], 8);
	OLED_ShowString(0, 5, (uint8_t*)"K1:Start K2:Stop", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K3:Back", 8);
}

static void Oled_Refresh(void)
{
	if (!s_oled_dirty)
		return;
	s_oled_dirty = false;

	switch (s_state)
	{
	case STATE_MENU:
		Menu_Show();
		break;
	case STATE_READY:
		Ready_Show();
		break;
	case STATE_RUNNING:
		break;   // 运行中不刷 OLED，节省主循环时间
	}
}

/* ========== 主函数 ========== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	Tracking_Init();

	/* 显示初始菜单 */
	s_oled_dirty = true;
	Oled_Refresh();

	while (1)
	{
		/* ======== 按键 ======== */

		switch (s_state)
		{
		case STATE_MENU:
			/* KEY3：切换题目 */
			if (Key_GetFlag(2))
			{
				s_selected_task = (s_selected_task + 1) % TASK_COUNT;
				s_oled_dirty    = true;
			}

			/* KEY4：确认选择 */
			if (Key_GetFlag(3))
			{
				s_state      = STATE_READY;
				s_oled_dirty = true;
			}
			break;

		case STATE_READY:
			/* KEY3：返回菜单重新选择 */
			if (Key_GetFlag(2))
			{
				s_state      = STATE_MENU;
				s_oled_dirty = true;
			}

			/* KEY1：启动 */
			if (Key_GetFlag(0))
			{
				/* 要求3（静态平衡）：车不动，暂不做任何事 */
				if (s_selected_task == TASK_BAL_STATIC)
				{
					/*
					 * TODO: 调用 Balance_Start() 执行 O→+5→-5cm
					 * 目前占位：进入 RUNNING 等待 KEY2 停止
					 */
				}
				else
				{
					/* 要求 2/4/5/6：循迹 */
					Tracking_Start();
				}

				s_state      = STATE_RUNNING;
				s_oled_dirty = true;
			}
			break;

		case STATE_RUNNING:
			/* KEY2：停止 */
			if (Key_GetFlag(1))
			{
				Tracking_Stop();
				s_state      = STATE_READY;
				s_oled_dirty = true;
			}

			/* 循迹内部停车检测自动停 → 回到 READY */
			if (!Tracking_IsRunning())
			{
				s_state      = STATE_READY;
				s_oled_dirty = true;
			}
			break;
		}

		/* ======== OLED 刷新 ======== */
		Oled_Refresh();

		/* ======== 任务更新 ======== */
		if (s_state != STATE_RUNNING)
			continue;

		switch (s_selected_task)
		{
		case TASK_TRACK_ONLY:
		case TASK_TRACK_BAL_AB:
		case TASK_TRACK_BAL_LAP_O:
		case TASK_TRACK_BAL_LAP_X:
			/* 循迹更新 */
			{
				uint8_t mask = Grayscale_ReadAll();
				Tracking_Update(Board_GetTickMs(), mask);
			}
			break;

		case TASK_BAL_STATIC:
			/*
			 * TODO: 调用 Balance_Update() 执行静态平衡流程
			 * 当前占位：无操作，等待 KEY2 停止
			 */
			break;
		}
	}
}
