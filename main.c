/**
 * @file    main.c
 * @brief   26H 车载平衡滚球运动控制系统 — 小车端主程序
 *
 * 循迹控制逻辑封装在 tracking.c，main.c 仅负责调度：
 *   - KEY3 选择题目，KEY4 确认选择
 *   - KEY1 启动 + 开始计时，KEY2 停止 + 停止计时
 *   - OLED 显示菜单、计时、状态
 *   - 循环调用 Tracking_Update()
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "tracking.h"
#include "grayscale.h"
#include "key.h"
#include "oled.h"
#include "balance.h"

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
	(uint8_t*)"2.Run 1 Lap",
	(uint8_t*)"3.Ball Balance",
	(uint8_t*)"4.Run+Bal AB",
	(uint8_t*)"5.Run+Bal 1Lap",
	(uint8_t*)"6.Run+Bal Any",
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

/* ========== 计时 ========== */

static uint32_t s_start_ms;         // 本次启动时刻 (tick)
static uint32_t s_last_time_ms;     // 上次运行用时 (ms)
static uint32_t s_last_oled_ms;     // 上次 OLED 刷新时刻
static uint8_t  s_time_buf[8];      // 格式化时间串 "MM:SS.T"

/**
 * @brief 将毫秒格式化为 "MM:SS.T" 写入 s_time_buf
 */
static void Format_Time(uint32_t ms)
{
	uint16_t total_tenth = (uint16_t)(ms / 100);
	uint8_t  min         = (uint8_t)(total_tenth / 600);
	uint8_t  sec         = (uint8_t)((total_tenth / 10) % 60);
	uint8_t  tenth       = (uint8_t)(total_tenth % 10);

	s_time_buf[0] = '0' + (min / 10);
	s_time_buf[1] = '0' + (min % 10);
	s_time_buf[2] = ':';
	s_time_buf[3] = '0' + (sec / 10);
	s_time_buf[4] = '0' + (sec % 10);
	s_time_buf[5] = '.';
	s_time_buf[6] = '0' + tenth;
	s_time_buf[7] = '\0';
}

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

	if (s_last_time_ms > 0)
	{
		Format_Time(s_last_time_ms);
		OLED_ShowString(0, 3, (uint8_t*)"Time:", 8);
		OLED_ShowString(30, 3, s_time_buf, 8);
	}

	OLED_ShowString(0, 5, (uint8_t*)"K1:Start K2:Stop", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K3:Back", 8);
}

/**
 * @brief 运行中首次绘制：清屏 + 静态元素 + 初始时间
 */
static void Running_Show_First(void)
{
	OLED_Clear();
	OLED_ShowString(0, 0, s_task_names[s_selected_task], 8);
	OLED_ShowString(0, 7, (uint8_t*)"K2:Stop", 8);

	Format_Time(0);
	/* 16px 大字居中: 7 字符 x 8px = 56px, (128-56)/2 = 36 */
	OLED_ShowString(36, 2, s_time_buf, 16);
}

/**
 * @brief 运行中更新时间：只重写数字，不清屏
 * @param elapsed_ms 当前已用时间 (ms)
 */
static void Running_Show_Time(uint32_t elapsed_ms)
{
	Format_Time(elapsed_ms);
	OLED_ShowString(36, 2, s_time_buf, 16);
}

static void Oled_Refresh(void)
{
	switch (s_state)
	{
	case STATE_MENU:
		if (!s_oled_dirty)
			return;
		s_oled_dirty = false;
		Menu_Show();
		break;

	case STATE_READY:
		if (!s_oled_dirty)
			return;
		s_oled_dirty = false;
		Ready_Show();
		break;

	case STATE_RUNNING:
		{
			uint32_t now_ms = Board_GetTickMs();

			/* 首次进入：全屏绘制静态元素 */
			if (s_oled_dirty)
			{
				s_oled_dirty   = false;
				s_last_oled_ms = now_ms;
				Running_Show_First();
				return;
			}

			/* 每 100ms：只重写时间数字 */
			if (now_ms - s_last_oled_ms < 100)
				return;
			s_last_oled_ms = now_ms;
			Running_Show_Time(now_ms - s_start_ms);
		}
		break;
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

			/* KEY1：启动 + 开始计时 */
			if (Key_GetFlag(0))
			{
				s_start_ms     = Board_GetTickMs();
				s_last_oled_ms = 0;

				if (s_selected_task == TASK_BAL_STATIC)
				{
					Balance_Start();
				}
				else
				{
					Tracking_Start();
				}

				s_state      = STATE_RUNNING;
				s_oled_dirty = true;
			}
			break;

		case STATE_RUNNING:
			{
				bool stopped = false;

				/* KEY2：停止 */
				if (Key_GetFlag(1))
				{
					stopped = true;

					if (s_selected_task == TASK_BAL_STATIC)
						Balance_Stop();
					else
						Tracking_Stop();
				}

				/* 自动停止检测 */
				if (s_selected_task == TASK_BAL_STATIC)
				{
					if (Balance_IsDone())
						stopped = true;
				}
				else
				{
					if (!Tracking_IsRunning())
						stopped = true;
				}

				if (stopped)
				{
					s_last_time_ms = Board_GetTickMs() - s_start_ms;
					s_state        = STATE_READY;
					s_oled_dirty   = true;
				}
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
			{
				uint8_t mask = Grayscale_ReadAll();
				Tracking_Update(Board_GetTickMs(), mask);
			}
			break;

		case TASK_BAL_STATIC:
			Balance_SeqUpdate();
			break;
		}
	}
}
