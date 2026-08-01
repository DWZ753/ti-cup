/**
 * @file    menu.c
 * @brief   OLED 菜单与状态显示实现
 */

#include "menu.h"
#include "board.h"
#include "oled.h"
#include <string.h>

/* ========== 任务名查找表 ========== */

uint8_t *g_task_names[TASK_COUNT] = {
	(uint8_t*)"2.Run 1 Lap",
	(uint8_t*)"3.Ball Balance",
	(uint8_t*)"4.Run+Bal AB",
	(uint8_t*)"5.Run+Bal 1Lap",
	(uint8_t*)"6.Run+Bal Any",
	(uint8_t*)"7.Remote",
};

/* ========== 内部状态 ========== */

static bool      s_dirty;
static uint32_t  s_last_oled_ms;
static uint8_t   s_time_buf[8];

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

/* ========== 各状态显示函数 ========== */

static void Menu_Show(UI_TaskMode selected)
{
	uint8_t i;

	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"=== Task Select ===", 8);

	for (i = 0; i < TASK_COUNT; i++)
	{
		uint8_t row = 1 + i;

		if (i == (uint8_t)selected)
			OLED_ShowString(0, row, (uint8_t*)">", 8);
		OLED_ShowString(12, row, g_task_names[i], 8);
	}

	OLED_ShowString(0, 7, (uint8_t*)"K3:Next K4:OK", 8);
}

static void Ready_Show(UI_TaskMode task, uint32_t last_time_ms)
{
	OLED_Clear();
	OLED_ShowString(0, 0, (uint8_t*)"Task:", 8);
	OLED_ShowString(0, 2, g_task_names[task], 8);

	if (last_time_ms > 0)
	{
		Format_Time(last_time_ms);
		OLED_ShowString(0, 3, (uint8_t*)"Time:", 8);
		OLED_ShowString(30, 3, s_time_buf, 8);
	}

	OLED_ShowString(0, 5, (uint8_t*)"K1:Start K2:Stop", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K3:Back", 8);
}

static void Running_Show_First(UI_TaskMode task)
{
	OLED_Clear();
	OLED_ShowString(0, 0, g_task_names[task], 8);
	OLED_ShowString(0, 1, (uint8_t*)"Tgt:", 8);
	OLED_ShowString(0, 7, (uint8_t*)"K2:Stop", 8);

	Format_Time(0);
	OLED_ShowString(36, 2, s_time_buf, 16);

	OLED_ShowString(0, 4, (uint8_t*)"a:", 8);
	OLED_ShowString(64, 4, (uint8_t*)"F:", 8);
	OLED_ShowString(0, 5, (uint8_t*)"P:", 8);
	OLED_ShowString(64, 5, (uint8_t*)"D:", 8);
	OLED_ShowString(0, 6, (uint8_t*)"I:", 8);
}

/**
 * @brief 显示带符号整数（值 × scale，3 位）
 */
static void Show_Signed(int32_t val_x100, uint8_t x, uint8_t y, uint8_t scale)
{
	int32_t val;
	uint8_t sign;

	val  = val_x100 / 100;
	sign = (val >= 0) ? '+' : '-';
	if (val < 0) val = -val;
	OLED_ShowChar(x, y, sign, scale);
	OLED_ShowNum(x + (scale == 16 ? 8 : 0), y, (uint32_t)val, 3, scale);
}

static void Running_Show_Time(uint32_t elapsed_ms,
                              const UI_DebugValues *dbg)
{
	Format_Time(elapsed_ms);
	OLED_ShowString(36, 2, s_time_buf, 16);

	if (dbg == NULL) return;

	/* 加速度 (cm/s²) — ff_accel × 100 */
	Show_Signed((int32_t)(dbg->ff_accel * 100.0f * 100.0f), 16, 4, 8);

	/* FF 倾角 (0.1°) — ff_angle × 10 */
	Show_Signed((int32_t)(dbg->ff_angle * 10.0f * 100.0f), 76, 4, 8);

	/* P 项 (0.1°) */
	Show_Signed((int32_t)(dbg->pid_p * 10.0f * 100.0f), 16, 5, 8);

	/* D 项 (0.1°) */
	Show_Signed((int32_t)(dbg->pid_d * 10.0f * 100.0f), 80, 5, 8);

	/* I 项 (0.01°) */
	{
		int32_t val = (int32_t)(dbg->pid_i * 100.0f);
		uint8_t sign = (val >= 0) ? '+' : '-';
		if (val < 0) val = -val;
		OLED_ShowChar(16, 6, sign, 8);
		OLED_ShowNum(24, 6, (uint32_t)val, 4, 8);
	}

	/* 目标位置 (mm) */
	{
		int32_t tgt = (int32_t)dbg->target_mm;
		uint8_t sign = (tgt >= 0) ? '+' : '-';
		if (tgt < 0) tgt = -tgt;
		OLED_ShowChar(32, 1, sign, 8);
		OLED_ShowNum(40, 1, (uint32_t)tgt, 4, 8);
		OLED_ShowString(72, 1, (uint8_t*)"mm", 8);
	}

	/* Pi 球位置 (mm) */
	{
		int32_t pos = (int32_t)dbg->pi_ball_pos;
		uint8_t sign = (pos >= 0) ? '+' : '-';
		if (pos < 0) pos = -pos;
		OLED_ShowChar(64, 6, sign, 8);
		OLED_ShowNum(72, 6, (uint32_t)pos, 4, 8);
		OLED_ShowString(104, 6, (uint8_t*)"mm", 8);
	}
}

/* ========== 公共 API ========== */

void UI_Init(void)
{
	s_dirty        = true;
	s_last_oled_ms = 0;
}

void UI_Refresh(UI_State state, UI_TaskMode task,
                uint32_t elapsed_ms, uint32_t last_time_ms,
                const UI_DebugValues *dbg)
{
	switch (state)
	{
	case STATE_MENU:
		if (!s_dirty) return;
		s_dirty = false;
		Menu_Show(task);
		break;

	case STATE_READY:
		if (!s_dirty) return;
		s_dirty = false;
		Ready_Show(task, last_time_ms);
		break;

	case STATE_RUNNING:
		/*
		 * 首次进入：清屏 + 静态元素
		 * 后续：每 100ms 刷新时间+调试数据
		 */
		if (s_dirty)
		{
			s_dirty         = false;
			s_last_oled_ms  = elapsed_ms;
			Running_Show_First(task);
			return;
		}

		/* 100ms throttle — 由 main.c 管理，这里做 1s 刷低保真 */
		if (elapsed_ms - s_last_oled_ms < 100)
			return;
		s_last_oled_ms = elapsed_ms;
		Running_Show_Time(elapsed_ms, dbg);
		break;
	}
}

void UI_SetDirty(void)
{
	s_dirty = true;
}
