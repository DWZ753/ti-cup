/**
 * @file    comm_handler.c
 * @brief   通信处理器：协议绑定 + 命令分发 + 传感上报
 */

#include "comm_handler.h"
#include "protocol.h"
#include "board.h"
#include "oled.h"
#include "ultrasonic.h"
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* ========== 常量 ========== */

#define OLED_LINE_CHARS 16
#define OLED_LINES      4
#define OLED_LINE_STEP  2

/* ========== 静态状态 ========== */

static Comm_Config s_cfg;
static uint32_t    s_last_imu;
static uint32_t    s_last_us;

/* ========== 辅助：大端 int16 写入 ========== */

static void put_i16(uint8_t *buf, int16_t val)
{
	buf[0] = (uint8_t)((val >> 8) & 0xFF);
	buf[1] = (uint8_t)(val & 0xFF);
}

/* ========== 命令处理 ========== */

static void handle_oled_show(const uint8_t *payload, uint8_t len)
{
	uint8_t line;
	uint8_t copy_len;
	uint8_t text[OLED_LINE_CHARS + 1];
	uint8_t i;

	if (len < 2) return;

	line = payload[0];
	if (line >= OLED_LINES || (line % OLED_LINE_STEP) != 0) return;

	copy_len = len - 1;
	if (copy_len > OLED_LINE_CHARS) copy_len = OLED_LINE_CHARS;

	for (i = 0; i < copy_len; i++)
	{
		text[i] = payload[i + 1];
	}
	text[copy_len] = '\0';

	OLED_ShowString(0, line, (uint8_t *)"                ", 16);
	OLED_ShowString(0, line, text, 16);
}

static void on_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	uint8_t info[OLED_LINE_CHARS + 1];
	uint8_t i;

	switch (cmd)
	{
	case CMD_OLED_SHOW:
		handle_oled_show(payload, len);
		break;

	default:
		for (i = 0; i < OLED_LINE_CHARS; i++)
		{
			info[i] = ' ';
		}
		snprintf((char *)info, sizeof(info), "CMD 0x%02X L=%d", cmd, len);
		OLED_ShowString(0, 0, (uint8_t *)"                ", 16);
		OLED_ShowString(0, 0, info, 16);
		break;
	}
}

/* ========== 传感上报 ========== */

static void report_ultrasonic(void)
{
	float   dist_mm;
	int16_t dist_raw;
	uint8_t buf[2];

	if (!Ultrasonic_IsNewData()) return;

	dist_mm  = Ultrasonic_GetDistance();
	dist_raw = (int16_t)dist_mm;

	put_i16(buf, dist_raw);
	Comm_SendFrame(CMD_ULTRASONIC, buf, 2);

	/* 刷新 OLED 第 2 行 */
	char dist_str[OLED_LINE_CHARS + 1];
	snprintf(dist_str, sizeof(dist_str), "dist: %d mm", (int)dist_mm);
	OLED_ShowString(0, 2, (uint8_t *)"                ", 16);
	OLED_ShowString(0, 2, (uint8_t *)dist_str, 16);
}

/* ========== 公共 API ========== */

void Comm_Init(const Comm_Config *cfg)
{
	s_cfg     = *cfg;
	s_last_imu = 0;
	s_last_us  = 0;

	Protocol_Init(on_frame, s_cfg.uart);
	Comm_SendFrame(CMD_STATUS, (const uint8_t *)"MSPM0 online", 12);
}

void Comm_Update(void)
{
	uint32_t now = Board_GetTickMs();

	Protocol_Update();

	/* 超声波状态机推进（不管是否上报都要调） */
	Ultrasonic_Update();

	/* ---- 定时上报 ---- */

	if (s_cfg.ultrasonic_report_ms > 0)
	{
		if (now - s_last_us >= s_cfg.ultrasonic_report_ms)
		{
			s_last_us = now;
			report_ultrasonic();
		}
	}
}

void Comm_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	Protocol_SendFrame(cmd, payload, len);
}
