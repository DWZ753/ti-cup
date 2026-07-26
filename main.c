/**
 * @file    main.c
 * @brief   TM1637 四位数码管综合测试（自动循环）
 *
 * 测试项：
 *   1. 数字 0→9999（前导零）
 *   2. 数字 0→9999（前导消隐）
 *   3. 负数 -1 / -12 / -123 / -999
 *   4. 时间格式 MM:SS（冒号闪烁）
 *   5. 亮度 0→7 循环（显示 8888）
 *   6. 段测试（逐位点亮全部段）
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "tm1637.h"

/* 每阶段 5 秒，每步 200ms */
#define PHASE_DURATION_MS   5000
#define STEP_INTERVAL_MS     200

typedef enum {
	TEST_NUM_LEADING_ZERO,
	TEST_NUM_NO_ZERO,
	TEST_NEGATIVE,
	TEST_TIME,
	TEST_BRIGHTNESS,
	TEST_SEGMENTS,
	TEST_COUNT
} TestPhase;

int main(void)
{
	TestPhase phase       = 0;
	uint32_t  phase_start;
	uint32_t  step_last;
	uint16_t  counter     = 0;
	uint8_t   sec         = 0;
	uint8_t   brightness  = 7;
	int8_t    bright_dir  = 1;
	uint8_t   seg_pos     = 0;
	bool      colon_state = true;

	SYSCFG_DL_init();
	Board_Init();

	TM1637_Init();
	TM1637_DisplayNum(0, true);

	phase_start = Board_GetTickMs();
	step_last   = phase_start;

	while (1)
	{
		uint32_t now = Board_GetTickMs();

		/* ---- 阶段切换 ---- */
		if (now - phase_start >= PHASE_DURATION_MS)
		{
			phase_start = now;
			step_last   = now;
			counter     = 0;
			sec         = 0;
			brightness  = 7;
			bright_dir  = 1;
			seg_pos     = 0;

			phase++;
			if (phase >= TEST_COUNT) phase = 0;

			TM1637_Clear();
		}

		/* ---- 步进控制 ---- */
		if (now - step_last < STEP_INTERVAL_MS) continue;
		step_last = now;

		switch (phase)
		{
		/* ---- 1. 前导零：0000→9999 ---- */
		case TEST_NUM_LEADING_ZERO:
			TM1637_DisplayNum((int16_t)counter, true);
			counter++;
			if (counter > 9999) counter = 0;
			break;

		/* ---- 2. 前导消隐：0→9999 ---- */
		case TEST_NUM_NO_ZERO:
			TM1637_DisplayNum((int16_t)counter, false);
			counter++;
			if (counter > 9999) counter = 0;
			break;

		/* ---- 3. 负数 ---- */
		case TEST_NEGATIVE:
		{
			int16_t neg_vals[] = {-1, -12, -123, -999};
			TM1637_DisplayNum(neg_vals[counter % 4], false);
			counter++;
			break;
		}

		/* ---- 4. 时间 MM:SS（冒号 400ms 周期闪烁） ---- */
		case TEST_TIME:
		{
			uint8_t min = sec / 60;
			uint8_t s   = sec % 60;

			if (counter % 2 == 0) colon_state = !colon_state;

			TM1637_DisplayTime(min, s, colon_state);
			counter++;
			sec++;
			if (sec >= 3600) sec = 0;
			break;
		}

		/* ---- 5. 亮度 0→7→0（显示 8888） ---- */
		case TEST_BRIGHTNESS:
		{
			uint8_t segs[4] = {0x7F, 0x7F, 0x7F, 0x7F}; /* 8. 不含 DP */

			TM1637_SetBrightness(brightness);
			TM1637_DisplaySegments(segs);

			brightness = (uint8_t)((int8_t)brightness + bright_dir);
			if (brightness >= 7) bright_dir = -1;
			if (brightness == 0) bright_dir = 1;

			counter++;
			break;
		}

		/* ---- 6. 逐位点亮全部段（含小数点） ---- */
		case TEST_SEGMENTS:
		{
			uint8_t segs[4] = {0, 0, 0, 0};
			segs[seg_pos] = 0xFF;

			TM1637_DisplaySegments(segs);

			seg_pos++;
			if (seg_pos >= 4) seg_pos = 0;
			counter++;
			break;
		}

		default:
			break;
		}
	}
}
