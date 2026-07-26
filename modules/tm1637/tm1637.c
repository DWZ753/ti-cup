/**
 * @file    tm1637.c
 * @brief   TM1637 四位数码管驱动（两线自定义串行协议）
 *
 * CLK 始终为输出，DIO 写数据时为输出、ACK 位跳过检测。
 * 时序参照 Arduino TM1637Display 库，与测试模块 A1-2-3F 兼容。
 */

#include "tm1637.h"
#include "ti_msp_dl_config.h"

/* ========== 常量 ========== */

/** 2µs 延时周期数 @ 80MHz CPUCLK_FREQ */
#define TM1637_DELAY_CYCLES  160

/** TM1637 命令 */
#define TM1637_CMD_DATA      0x40  /**< 数据写，自动地址递增 */
#define TM1637_CMD_ADDR      0xC0  /**< 起始地址 (GRID1) */
#define TM1637_CMD_DISP_ON   0x88  /**< 显示开 + 亮度低 3 位 */

/** 段码：负号 */
#define TM1637_SEG_MINUS     0x40

/* ========== 7 段字库（0~9，与 Arduino 参考代码一致） ========== */

static const uint8_t s_font[] = {
	0x3F, /* 0 */
	0x06, /* 1 */
	0x5B, /* 2 */
	0x4F, /* 3 */
	0x66, /* 4 */
	0x6D, /* 5 */
	0x7D, /* 6 */
	0x07, /* 7 */
	0x7F, /* 8 */
	0x6F, /* 9 */
};

/* ========== 静态状态 ========== */

static uint8_t s_brightness; /**< 当前亮度 0~7 */

/* ========== 底层协议（私有） ========== */

/**
 * @brief 发送 start 条件：DIO↓ → CLK↓
 * @note  调用前 CLK/DIO 应在 idle 态（均为 HIGH）
 */
static void tm1637_start(void)
{
	DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_DIO_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
}

/**
 * @brief 发送 stop 条件：DIO↓ → CLK↑ → DIO↑
 */
static void tm1637_stop(void)
{
	DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_DIO_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_setPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_setPins(GPIO_TM1637_PORT, GPIO_TM1637_DIO_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
}

/**
 * @brief 向 TM1637 写入一个字节（含 ACK 时钟位）
 * @param data 待发送字节
 * @note  LSB first。ACK 位仅发送时钟脉冲，不检测应答。
 */
static void tm1637_write_byte(uint8_t data)
{
	uint8_t i;

	for (i = 0; i < 8; i++)
	{
		DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
		delay_cycles(TM1637_DELAY_CYCLES);

		if (data & 0x01)
		{
			DL_GPIO_setPins(GPIO_TM1637_PORT,
			                GPIO_TM1637_DIO_PIN);
		}
		else
		{
			DL_GPIO_clearPins(GPIO_TM1637_PORT,
			                  GPIO_TM1637_DIO_PIN);
		}

		delay_cycles(TM1637_DELAY_CYCLES);
		DL_GPIO_setPins(GPIO_TM1637_PORT,
		                GPIO_TM1637_CLK_PIN);
		delay_cycles(TM1637_DELAY_CYCLES);
		data >>= 1;
	}

	/* ACK 时钟位：释放 DIO → 第 9 个脉冲 → 忽略应答 */
	DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_setPins(GPIO_TM1637_PORT, GPIO_TM1637_DIO_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_setPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
	DL_GPIO_clearPins(GPIO_TM1637_PORT, GPIO_TM1637_CLK_PIN);
	delay_cycles(TM1637_DELAY_CYCLES);
}

/* ======================================================================== */
/*  API                                                                      */
/* ======================================================================== */

void TM1637_Init(void)
{
	/* 初始化为输出、高电平空闲态（SysConfig 已配，此处设初始值） */
	DL_GPIO_setPins(GPIO_TM1637_PORT,
	                GPIO_TM1637_CLK_PIN | GPIO_TM1637_DIO_PIN);

	s_brightness = 7;

	/* 上电后发一次亮度命令，确保显示关闭 */
	tm1637_start();
	tm1637_write_byte(0x80); /* 显示关 */
	tm1637_stop();

	TM1637_Clear();
}

void TM1637_DisplaySegments(const uint8_t segs[4])
{
	uint8_t i;

	/* 写数据命令 */
	tm1637_start();
	tm1637_write_byte(TM1637_CMD_DATA);
	tm1637_stop();

	/* 写地址 + 4 字节段码 */
	tm1637_start();
	tm1637_write_byte(TM1637_CMD_ADDR);
	for (i = 0; i < 4; i++)
	{
		tm1637_write_byte(segs[i]);
	}
	tm1637_stop();

	/* 显示开 + 亮度 */
	tm1637_start();
	tm1637_write_byte(TM1637_CMD_DISP_ON | (s_brightness & 0x07));
	tm1637_stop();
}

void TM1637_Clear(void)
{
	uint8_t blank[4] = {0, 0, 0, 0};

	TM1637_DisplaySegments(blank);
}

void TM1637_SetBrightness(uint8_t level)
{
	s_brightness = level & 0x07;

	tm1637_start();
	tm1637_write_byte(TM1637_CMD_DISP_ON | s_brightness);
	tm1637_stop();
}

uint8_t TM1637_EncodeDigit(uint8_t digit)
{
	if (digit > 9) return 0;

	return s_font[digit];
}

void TM1637_DisplayNum(int16_t num, bool leading_zero)
{
	uint8_t segs[4];
	bool    negative;
	uint8_t d3, d2, d1, d0;

	if (num < 0)
	{
		negative = true;
		num      = -num;
		if (num > 999) num = 999;
	}
	else
	{
		negative = false;
		if (num > 9999) num = 9999;
	}

	d3 = (num / 1000) % 10;
	d2 = (num / 100) % 10;
	d1 = (num / 10) % 10;
	d0 = num % 10;

	if (negative)
	{
		segs[0] = TM1637_SEG_MINUS;
		segs[1] = s_font[d2];
		segs[2] = s_font[d1];
		segs[3] = s_font[d0];

		if (!leading_zero)
		{
			if (d2 == 0) { segs[1] = 0; if (d1 == 0) { segs[2] = 0; } }
		}
	}
	else
	{
		segs[0] = s_font[d3];
		segs[1] = s_font[d2];
		segs[2] = s_font[d1];
		segs[3] = s_font[d0];

		if (!leading_zero)
		{
			if (d3 == 0)
			{
				segs[0] = 0;
				if (d2 == 0)
				{
					segs[1] = 0;
					if (d1 == 0)
					{
						segs[2] = 0;
					}
				}
			}
		}
	}

	TM1637_DisplaySegments(segs);
}

void TM1637_DisplayTime(uint8_t min, uint8_t sec, bool colon_on)
{
	uint8_t segs[4];

	if (min > 99) min = 99;
	if (sec > 59) sec = 59;

	segs[0] = s_font[min / 10];
	segs[1] = s_font[min % 10];
	segs[2] = s_font[sec / 10];
	segs[3] = s_font[sec % 10];

	if (colon_on)
	{
		segs[1] |= 0x80;
	}

	TM1637_DisplaySegments(segs);
}
