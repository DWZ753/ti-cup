/**
 * @file    main.c
 * @brief   智能物流搬运系统 — 无线计分显示装置 主程序
 *
 * 功能：
 *   - 通过按键发送启动/停止/急停指令给小车
 *   - 实时显示已搬运数量和系统状态（TM1637 数码管 + OLED）
 *   - 记录并显示单球耗时、平均时间、总耗时
 *   - 三色 LED 指示系统状态（绿=运行 / 黄=待命 / 红=故障）
 *   - 全部搬运完成后蜂鸣器提示
 *
 * 通信：UART1 → 无线模块 → 小车，COBS + XOR 帧协议
 * 协议命令码定义于 application/comm/protocol_commands.h
 */

#include "ti_msp_dl_config.h"
#include "board.h"
#include "protocol.h"
#include "protocol_commands.h"

/* ========================================================================
 * 常量
 * ======================================================================== */

#define TOTAL_BALLS        5       /**< 钢球总数量 */
#define COMPLETE_BEEP_MS   500    /**< 完成提示蜂鸣时长 ms */
#define COMPLETE_HOLD_MS   3000    /**< 完成后保持显示时长 ms */
#define OLED_REFRESH_MS    200     /**< OLED 刷新间隔 ms */

/* ---- 数码管状态字符段码 ---- */
#define SEG_DASH           0x40    /**< '-' 段码 */
#define SEG_BLANK          0x00    /**< 全灭 */
#define SEG_E              0x79    /**< 'E' 段码 */
#define SEG_n              0x54    /**< 'n' 段码 */
#define SEG_d              0x5E    /**< 'd' 段码 */
#define SEG_F              0x71    /**< 'F' 段码 */
#define SEG_A              0x77    /**< 'A' 段码 */
#define SEG_I              0x30    /**< 'I' 段码 */
#define SEG_L              0x38    /**< 'L' 段码 */
#define SEG_r              0x50    /**< 'r' 段码 */

/* LED 电平: 低电平点亮 */
#define LED_ON(port, pin)  DL_GPIO_clearPins((port), (pin))
#define LED_OFF(port, pin) DL_GPIO_setPins((port), (pin))

/* ========================================================================
 * 状态机
 * ======================================================================== */

typedef enum {
	STATE_IDLE,         /**< 待命：等待启动指令 */
	STATE_RUNNING,      /**< 运行中：小车正在搬运 */
	STATE_COMPLETE,     /**< 完成：5 颗球全部搬运完毕 */
	STATE_FAULT,        /**< 故障：急停或小车上报故障 */
} state_e;

/* ========================================================================
 * 运行时上下文
 * ======================================================================== */

static struct {
	state_e  state;                     /**< 当前状态 */
	uint8_t  ball_count;                /**< 已搬运数量 (0~5) */
	uint16_t ball_time_s[TOTAL_BALLS]; /**< 每颗球耗时 (秒)，本地计时 */
	uint32_t task_start_ms;             /**< 任务开始时刻 (ms) */
	uint32_t last_ball_ms;              /**< 上一颗球到达时刻 (ms) */
	uint32_t state_enter_ms;            /**< 进入当前状态的时刻 (ms) */
	uint32_t last_oled_ms;              /**< OLED 上次刷新时刻 */
} g_ctx;

/* ========================================================================
 * 内部函数声明
 * ======================================================================== */

/* ---- 状态切换 ---- */
static void enter_idle(void);
static void enter_running(void);
static void enter_complete(void);
static void enter_fault(void);

/* ---- 通信回调 ---- */
static void on_frame(uint8_t cmd, const uint8_t *payload, uint8_t len);

/* ---- 数码管显示 ---- */
static void tm_show_count(uint8_t count);
static void tm_show_text(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3);

/* ---- 按键处理 ---- */
static void handle_keys(void);

/* ---- OLED 刷新 ---- */
static void oled_update(void);

/* ---- LED 刷新 ---- */
static void led_update(state_e state);

/* ========================================================================
 * 状态切换
 * ======================================================================== */

static void enter_idle(void)
{
	g_ctx.state          = STATE_IDLE;
	g_ctx.state_enter_ms = Board_GetTickMs();

	tm_show_text(SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH);
	led_update(STATE_IDLE);
}

static void enter_running(void)
{
	g_ctx.state          = STATE_RUNNING;
	g_ctx.ball_count     = 0;
	g_ctx.task_start_ms  = Board_GetTickMs();
	g_ctx.last_ball_ms   = g_ctx.task_start_ms;
	g_ctx.state_enter_ms = g_ctx.task_start_ms;

	tm_show_count(0);
	led_update(STATE_RUNNING);
}

static void enter_complete(void)
{
	g_ctx.state          = STATE_COMPLETE;
	g_ctx.state_enter_ms = Board_GetTickMs();

	/* 数码管显示 "End" */
	tm_show_text(SEG_E, SEG_n, SEG_d, SEG_BLANK);

	/* 蜂鸣器提示 */
	Buzzer_Beep(COMPLETE_BEEP_MS);

	/* LED 绿灯闪烁表示完成 */
	led_update(STATE_COMPLETE);
}

static void enter_fault(void)
{
	g_ctx.state          = STATE_FAULT;
	g_ctx.state_enter_ms = Board_GetTickMs();

	/* 数码管显示 "FAIL" */
	tm_show_text(SEG_F, SEG_A, SEG_I, SEG_L);
	led_update(STATE_FAULT);
}

/* ========================================================================
 * 通信：接收帧回调
 * ======================================================================== */

static void on_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	uint8_t  new_count;
	uint32_t now;

	switch (cmd)
	{
	case CMD_COUNT:
		/* 小车上报已搬运数量: payload = uint8 */
		if (len >= 1 && g_ctx.state == STATE_RUNNING)
		{
			new_count = payload[0];
			if (new_count > TOTAL_BALLS)
				new_count = TOTAL_BALLS;

			/* 数量增加时，本地记录与前一颗球的时间差 */
			if (new_count > g_ctx.ball_count)
			{
				now = Board_GetTickMs();
				g_ctx.ball_time_s[new_count - 1] =
					(uint16_t)((now - g_ctx.last_ball_ms) / 1000);
				g_ctx.last_ball_ms = now;
			}

			g_ctx.ball_count = new_count;
			tm_show_count(g_ctx.ball_count);

			/* 全部搬运完毕 → 自动进入完成状态 */
			if (g_ctx.ball_count >= TOTAL_BALLS)
			{
				enter_complete();
			}
		}
		break;

	case CMD_WEIGHT:
		/* 小车上报重量: payload = uint16 大端，单位 g */
		/* 仅记录，OLED 显示用（暂未实现） */
		break;

	case CMD_STATUS:
		/* 小车上报系统状态 */
		if (len >= 1 && payload[0] == STATUS_FAULT)
		{
			enter_fault();
		}
		break;

	default:
		break;
	}
}

/* ========================================================================
 * 通信：发送指令
 * ======================================================================== */

/** @brief 向小车发送无载荷指令 */
static void send_cmd(uint8_t cmd)
{
	Protocol_SendFrame(cmd, NULL, 0);
}

/* ========================================================================
 * 按键处理
 * ======================================================================== */

static void handle_keys(void)
{
	/* KEY1: 启动 (仅 IDLE 状态有效) */
	if (Key_GetFlag(0))
	{
		if (g_ctx.state == STATE_IDLE)
		{
			send_cmd(CMD_START);
			enter_running();
		}
	}

	/* KEY2: 正常停止 (仅 RUNNING 状态有效) */
	if (Key_GetFlag(1))
	{
		if (g_ctx.state == STATE_RUNNING)
		{
			send_cmd(CMD_STOP);
			enter_idle();
		}
	}

	/* KEY3: 急停 (RUNNING 状态有效) */
	if (Key_GetFlag(2))
	{
		if (g_ctx.state == STATE_RUNNING)
		{
			send_cmd(CMD_EMERGENCY);
			enter_fault();
		}
	}

	/* KEY4: 复位 (FAULT 或 COMPLETE 状态回到 IDLE) */
	if (Key_GetFlag(3))
	{
		if (g_ctx.state == STATE_FAULT
		    || g_ctx.state == STATE_COMPLETE)
		{
			enter_idle();
		}
	}
}

/* ========================================================================
 * 数码管显示
 * ======================================================================== */

/**
 * @brief 显示搬运数量（例: "0003" 表示 3 颗）
 * @param count 已搬运数量 0~5
 */
static void tm_show_count(uint8_t count)
{
	TM1637_DisplayNum((int16_t)count, true);
}

/**
 * @brief 显示自定义 4 字符（用于状态文字）
 */
static void tm_show_text(uint8_t s0, uint8_t s1, uint8_t s2, uint8_t s3)
{
	uint8_t segs[4] = { s0, s1, s2, s3 };
	TM1637_DisplaySegments(segs);
}

/* ========================================================================
 * OLED 刷新
 * ======================================================================== */

static void oled_update(void)
{
	uint32_t now       = Board_GetTickMs();
	uint32_t elapsed_s;
	uint32_t avg_s;
	uint8_t  cnt;
	uint8_t  i;
	uint32_t total_s;
	char     line[17];  /* OLED 每行 16 字符 + '\0' */

	if (now - g_ctx.last_oled_ms < OLED_REFRESH_MS) return;
	g_ctx.last_oled_ms = now;

	/* ---- 第 0 行: 系统状态 ---- */
	switch (g_ctx.state)
	{
	case STATE_IDLE:
		OLED_ShowString(0, 0, (uint8_t *)"Sta: IDLE      ", 16);
		break;
	case STATE_RUNNING:
		OLED_ShowString(0, 0, (uint8_t *)"Sta: RUNNING   ", 16);
		break;
	case STATE_COMPLETE:
		OLED_ShowString(0, 0, (uint8_t *)"Sta: COMPLETE  ", 16);
		break;
	case STATE_FAULT:
		OLED_ShowString(0, 0, (uint8_t *)"Sta: FAULT     ", 16);
		break;
	}

	/* ---- 第 1 行: 搬运进度 ---- */
	if (g_ctx.state == STATE_IDLE)
	{
		OLED_ShowString(0, 2, (uint8_t *)"Press KEY1 start", 16);
	}
	else
	{
		cnt = g_ctx.ball_count;
		(void)snprintf(line, sizeof(line),
			"Ball: %d / %d      ", cnt, TOTAL_BALLS);
		OLED_ShowString(0, 2, (uint8_t *)line, 16);
	}

	/* ---- 第 2 行: 耗时信息 ---- */
	if (g_ctx.state == STATE_RUNNING || g_ctx.state == STATE_COMPLETE)
	{
		elapsed_s = (now - g_ctx.task_start_ms) / 1000;

		/* 计算单球平均时间 */
		cnt = g_ctx.ball_count;
		if (cnt > 0)
		{
			total_s = 0;
			for (i = 0; i < cnt; i++)
				total_s += g_ctx.ball_time_s[i];
			avg_s = total_s / cnt;
		}
		else
		{
			total_s = 0;
			avg_s   = 0;
		}

		(void)snprintf(line, sizeof(line),
			"T:%lus Avg:%lus   ", (unsigned long)elapsed_s,
			(unsigned long)avg_s);
		OLED_ShowString(0, 4, (uint8_t *)line, 16);
	}
	else if (g_ctx.state == STATE_FAULT)
	{
		OLED_ShowString(0, 4, (uint8_t *)"KEY4 -> reset   ", 16);
	}
	else
	{
		OLED_ShowString(0, 4, (uint8_t *)"                ", 16);
	}

	/* ---- 第 3 行: 辅助信息 ---- */
	if (g_ctx.state == STATE_RUNNING && g_ctx.ball_count > 0)
	{
		(void)snprintf(line, sizeof(line),
			"Last:%lus         ",
			(unsigned long)g_ctx.ball_time_s[g_ctx.ball_count - 1]);
		OLED_ShowString(0, 6, (uint8_t *)line, 16);
	}
	else
	{
		OLED_ShowString(0, 6, (uint8_t *)"                ", 16);
	}
}

/* ========================================================================
 * LED 状态指示
 * ======================================================================== */

/**
 * @brief 根据状态设置三色 LED
 *
 * IDLE      → 黄灯亮
 * RUNNING   → 绿灯亮
 * COMPLETE  → 绿灯闪烁（主循环控制）
 * FAULT     → 红灯亮
 */
static void led_update(state_e state)
{
	/* 全部先灭 */
	LED_OFF(GPIO_LEDs_GPIO_LED_GREEN_PORT,  GPIO_LEDs_GPIO_LED_GREEN_PIN);
	LED_OFF(GPIO_LEDs_GPIO_LED_YELLOW_PORT, GPIO_LEDs_GPIO_LED_YELLOW_PIN);
	LED_OFF(GPIO_LEDs_GPIO_LED_RED_PORT,    GPIO_LEDs_GPIO_LED_RED_PIN);

	switch (state)
	{
	case STATE_IDLE:
		LED_ON(GPIO_LEDs_GPIO_LED_YELLOW_PORT, GPIO_LEDs_GPIO_LED_YELLOW_PIN);
		break;
	case STATE_RUNNING:
		LED_ON(GPIO_LEDs_GPIO_LED_GREEN_PORT, GPIO_LEDs_GPIO_LED_GREEN_PIN);
		break;
	case STATE_COMPLETE:
		/* 绿灯闪烁：每 300ms 翻转，由主循环处理 */
		break;
	case STATE_FAULT:
		LED_ON(GPIO_LEDs_GPIO_LED_RED_PORT, GPIO_LEDs_GPIO_LED_RED_PIN);
		break;
	}
}

/* ========================================================================
 * 主函数
 * ======================================================================== */

int main(void)
{
	SYSCFG_DL_init();
	Board_Init();

	/* ---- 通信初始化 ---- */
	Protocol_Init(on_frame, Board_GetUART_BT());

	/* ---- OLED 初始界面 ---- */
	OLED_ShowString(0, 0, (uint8_t *)"TI Cup 2026    ", 16);
	OLED_ShowString(0, 2, (uint8_t *)"Smart Logistics", 16);
	OLED_ShowString(0, 4, (uint8_t *)"Display Console ", 16);
	OLED_ShowString(0, 6, (uint8_t *)"KEY1 = START   ", 16);

	/* ---- 进入待命状态 ---- */
	enter_idle();

	while (1)
	{
		uint32_t now = Board_GetTickMs();

		/* 收帧：处理小车发来的数据 */
		Protocol_Update();

		/* 按键扫描与处理 */
		handle_keys();

		/* COMPLETE 状态：绿灯闪烁 + 超时自动回 IDLE */
		if (g_ctx.state == STATE_COMPLETE)
		{
			if (now - g_ctx.state_enter_ms >= COMPLETE_HOLD_MS)
			{
				enter_idle();
			}
			else
			{
				/* 每 300ms 翻转绿灯 */
				if (((now - g_ctx.state_enter_ms) / 300) & 1)
					LED_ON(GPIO_LEDs_GPIO_LED_GREEN_PORT,
					       GPIO_LEDs_GPIO_LED_GREEN_PIN);
				else
					LED_OFF(GPIO_LEDs_GPIO_LED_GREEN_PORT,
					        GPIO_LEDs_GPIO_LED_GREEN_PIN);
			}
		}

		/* OLED 定期刷新 */
		oled_update();
	}
}
