/**
 * @file    ultrasonic.c
 * @brief   HC-SR04 超声波测距模块实现（非阻塞状态机）
 *
 * 测量原理：TRIG 发 15µs 高脉冲 → 模块发 40kHz 超声波 →
 * ECHO 输出高电平（时长 = 声波往返 µs）→ 双边沿中断捕获脉宽。
 *
 * 复用 PIT_CONTROL_TICK (TIMG0) 做 1µs 精度计时，无需额外定时器。
 * 测距公式: distance_mm = pulse_width_us / 5.8
 *
 * 非阻塞设计：Ultrasonic_Update() 驱动状态机，主循环中周期性调用即可，
 * 每 ~65ms 自动完成一次测距并更新滑动窗口均值。
 */

#include "ultrasonic.h"
#include "ti_msp_dl_config.h"
#include "board.h"
#include "delay.h"

/* ========== 常量 ========== */

/** 测距系数：distance_mm = pulse_us / 5.8 */
#define US_DISTANCE_FACTOR        5.8f

/** 两次触发间最小间隔 ms（规格书建议 ≥ 60ms） */
#define US_COOLDOWN_MS            65

/** 单次回波超时 ms（400cm ≈ 23ms，留足余量） */
#define US_ECHO_TIMEOUT_MS        100

/** 滑动窗口大小 */
#define US_WINDOW_SIZE            5

/** PIT_CONTROL_TICK 周期 tick 数（1MHz * 20ms） */
#define TIMG_PERIOD_TICKS         (PIT_CONTROL_TICK_INST_LOAD_VALUE + 1)

/* ========== 状态机 ========== */

typedef enum {
	US_STATE_IDLE,       /**< 就绪，可触发下一轮 */
	US_STATE_TRIGGERED,  /**< 已发触发脉冲，等待 ECHO 中断 */
	US_STATE_COOLDOWN,   /**< 本次完成，等待最短间隔 */
} US_State;

/* ========== 静态变量 ========== */

static US_State  s_state;                   /**< 当前状态 */
static uint32_t  s_trigger_tick;            /**< 当次触发时刻 (ms) */
static uint32_t  s_echo_start;              /**< 上升沿 TIMG0 计数值 */
static uint32_t  s_echo_width;              /**< 当次脉宽 (µs) */
static uint8_t   s_echo_done;               /**< ISR 置位：本次捕获完成 */

static float     s_window[US_WINDOW_SIZE];  /**< 滑动窗口样本 */
static uint8_t   s_window_idx;              /**< 窗口写入位置 */
static uint8_t   s_window_count;            /**< 窗口中有效样本数 */
static float     s_cached_avg;              /**< 缓存的均值 */
static bool      s_new_data;                /**< 自上次查询后是否有新数据 */

/* ========== 内部辅助 ========== */

/**
 * @brief 发送一次 TRIG 触发脉冲（~15µs 高电平）
 */
static void trigger(void)
{
	DL_GPIO_setPins(GPIO_ULTRASONIC_PORT, GPIO_ULTRASONIC_TRIGGER_PIN);
	delay_cycles(1200);  /* 15µs @ 80MHz */
	DL_GPIO_clearPins(GPIO_ULTRASONIC_PORT, GPIO_ULTRASONIC_TRIGGER_PIN);
}

/**
 * @brief 将新样本推入滑动窗口并更新缓存均值
 */
static void push_sample(float dist_mm)
{
	s_window[s_window_idx] = dist_mm;
	if (s_window_count < US_WINDOW_SIZE) s_window_count++;
	s_window_idx = (s_window_idx + 1) % US_WINDOW_SIZE;

	/* 重算均值 */
	float sum = 0.0f;
	uint8_t i;
	for (i = 0; i < s_window_count; i++)
	{
		sum += s_window[i];
	}
	s_cached_avg = sum / (float)s_window_count;
	s_new_data   = true;
}

/* ========== 公共 API ========== */

void Ultrasonic_Init(void)
{
	s_state        = US_STATE_IDLE;
	s_trigger_tick = 0;
	s_echo_start   = 0;
	s_echo_width   = 0;
	s_echo_done    = 0;

	s_window_idx   = 0;
	s_window_count = 0;
	s_cached_avg   = -1.0f;
	s_new_data     = false;
}

void Ultrasonic_Update(void)
{
	uint32_t now = Board_GetTickMs();

	switch (s_state)
	{
	case US_STATE_IDLE:
		s_echo_done = 0;
		trigger();
		s_trigger_tick = now;
		s_state = US_STATE_TRIGGERED;
		break;

	case US_STATE_TRIGGERED:
		if (s_echo_done)
		{
			float dist = (float)s_echo_width / US_DISTANCE_FACTOR;
			push_sample(dist);
			s_state = US_STATE_COOLDOWN;
		}
		else if (now - s_trigger_tick >= US_ECHO_TIMEOUT_MS)
		{
			/* 超时：本帧丢弃，进入冷却 */
			s_state = US_STATE_COOLDOWN;
		}
		break;

	case US_STATE_COOLDOWN:
		if (now - s_trigger_tick >= US_COOLDOWN_MS)
		{
			s_state = US_STATE_IDLE;
		}
		break;
	}
}

float Ultrasonic_GetDistance(void)
{
	return s_cached_avg;
}

bool Ultrasonic_IsNewData(void)
{
	bool ret   = s_new_data;
	s_new_data = false;
	return ret;
}

/* ========== 中断处理 ========== */

void Ultrasonic_ECHO_ISR(void)
{
	if (DL_GPIO_getEnabledInterruptStatus(GPIO_ULTRASONIC_PORT,
	                                      GPIO_ULTRASONIC_ECHO_PIN))
	{
		uint32_t count = DL_TimerG_getTimerCount(PIT_CONTROL_TICK_INST);

		if (DL_GPIO_readPins(GPIO_ULTRASONIC_PORT,
		                     GPIO_ULTRASONIC_ECHO_PIN) != 0)
		{
			/* 上升沿：记录起始时刻 */
			s_echo_start = count;
		}
		else
		{
			/* 下降沿：计算脉宽（处理计数器回绕） */
			if (count >= s_echo_start)
			{
				s_echo_width = count - s_echo_start;
			}
			else
			{
				s_echo_width = count - s_echo_start + TIMG_PERIOD_TICKS;
			}
			s_echo_done = 1;
		}

		DL_GPIO_clearInterruptStatus(GPIO_ULTRASONIC_PORT,
		                             GPIO_ULTRASONIC_ECHO_PIN);
	}
}
