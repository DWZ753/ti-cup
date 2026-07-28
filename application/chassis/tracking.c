/**
 * @file    tracking.c
 * @brief   循迹模块实现
 *
 * 直角弯检测有两条路径（共用同一计数器，先到先触发）：
 *   1. 位置跳变（主）：|position| > CORNER_POS_THRESHOLD
 *      接近弯道时线偏移到阵列边缘，方向信号最清晰。
 *   2. 丢线（后备）：position == 99.0f（全白）
 *      转弯过程中或漏过第 1 条时触发。
 *
 * 方向推断：
 *   跳变触发时 s_corner_position 记录了跳变瞬间的位置，
 *   正值 = 线偏右 = 需要右转，负值 = 线偏左 = 需要左转。
 *   丢线触发时用丢线前最后有效位置，但可能不如跳变准确。
 */

#include "tracking.h"

/* ========== 内部状态 ========== */

typedef enum {
	TR_STATE_ON_LINE,         /**< 正常循迹中 */
	TR_STATE_IN_CORNER,       /**< 确认直角弯，转弯中 */
} tr_state_e;

static tr_state_e          s_state;            /**< 当前状态 */
static uint8_t             s_corner_debounce;  /**< 连续满足触发条件的帧数 */
static uint8_t             s_reacquire_cnt;    /**< 转弯中连续读到线计数 */
static float               s_current_position; /**< 最近一次位置值 */
static float               s_corner_position;  /**< 跳变瞬间的位置（方向依据） */
static tracking_turn_dir_e s_force_dir;        /**< 上层强制方向 */

/* ========== 位置解算 ========== */

float Tracking_CalcPosition(uint8_t mask)
{
	uint8_t bits = ~mask;   /* 取反后 1 = 黑线 */
	float   sum_weight = 0;
	float   count      = 0;

	for (uint8_t i = 0; i < SENSOR_COUNT; i++)
	{
		if (bits & (1 << i))
		{
			sum_weight += (float)i;
			count      += 1.0f;
		}
	}

	if (count == 0)
		return 99.0f;

	float center = sum_weight / count;
	return (center - SENSOR_CENTER) / SENSOR_CENTER;
}

/* ========== 直角弯检测 ========== */

/**
 * @brief 判断当前帧是否满足直角弯触发条件
 * @retval true  位置跳变 或 全白丢线
 */
static bool is_corner_trigger(float pos)
{
	if (pos == 99.0f)
		return true;

	if (pos > TRACKING_CORNER_POS_THRESHOLD
	 || pos < -TRACKING_CORNER_POS_THRESHOLD)
		return true;

	return false;
}

void Tracking_Update(uint8_t mask)
{
	float position = Tracking_CalcPosition(mask);
	s_current_position = position;

	if (s_state == TR_STATE_ON_LINE)
	{
		if (is_corner_trigger(position))
		{
			/* 位置跳变或丢线 */
			/*
			 * 跳变时 position 是有效值（偏极端），它比丢线时的
			 * s_corner_position 更可靠——直接记录当前跳变位置。
			 * 丢线时 position=99.0f，这里不覆盖，保留之前记录的值。
			 */
			if (position != 99.0f)
				s_corner_position = position;

			s_corner_debounce++;
			if (s_corner_debounce >= TRACKING_CORNER_DEBOUNCE)
				s_state = TR_STATE_IN_CORNER;
		}
		else
		{
			/* 正常循迹：重置计数器，持续更新位置 */
			s_corner_debounce = 0;
			s_corner_position = position;
		}
	}
	else
	{
		/* 转弯中：检测重新捕获 */
		if (position != 99.0f)
			s_reacquire_cnt++;
		else
			s_reacquire_cnt = 0;
	}
}

bool Tracking_IsCorner(void)
{
	return s_state == TR_STATE_IN_CORNER;
}

tracking_turn_dir_e Tracking_GetTurnDirection(void)
{
	if (s_force_dir != TURN_UNKNOWN)
		return s_force_dir;

	/*
	 * 用跳变瞬间记录的位置判断方向。
	 * 正值 = 线在传感器阵列右侧 = 车应该右转去追线。
	 */
	if (s_corner_position > TRACKING_CORNER_POS_THRESHOLD)
		return TURN_RIGHT;

	if (s_corner_position < -TRACKING_CORNER_POS_THRESHOLD)
		return TURN_LEFT;

	return TURN_UNKNOWN;
}

void Tracking_SetTurnDirection(tracking_turn_dir_e dir)
{
	s_force_dir = dir;
}

bool Tracking_IsLineReacquired(void)
{
	return s_reacquire_cnt >= TRACKING_REACQUIRE_CNT;
}

float Tracking_GetPosition(void)
{
	return s_current_position;
}

void Tracking_Reset(void)
{
	s_state           = TR_STATE_ON_LINE;
	s_corner_debounce = 0;
	s_reacquire_cnt   = 0;
}
