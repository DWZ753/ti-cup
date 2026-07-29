/**
 * @file    tracking.c
 * @brief   循迹模块实现 — 加权平均法位置解算
 */

#include "tracking.h"

/**
 * @brief 根据灰度传感器原始读数计算黑线位置
 *
 * 加权平均法：将每个检测到黑线的传感器索引作为权重，
 * 计算黑线中心位置，归一化到 [-1, +1]。
 *
 *        传感器阵列:  0   1   2   3   4   5   6   7
 *        归一化位置: -1.0                      +1.0
 *
 * @param mask 灰度传感器原始读数（0=黑/压线, 1=白）
 * @return 归一化位置 [-1.0, +1.0]，99.0f = 完全丢线
 */
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
