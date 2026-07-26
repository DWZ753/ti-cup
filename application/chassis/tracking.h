#ifndef __TRACKING_H__
#define __TRACKING_H__

#include "ti_msp_dl_config.h"
#include <stdint.h>

#define SENSOR_COUNT    8
#define SENSOR_CENTER   ((SENSOR_COUNT - 1) / 2.0f)

/**
 * @brief 根据灰度传感器原始读数计算黑线位置
 * @param mask 8 位掩码，bit[i] 对应第 i 号传感器，
 *             1 = 高电平（白色/未压线），0 = 低电平（黑色/压线）
 * @return 归一化位置 [-1.0f, +1.0f]，负值偏左、正值偏右，0 表示居中，
 *         返回值 99.0f 表示所有传感器均未检测到黑线
 */
float Tracking_CalcPosition(uint8_t mask);

#endif
