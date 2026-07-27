#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/* ========== 基础设施 ========== */
#include "pit_fast_tick.h"
#include "pit_control_tick.h"
#include "delay.h"
#include "i2c.h"

/* ========== 外设模块 ========== */
#include "key.h"
#include "tm1637.h"
#include "buzzer.h"
#include "oled.h"
#include "servo.h"


/**
 * @brief 板级初始化（计分显示装置）
 * @note  初始化顺序：SysTick → 定时器 → 外设 → 通信
 */
void Board_Init(void);

/**
 * @brief 获取系统 1ms 滴答计数
 * @return 自启动以来的毫秒数
 */
uint32_t Board_GetTickMs(void);

#endif
