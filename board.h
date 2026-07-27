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

/* ========== 通信 ========== */
#include "uart.h"

void Board_Init(void);
uint32_t Board_GetTickMs(void);

/**
 * @brief 获取蓝牙通信 UART 句柄（UART1, PB6/TX, PB7/RX）
 */
UART_Handle* Board_GetUART_BT(void);

#endif
