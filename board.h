/**
 * @file board.h
 * @brief 板级初始化声明
 */
#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "uart.h"

void     Board_Init(void);
uint32_t Board_GetTickMs(void);

/**
 * @brief 获取 Pi 通信 UART 句柄
 * @return UART 句柄指针（供 Protocol_Init 使用）
 */
UART_Handle* Board_GetPiUART(void);

#endif
