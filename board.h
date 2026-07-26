#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>
#include "uart.h"

/**
 * @brief 板级初始化，按依赖顺序调用所有模块的 Init
 * @note  调用前需确保 SYSCFG_DL_init() 已执行
 */
void Board_Init(void);

/**
 * @brief 获取系统 1ms 滴答计数
 * @return 自启动以来的毫秒数
 */
uint32_t Board_GetTickMs(void);

/**
 * @brief 获取 UART 打印句柄
 * @return UART 句柄指针
 */
UART_Handle* Board_GetUART(void);

/**
 * @brief 获取树莓派通信 UART 句柄 (UART1)
 * @return UART 句柄指针
 */
UART_Handle* Board_GetUART_PI(void);

#endif
