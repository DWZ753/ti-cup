#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/**
 * @brief 板级初始化（小车）
 * @note  初始化顺序：SysTick → 定时器 → 执行器 → 输入 → 通信 → 传感器
 */
void Board_Init(void);

/**
 * @brief 获取系统 1ms 滴答计数
 * @return 自启动以来的毫秒数
 */
uint32_t Board_GetTickMs(void);

#endif
