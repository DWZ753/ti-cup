#ifndef BOARD_H
#define BOARD_H

#include <stdint.h>

/**
 * @brief 板级初始化模板（公共驱动基线）
 * @note  赛题分支基于此文件扩展，添加各自外设的初始化
 */
void Board_Init(void);

/**
 * @brief 获取系统 1ms 滴答计数
 * @return 自启动以来的毫秒数
 */
uint32_t Board_GetTickMs(void);

#endif
