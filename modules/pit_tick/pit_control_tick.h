#ifndef PIT_CONTROL_TICK_H
#define PIT_CONTROL_TICK_H

#include <stdbool.h>

typedef void (*PIT_Control_Callback_t)(void);

/**
 * @brief 初始化 PIT 控制滴答定时器，使能其中断
 */
void PIT_Control_Tick_Init(void);

/**
 * @brief 注册 PIT 定时器回调函数
 * @param callback 回调函数指针（无参数无返回值）
 * @return true 注册成功，false 注册失败（任务已满或 callback 为空）
 * @note 最多可注册 8 个回调，PIT LOAD 中断触发时依次调用
 */
bool PIT_Control_Tick_RegisterCallback(PIT_Control_Callback_t callback);

#endif
