#ifndef __BUZZER_H__
#define __BUZZER_H__

#include <stdint.h>

/**
 * @brief 启动蜂鸣器并设置鸣叫时长
 * @param time_ms 蜂鸣器鸣叫的持续时间（ms），由定时器回调自动控制停止
 * @note 调用前需先通过 Buzzer_Init() 完成初始化；若蜂鸣器已在运行，再次调用将重置鸣叫时间
 */
void Buzzer_Beep(uint32_t time_ms);

/**
 * @brief 立即停止蜂鸣器
 * @note 清除激活状态并复位 GPIO；如需重新启动，需再次调用 Buzzer_Beep()
 */
void Buzzer_Stop(void);

/**
 * @brief 初始化蜂鸣器模块
 * @note 系统启动时调用一次，注册定时器回调用于控制鸣叫时长
 */
void Buzzer_Init(void);

#endif
