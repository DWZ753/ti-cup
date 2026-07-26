#ifndef __KEY_H__
#define __KEY_H__

#include <stdint.h>
#include "ti_msp_dl_config.h"

#define KEY_PORT                GPIO_KEYs_PORT
// 按键数量定义
#define KEY_NUM                 4
// 消抖确认次数（连续检测到相同状态的次数）
#define KEY_DEBOUNCE_COUNT      3

// 按键有效电平枚举
typedef enum {
    KEY_ACTIVE_LOW = 0,   // 低电平有效（配置上拉电阻）
    KEY_ACTIVE_HIGH       // 高电平有效（配置下拉电阻）
} KeyActiveLevel_t;

typedef enum {
    KEY_STATE_IDLE = 0,      // 空闲状态
    KEY_STATE_PRESSED,       // 按下状态（消抖中）
    KEY_STATE_RELEASED       // 释放状态（已触发）
} KeyState_t;

typedef struct {
    uint32_t pin;                    // 按键引脚
    KeyActiveLevel_t active_level;  // 有效电平配置
    KeyState_t state;               // 当前状态
    uint8_t debounce_cnt;           // 消抖计数器
    uint8_t flag;                   // 按键触发标志
} Key_t;

// 按键回调类型：参数 key_index 为触发的按键索引（0 - KEY_NUM-1）
typedef void (*Key_Callback_t)(uint8_t key_index);

// 外部按键数组声明
extern Key_t keys[KEY_NUM];

/**
 * @brief 注册按键触发回调
 * @param callback 回调函数指针，按键触发时被调用并传入按键索引
 */
void Key_RegisterCallback(Key_Callback_t callback);

/**
 * @brief 按键模块初始化
 * @note 遍历配置数组初始化所有按键状态，注册 PIT 定时器回调用于周期性扫描
 */
void Key_Init(void);

/**
 * @brief 按键扫描与状态机处理
 * @note 基于有限状态机实现消抖和触发检测，触发后置位 flag 供 Key_GetFlag() 读取
 */
void Key_Scan(void);

/**
 * @brief 获取按键触发标志并自动清除
 * @param key_index 按键索引（0 ~ KEY_NUM-1）
 * @return 非 0 表示按键已触发，0 表示未触发或索引无效
 */
uint8_t Key_GetFlag(uint8_t key_index);

#endif
