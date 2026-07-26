#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief 初始化 HC-SR04 超声波模块
 * @note  依赖 SysConfig 完成 TRIG (PA16) 输出 + ECHO (PA8) 双边沿中断配置
 *        复用 PIT_CONTROL_TICK (TIMG0, 1MHz) 做脉宽计时，不占用额外定时器
 */
void Ultrasonic_Init(void);

/**
 * @brief 非阻塞状态机推进（主循环中周期性调用，约 ≥ 1kHz）
 * @note  内部自动管理：触发 → 等待回波 → 冷却 60ms → 下一轮
 *        单次测量耗时 ~65ms，结果自动累积至滑动窗口
 */
void Ultrasonic_Update(void);

/**
 * @brief 获取最新有效测距值（非阻塞）
 * @return 距离 mm，范围约 20~4000；尚无有效数据返回 -1.0f
 * @note  返回值为滑动窗口均值（最多 5 次），每 ~65ms 更新一次
 */
float Ultrasonic_GetDistance(void);

/**
 * @brief 查询是否有新的测距结果就绪（自清除）
 * @return true = 自上次查询后有新结果
 */
bool Ultrasonic_IsNewData(void);

/**
 * @brief ECHO 引脚中断处理，需从 GROUP1_IRQHandler 中调用
 * @note  由 motor.c 的 GROUP1_IRQHandler 统一分发
 */
void Ultrasonic_ECHO_ISR(void);

#endif
