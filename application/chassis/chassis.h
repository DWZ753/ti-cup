/**
 * @file    chassis.h
 * @brief   底盘控制子系统 — 只暴露 Init / Task 两个入口
 *
 * Chassis_Init()  在 main() 中调用一次，初始化 PID、角度环、状态机
 * Chassis_Task()  在主循环中反复调用，执行全部控制逻辑和遥测
 *
 * 内部包含:
 *   - arc segment:  灰度循迹 → 舵机差速控制
 *   - straight seg: 角度保持 → 阿克曼固定速度 + 差速辅助
 *   - segment 切换: 丢线防抖 / 离-回线检测
 *   - 遥测: 每 100ms UART 输出 + OLED 显示
 */

#ifndef __CHASSIS_H__
#define __CHASSIS_H__

/**
 * @brief 底盘控制初始化
 * @note  必须在 SYSCFG_DL_init() + Board_Init() 之后调用
 */
void Chassis_Init(void);

/**
 * @brief 底盘控制主任务（每次主循环调用一次）
 */
void Chassis_Task(void);

#endif /* __CHASSIS_H__ */
