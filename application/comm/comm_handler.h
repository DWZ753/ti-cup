#ifndef COMM_HANDLER_H
#define COMM_HANDLER_H

#include <stdint.h>
#include "uart.h"

/* ========== 应用层命令码 ========== */

/* RPi → MCU */
#define CMD_OLED_SHOW    0x13  /**< OLED 显示文本 */

/* MCU → RPi */
#define CMD_ULTRASONIC   0x22  /**< 超声波距离 */
#define CMD_STATUS       0x2F  /**< 状态/启动通知 */

/**
 * @brief 通信处理器：协议绑定 + 命令分发 + 传感上报
 *
 * 使用方法:
 *   1. Comm_Init(uart) — 绑定 UART，注册命令回调
 *   2. Comm_Update()   — 主循环每圈调用
 *
 * 当前支持:
 *   RPi → MCU:  0x13 OLED 显示
 *   MCU → RPi:  0x22 超声波距离
 *
 * 新增命令: 在 Comm_Init() 的回调 switch 中追加 case
 * 新增上报: 在 comm_handler.c 的 Comm_Update() 中追加定时上报逻辑
 */

typedef struct Comm_Config
{
	UART_Handle *uart;   /**< UART 句柄（需已完成 Init） */

	/* ---- 传感器上报开关（0 = 关闭） ---- */
	uint16_t imu_report_ms;         /**< IMU 上报间隔 ms，0 = 不启用 */
	uint16_t ultrasonic_report_ms;  /**< 超声波上报间隔 ms，0 = 不启用 */
} Comm_Config;

/**
 * @brief 初始化通信处理器
 * @param cfg 配置（UART + 上报开关）
 */
void Comm_Init(const Comm_Config *cfg);

/**
 * @brief 主循环更新：收帧 + 定时上报
 * @note  必须在主循环中高频调用
 */
void Comm_Update(void);

/**
 * @brief 发送一帧（供外部模块直接使用）
 * @param cmd     命令字
 * @param payload 载荷
 * @param len     载荷长度
 */
void Comm_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif
