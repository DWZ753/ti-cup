#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "uart.h"

/* ========== 帧参数 ========== */

/** 单帧最大载荷字节数 */
#define PROTOCOL_MAX_PAYLOAD  64

/* ========== 回调类型 ========== */

/**
 * @brief 收到完整帧后的回调（在 Protocol_Update 内同步调用）
 * @param cmd     命令字
 * @param payload 载荷指针（仅回调内有效）
 * @param len     载荷字节数
 */
typedef void (*Protocol_RxCallback)(uint8_t cmd,
                                    const uint8_t *payload,
                                    uint8_t len);

/* ========== API ========== */

/**
 * @brief 初始化协议层
 * @param rx_cb 收到帧时的回调
 * @param uart  UART 句柄（需已完成 Init）
 */
void Protocol_Init(Protocol_RxCallback rx_cb, UART_Handle *uart);

/**
 * @brief 协议层主循环：从 UART 读字节 → 拼帧 → 校验 → 回调
 * @note  在主循环中高频调用
 */
void Protocol_Update(void);

/**
 * @brief 发送一帧
 * @param cmd     命令字
 * @param payload 载荷指针
 * @param len     载荷字节数（≤ PROTOCOL_MAX_PAYLOAD）
 */
void Protocol_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);

#endif
