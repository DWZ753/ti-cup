#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdint.h>
#include "uart.h"

/* ========== 帧参数 ========== */

/** 单帧最大载荷字节数 */
#define PROTOCOL_MAX_PAYLOAD  32

/* ========== 命令字 ========== */

/* ---- RPi → MCU ---- */
#define CMD_LINE_OFFSET     0x10  /* int16 dx (mm) */
#define CMD_TARGET_ANGLE    0x11  /* int16 angle (0.01°) */
#define CMD_SPEED_CMD       0x12  /* int16 vL, int16 vR (mm/s) */
#define CMD_EMERGENCY_STOP  0x1F  /* 无载荷 */

/* ---- MCU → RPi ---- */
#define CMD_IMU_DATA        0x20  /* int16 roll, pitch, yaw (0.01°) */
#define CMD_ENCODER_DATA    0x21  /* int16 left_rpm, right_rpm */
#define CMD_ULTRASONIC      0x22  /* int16 dist_mm */
#define CMD_STATUS          0x2F  /* uint8 state */
#define CMD_ERROR           0xFF  /* string msg */

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
