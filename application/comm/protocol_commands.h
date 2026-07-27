/**
 * @file    protocol_commands.h
 * @brief   2026 智能物流搬运 — 小车↔计分显示装置 通信协议命令码
 *
 * 基于 modules/protocol/ 传输层（COBS + XOR 校验）
 * 所有多字节整数使用大端序
 */

#ifndef APP_PROTOCOL_COMMANDS_H
#define APP_PROTOCOL_COMMANDS_H

#include <stdint.h>

/* ================================================================
 * 命令码
 * ================================================================ */

/* ---- 计分显示装置 → 小车 ---- */
#define CMD_START         0x01  /**< 启动搬运任务（无载荷） */
#define CMD_STOP          0x02  /**< 正常停止（无载荷） */
#define CMD_EMERGENCY     0x03  /**< 急停（无载荷） */

/* ---- 小车 → 计分显示装置 ---- */
#define CMD_COUNT         0x10  /**< 上报已搬运数量  payload: uint8 */
#define CMD_WEIGHT        0x11  /**< 上报重量变化    payload: uint16 (g, 大端) */
#define CMD_STATUS        0x12  /**< 上报系统状态    payload: uint8 */
#define CMD_TIME          0x13  /**< 上报单球耗时    payload: uint16 (s, 大端) */
#define CMD_COMPLETE      0x14  /**< 全部搬运完成（无载荷） */

/* ================================================================
 * 系统状态
 * ================================================================ */

typedef enum {
	STATUS_IDLE    = 0,  /**< 待命中 */
	STATUS_RUNNING = 1,  /**< 运行中 */
	STATUS_FAULT   = 2,  /**< 故障 */
} system_status_e;

/* ================================================================
 * 辅助：大端 int16 读写
 * ================================================================ */

/** @brief 写入大端 int16_t 到缓冲区 */
static inline void put_i16(uint8_t *buf, int16_t val)
{
	buf[0] = (uint8_t)((val >> 8) & 0xFF);
	buf[1] = (uint8_t)(val & 0xFF);
}

/** @brief 从缓冲区读取大端 int16_t */
static inline int16_t get_i16(const uint8_t *buf)
{
	return (int16_t)(((uint16_t)buf[0] << 8) | buf[1]);
}

#endif /* APP_PROTOCOL_COMMANDS_H */
