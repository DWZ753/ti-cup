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

/* ---- 树莓派 → MSPM0 小车 ---- */
#define CMD_GUIDE         0x20  /**< 锁定引导: speed(int8), diff(int8) */
#define CMD_STOP          0x21  /**< 立即停车（丢锁/误检/到达）（无载荷） */
#define CMD_MAGNET_ON     0x22  /**< 电磁铁吸合（无载荷） */
#define CMD_MAGNET_OFF    0x23  /**< 电磁铁释放（无载荷） */
#define CMD_GIMBAL        0x24  /**< 云台pitch: angle_deg(int16_t, 大端) */
#define CMD_RESUME_LINE   0x2F  /**< 回到灰度循迹模式（无载荷） */

/* ---- MSPM0 小车 → 树莓派 ---- */
#define CMD_TASK_START    0x31  /**< 任务开始: task_id(uint8) */
#define CMD_TASK_STOP     0x32  /**< 任务停止: time_ms(uint16, 大端) */
#define CMD_GIMBAL_OK     0x30  /**< 云台已到位（无载荷） */
#define CMD_FAULT         0x3F  /**< 故障: error_code(uint8_t) */

/* ================================================================
 * 系统状态
 * ================================================================ */

typedef enum {
	STATUS_IDLE    = 0,  /**< 待命中 */
	STATUS_RUNNING = 1,  /**< 运行中 */
	STATUS_FAULT   = 2,  /**< 故障 */
} system_status_e;

/* ================================================================
 * 故障码（CMD_FAULT 的 payload）
 * ================================================================ */

typedef enum {
	FAULT_TIMEOUT      = 1,  /**< GUIDED 模式 200ms 未收到指令，已自动停车 */
	FAULT_GIMBAL_OCP   = 2,  /**< 云台过流保护触发 */
	FAULT_GIMBAL_STALL = 3,  /**< 云台堵转 */
} fault_code_e;

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
