#ifndef PID_HELPER_H
#define PID_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include "uart.h"
#include "gimbal.h"

/* ========== 协议常量（与 PID Helper src/shared/float-protocol.ts 对齐） ========== */

#define PH_CMD_PID              1
#define PH_CMD_MODE             2
#define PH_CMD_OUT              3
#define PH_CMD_SP               4
#define PH_CMD_APPMODE          5

#define PH_STATUS_FAIL          0
#define PH_STATUS_OK            1

#define PH_MODE_CLOSED          0
#define PH_MODE_OPEN            1

#define PH_APPMODE_SPEED        0
#define PH_APPMODE_POSITION     1

#define PH_FRAME_HEADER_0       0xAA
#define PH_FRAME_HEADER_1       0x55
#define PH_CMD_FRAME_SIZE       17
#define PH_CMD_FRAME_CHECKSUM_OFFSET  16
#define PH_SAMPLE_FRAME_SIZE    36
#define PH_ACK_FRAME_SIZE       20
#define PH_FLOAT_BYTE_LENGTH    4
#define PH_TELEMETRY_FIELD_COUNT 8

/** 帧尾: 00 00 80 7f (float +Inf 的位模式) */
#define PH_FRAME_TAIL_0         0x00
#define PH_FRAME_TAIL_1         0x00
#define PH_FRAME_TAIL_2         0x80
#define PH_FRAME_TAIL_3         0x7F

/* ========== 默认活跃轴选择 ========== */

#define PH_DEFAULT_AXIS_ROLL    0
#define PH_DEFAULT_AXIS_YAW     1

/**
 * @brief 修改此定义切换 PID Helper 调参目标轴:
 *        PH_DEFAULT_AXIS_ROLL (0) 或 PH_DEFAULT_AXIS_YAW (1)
 */
#define PH_DEFAULT_AXIS         PH_DEFAULT_AXIS_ROLL

/* ========== 回调类型 ========== */

typedef void (*PH_CommandCallback)(uint8_t cmd, uint8_t seq,
                                   float arg0, float arg1, float arg2);

/* ========== API ========== */

/**
 * @brief 初始化 PID 协议模块
 * @param uart  已初始化的 UART 句柄（需提前通过 UART_Init 注册并设置为 RAW 模式）
 * @param roll  云台 Roll 轴指针
 * @param yaw   云台 Yaw 轴指针
 */
void PH_Init(UART_Handle *uart, GimbalAxis *roll, GimbalAxis *yaw);

/**
 * @brief 注册命令回调（可选，用于自定义命令处理）
 */
void PH_SetCallback(PH_CommandCallback cb);

/**
 * @brief 发送一帧采样数据
 * @note  根据当前 APPMODE 自动选择位置环或速度环数据
 */
void PH_SendSample(void);

/**
 * @brief 发送 ACK 帧
 * @param cmd     命令 ID
 * @param status  状态码: PH_STATUS_OK 或 PH_STATUS_FAIL
 * @param value   返回值 (float)
 * @param seq     命令序号回显
 */
void PH_SendAck(uint8_t cmd, uint8_t status, float value, uint8_t seq);

/**
 * @brief 处理接收缓冲（在主循环中周期性调用）
 * @note  从 UART 原始缓冲区读取字节，解析命令帧并分发
 */
void PH_Process(void);

/**
 * @brief 设置活跃轴索引
 * @param axis_index 0=Roll, 1=Yaw
 */
void PH_SetActiveAxis(uint8_t axis_index);

#endif
