#ifndef PID_HELPER_H
#define PID_HELPER_H

#include <stdint.h>
#include "uart.h"

/**
 * @brief PID Helper 串口协议模块 — 与 PC 端 PID Helper 软件通信
 *
 * 协议: AA 55 + cmd + seq + 3×float32LE + checksum（下行）
 *       timestamp + 7×float32LE + 00 00 80 7f（上行采样）
 *       4×float32LE + 00 00 80 7f（上行 ACK）
 *
 * 使用方法:
 *   1. 实现 PH_SetPID / PH_GetPID / PH_SetTarget 回调
 *   2. 填充 PH_AxisConfig（回调 + 遥测指针）
 *   3. PH_Init(uart, &pos_axis, &vel_axis)
 *   4. 主循环调用 PH_Process()
 *   5. 需要上报时调用 PH_SendSample()
 */

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

#define PH_FRAME_TAIL_0         0x00
#define PH_FRAME_TAIL_1         0x00
#define PH_FRAME_TAIL_2         0x80
#define PH_FRAME_TAIL_3         0x7F

/* ========== 回调类型 ========== */

/** 设置 PID 参数（含副作用，如清零积分） */
typedef void (*PH_SetPID)(void *ctx, float kp, float ki, float kd);

/** 读取当前 PID 参数 */
typedef void (*PH_GetPID)(void *ctx, float *kp, float *ki, float *kd);

/** 设置目标值 */
typedef void (*PH_SetTarget)(void *ctx, float target);

/** 可选：收到命令时的通知回调 */
typedef void (*PH_CommandCallback)(uint8_t cmd, uint8_t seq,
                                   float arg0, float arg1, float arg2);

/* ========== 单轴配置 ========== */

typedef struct {
	PH_SetPID    set_pid;     /**< 写 PID（NULL = 不支持） */
	PH_GetPID    get_pid;     /**< 读 PID（NULL = 上报时 kp/ki/kd=0） */
	PH_SetTarget set_target;  /**< 写目标值（NULL = 不支持） */
	void        *ctx;         /**< 回调上下文（传给 set_pid/get_pid/set_target） */

	/* ---- 遥测指针（外部持续更新，PH 只读） ---- */
	float *actual;   /**< 当前值（位置/速度） */
	float *target;   /**< 当前目标 */
	float *error;    /**< 当前误差 */
	float *input;    /**< 当前输入指令 */
} PH_AxisConfig;

/* ========== API ========== */

/**
 * @brief 初始化 PID Helper 协议模块
 * @param uart  已初始化的 UART 句柄
 * @param pos   位置环轴配置（appmode=1 时使用）
 * @param vel   速度环轴配置（appmode=0 时使用）
 * @note  pos 和 vel 可指向同一轴的不同环，也可指向不同轴的同一类环
 */
void PH_Init(UART_Handle *uart,
             const PH_AxisConfig *pos,
             const PH_AxisConfig *vel);

/**
 * @brief 注册命令回调（可选）
 */
void PH_SetCallback(PH_CommandCallback cb);

/**
 * @brief 发送采样帧
 * @note  根据当前 APPMODE 自动选择位置环或速度环数据
 */
void PH_SendSample(void);

/**
 * @brief 发送 ACK 帧
 */
void PH_SendAck(uint8_t cmd, uint8_t status, float value, uint8_t seq);

/**
 * @brief 主循环处理（从 UART 原始缓冲读字节 → 解析命令帧 → 分发）
 */
void PH_Process(void);

/**
 * @brief 运行时切换控制的轴
 * @param pos  新的位置环轴配置
 * @param vel  新的速度环轴配置
 */
void PH_BindAxis(const PH_AxisConfig *pos, const PH_AxisConfig *vel);

#endif
