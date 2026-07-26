/**
 * @file    protocol.c
 * @brief   通信协议层：COBS 帧分隔 + XOR 校验 + 命令分发
 *
 * 帧格式（线上）：
 *   [COBS-encoded(cmd + payload + checksum)] [0x00]
 *
 * COBS 解码后：
 *   [cmd(1B)] [payload(0~N B)] [checksum(1B)]
 *
 * 校验 = cmd ^ payload[0] ^ ... ^ payload[N-1] ^ checksum == 0
 */

#include "protocol.h"
#include "cobs.h"
#include "uart.h"
#include <stdbool.h>
#include <string.h>

/* ========== 帧边界常量 ========== */

/** 帧尾分隔符 */
#define FRAME_DELIMITER  0x00

/** 内部缓冲区：裸帧 = cmd(1) + 载荷(N) + checksum(1) */
#define FRAME_RAW_MAX    (1 + PROTOCOL_MAX_PAYLOAD + 1)

/** 内部缓冲区：COBS 编码后 = 裸帧 + 开销 + 帧尾 */
#define FRAME_COBS_MAX   (FRAME_RAW_MAX + FRAME_RAW_MAX / 254 + 2)

/* ========== 静态状态 ========== */

static Protocol_RxCallback s_rx_cb;
static UART_Handle        *s_uart;

static uint8_t  s_rx_buf[FRAME_COBS_MAX];  /**< 接收累积缓冲 */
static uint8_t  s_rx_pos;                  /**< 缓冲中有效字节数 */

/* ========== 内部函数 ========== */

/**
 * @brief 计算并验证 XOR 校验
 * @param data cmd + payload + checksum 的连续内存
 * @param len  总长度（含 checksum）
 * @return true = 校验通过
 */
static bool verify_checksum(const uint8_t *data, uint8_t len)
{
	uint8_t xor_sum = 0;
	uint8_t i;

	for (i = 0; i < len; i++)
	{
		xor_sum ^= data[i];
	}

	return xor_sum == 0;
}

/**
 * @brief 计算单字节 XOR 校验和
 * @param data 数据指针
 * @param len  数据长度
 * @return 校验字节
 */
static uint8_t calc_checksum(const uint8_t *data, uint8_t len)
{
	uint8_t xor_sum = 0;
	uint8_t i;

	for (i = 0; i < len; i++)
	{
		xor_sum ^= data[i];
	}

	return xor_sum;
}

/**
 * @brief 处理一个完整的接收帧（已去除 0x00 帧尾）
 */
static void process_frame(void)
{
	uint8_t raw[FRAME_RAW_MAX];
	uint8_t decoded_len;
	uint8_t cmd;
	uint8_t payload_len;

	/* COBS 解码 */
	decoded_len = COBS_Decode(s_rx_buf, s_rx_pos, raw);
	if (decoded_len < 3)  /* 至少 cmd(1) + checksum(1) + COBS 尾零 */
	{
		return;
	}

	/* COBS 解码末尾有一个尾零，去掉 */
	decoded_len--;

	/* 校验 */
	if (!verify_checksum(raw, decoded_len))
	{
		return;
	}

	/* 提取 cmd + payload */
	cmd = raw[0];
	payload_len = decoded_len - 2;  /* 去掉 cmd + checksum */

	s_rx_cb(cmd, &raw[1], payload_len);
}

/* ========== 公共 API ========== */

void Protocol_Init(Protocol_RxCallback rx_cb, UART_Handle *uart)
{
	s_rx_cb  = rx_cb;
	s_uart   = uart;
	s_rx_pos = 0;

	UART_StartReceiveRaw(uart);
}

void Protocol_Update(void)
{
	uint16_t avail;
	uint8_t  byte;

	avail = UART_RawRxAvailable(s_uart);

	while (avail > 0)
	{
		byte = UART_ReadRawByte(s_uart);
		avail--;

		if (byte == FRAME_DELIMITER)
		{
			/* 帧结束：处理已累积的字节（忽略空帧） */
			if (s_rx_pos > 0)
			{
				process_frame();
				s_rx_pos = 0;
			}
		}
		else
		{
			/* 缓冲区溢出保护：丢弃整帧 */
			if (s_rx_pos >= FRAME_COBS_MAX)
			{
				s_rx_pos = 0;
			}
			s_rx_buf[s_rx_pos] = byte;
			s_rx_pos++;
		}
	}
}

void Protocol_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
	uint8_t raw[FRAME_RAW_MAX];
	uint8_t raw_len;
	uint8_t encoded[FRAME_COBS_MAX];
	uint8_t encoded_len;

	/* 构建裸帧: cmd + payload + checksum */
	raw[0] = cmd;
	raw_len = 1;

	if (len > 0 && payload != NULL)
	{
		(void)memcpy(&raw[raw_len], payload, len);
		raw_len += len;
	}

	raw[raw_len] = calc_checksum(raw, raw_len);
	raw_len++;

	/* COBS 编码 + 帧尾 */
	encoded_len = COBS_Encode(raw, raw_len, encoded);
	encoded[encoded_len] = FRAME_DELIMITER;

	UART_SendData(s_uart, encoded, encoded_len + 1);
}
