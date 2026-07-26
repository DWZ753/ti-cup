/**
 * @file    cobs.c
 * @brief   Consistent Overhead Byte Stuffing 编解码实现
 *
 * 编码规则：
 * - 每 254 字节块前插入一个"开销字节" = 块内下一个零的距离 + 1
 * - 数据中的 0x00 被替换为距离值，不再直接出现
 * - 0xFF = 连续 254 字节不含零，不插入尾零
 * - 解码后末尾会多一个 0x00（协议外保留语义），由上层裁剪
 */

#include "cobs.h"
#include <stdint.h>

/** 每块最大非零字节数（对应 0xFF 开销字节） */
#define COBS_BLOCK_MAX 254

uint8_t COBS_Encode(const uint8_t *in, uint8_t in_len, uint8_t *out)
{
	uint8_t code_idx  = 0;  /* 当前开销字节在 out 中的位置 */
	uint8_t code_val  = 1;  /* 当前开销字节的值 (= 已复制字节数 + 1) */
	uint8_t src;
	uint8_t write_pos  = 1;  /* 写位置（跳过第一个开销字节） */
	uint8_t i;

	if (in_len == 0)
	{
		out[0] = 0x01;
		return 1;
	}

	for (i = 0; i < in_len; i++)
	{
		src = in[i];

		if (src == 0x00)
		{
			/* 遇到零：写回开销字节，新开一块 */
			out[code_idx] = code_val;
			code_idx       = write_pos;
			code_val       = 1;
			write_pos++;
		}
		else
		{
			out[write_pos] = src;
			write_pos++;
			code_val++;

			/* 块已满（254 字节不含零），强制新开一块 */
			if (code_val == 0xFF)
			{
				out[code_idx] = code_val;
				code_idx       = write_pos;
				code_val       = 1;
				write_pos++;
			}
		}
	}

	/* 写回最后一个开销字节 */
	out[code_idx] = code_val;

	return write_pos;
}

uint8_t COBS_Decode(const uint8_t *in, uint8_t in_len, uint8_t *out)
{
	uint8_t code;
	uint8_t copy_count;
	uint8_t write_pos = 0;
	uint8_t read_pos  = 0;
	uint8_t j;

	if (in_len == 0) return 0;

	while (read_pos < in_len)
	{
		code = in[read_pos];
		read_pos++;

		/* 开销字节为 0 是非法帧（0x00 只允许作为帧尾） */
		if (code == 0x00) return 0;

		copy_count = code - 1;

		/* 检查读越界 */
		if (read_pos + copy_count > in_len) return 0;

		for (j = 0; j < copy_count; j++)
		{
			out[write_pos] = in[read_pos];
			write_pos++;
			read_pos++;
		}

		/* code < 0xFF 表示块尾有一个隐含的零 */
		if (code != 0xFF)
		{
			out[write_pos] = 0x00;
			write_pos++;
		}
	}

	return write_pos;
}
