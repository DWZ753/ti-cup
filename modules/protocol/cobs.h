#ifndef COBS_H
#define COBS_H

#include <stdint.h>

/**
 * @brief COBS 编码
 * @param in     输入数据
 * @param in_len 输入字节数
 * @param out    输出缓冲区（需 >= in_len + in_len/254 + 2 字节）
 * @return 编码后字节数（不含帧尾 0x00）
 */
uint8_t COBS_Encode(const uint8_t *in, uint8_t in_len, uint8_t *out);

/**
 * @brief COBS 解码
 * @param in     编码数据（不含帧尾 0x00）
 * @param in_len 编码字节数
 * @param out    输出缓冲区（需 >= in_len 字节）
 * @return 解码后字节数（含 COBS 尾零），0 = 格式错误
 */
uint8_t COBS_Decode(const uint8_t *in, uint8_t in_len, uint8_t *out);

#endif
