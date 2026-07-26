#ifndef TM1637_H
#define TM1637_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief TM1637 四位数码管驱动模块
 *
 * 自包含初始化：依赖 SysConfig 生成 GPIO_TM1637 (CLK=PB3, DIO=PB7)，
 * 不依赖 BSP 抽象层，直接使用 DriverLib GPIO API。
 *
 * 协议类似 I2C：start → cmd → stop → start → data → stop → start → ctrl → stop
 * ACK 位跳过检测（不读回），与 TM1637 硬件兼容。
 */

/**
 * @brief 初始化 TM1637（调用前需先执行 SYSCFG_DL_init）
 * @note  输出引脚设为高电平空闲态，清屏，设置默认亮度 7
 */
void TM1637_Init(void);

/**
 * @brief 显示整数
 * @param num          -999 ~ 9999
 * @param leading_zero true=前导零填充，false=前导消隐
 */
void TM1637_DisplayNum(int16_t num, bool leading_zero);

/**
 * @brief 显示原始段码（底层接口）
 * @param segs  4 字节段码，segs[0]=最左位(DIG1)
 *              bit0=A, bit1=B, bit2=C, bit3=D, bit4=E, bit5=F, bit6=G, bit7=DP/冒号
 */
void TM1637_DisplaySegments(const uint8_t segs[4]);

/**
 * @brief 清屏（全灭）
 */
void TM1637_Clear(void);

/**
 * @brief 设置亮度
 * @param level 0（最暗/PWM 1/16）~ 7（最亮/PWM 14/16）
 */
void TM1637_SetBrightness(uint8_t level);

/**
 * @brief 显示时间格式 (MM:SS)
 * @param min      分钟 0~99
 * @param sec      秒 0~59
 * @param colon_on true=冒号点亮，false=熄灭
 */
void TM1637_DisplayTime(uint8_t min, uint8_t sec, bool colon_on);

/**
 * @brief 单数字转 7 段码
 * @param digit 0~9
 * @return 段码（不含小数点）
 */
uint8_t TM1637_EncodeDigit(uint8_t digit);

#endif
