#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_CMD  0
#define OLED_DATA 1

/**
 * @brief 初始化 SSD1306 OLED 显示屏
 * @param i2c 已注册的 I2C 句柄指针
 * @note 需先通过 I2C_Init 注册对应 I2C 实例，再传入句柄
 */
/**
 * @brief 自包含初始化 OLED（内部完成 I2C 注册 + SSD1306 初始化）
 * @note  使用 SysConfig 生成的 I2C_OLED 配置，无需外部传入句柄
 */
void OLED_Init(void);

/**
 * @brief 清屏（全黑）
 */
void OLED_Clear(void);

/**
 * @brief 开启 OLED 显示
 */
void OLED_DisplayOn(void);

/**
 * @brief 关闭 OLED 显示（进入休眠模式）
 */
void OLED_DisplayOff(void);

/**
 * @brief 正常/反色显示切换
 * @param mode 0 = 正常显示，1 = 反色显示
 */
void OLED_ColorTurn(uint8_t mode);

/**
 * @brief 屏幕旋转 180 度
 * @param mode 0 = 正常方向，1 = 旋转 180 度
 */
void OLED_DisplayTurn(uint8_t mode);

/**
 * @brief 向 SSD1306 写入一个字节（底层 I2C 操作）
 * @param dat  数据
 * @param mode OLED_CMD（命令）或 OLED_DATA（数据）
 */
void OLED_WriteByte(uint8_t dat, uint8_t mode);

/**
 * @brief 设置光标位置
 * @param x 列地址（0~127）
 * @param y 页地址（0~7）
 */
void OLED_SetPos(uint8_t x, uint8_t y);

/**
 * @brief 在指定位置显示一个 ASCII 字符
 * @param x     列地址（0~127）
 * @param y     行地址（0~7）
 * @param chr   要显示的字符
 * @param sizey 字体高度：8 或 16
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);

/**
 * @brief 在指定位置显示字符串
 * @param x     列地址（0-127）
 * @param y     行地址（0-7）
 * @param str   待显示的字符串（以 \0 结尾）
 * @param sizey 字体高度：8 或 16
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *str, uint8_t sizey);

/**
 * @brief 在指定位置显示数字
 * @param x     列地址（0~127）
 * @param y     行地址（0~7）
 * @param num   要显示的数字
 * @param len   数字位数
 * @param sizey 字体高度：8 或 16
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);

/**
 * @brief 在指定位置显示汉字
 * @param x     列地址（0~127）
 * @param y     行地址（0~7）
 * @param no    汉字在 Hzk 字库中的索引
 * @param sizey 字体高度（目前仅支持 16）
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey);

/**
 * @brief 显示 BMP 图片
 * @param x     起始列地址
 * @param y     起始行地址
 * @param sizex 图片宽度（像素）
 * @param sizey 图片高度（像素）
 * @param BMP   图片点阵数据
 */
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[]);

/**
 * @brief I2C SDA 解锁（从 SDA 被拉低的状态恢复）
 * @note 委托给 I2C 模块的 I2C_SDAUnlock，通过模拟 SCL 时钟脉冲尝试释放 SDA 线
 */
void OLED_SDAUnlock(void);

#endif
