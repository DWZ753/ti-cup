# OLED 显示模块

## 概述

SSD1306 OLED 显示屏驱动模块（128×64 像素），通过 I2C 接口通信。支持 ASCII 字符（8px/16px 字体）、汉字（16px）、数字、BMP 图片显示，以及清屏、反色、屏幕旋转等功能。

## 文件

| 文件 | 说明 |
|------|------|
| `oled.h` | 模块头文件，API 声明 |
| `oled.c` | 模块实现 |
| `oledfont.h` | 字库文件（ASCII 字模 + 汉字点阵） |

## 硬件说明

| 参数 | 说明 |
|------|------|
| 接口 | I2C（I2C0） |
| 从机地址 | `0x3C`（7 位地址） |
| 分辨率 | 128 × 64 像素 |
| 驱动芯片 | SSD1306 |

I2C 引脚由 SysConfig 配置，OLED 初始化时自动注册 I2C 实例。若检测到 SDA 被拉低（上电时序异常），自动调用 `I2C_SDAUnlock()` 恢复总线。

## 坐标系

```
(0,0) ────────── x (0~127) ──────────→
  │
  │   y (0~7)，每页 8 像素高
  │   共 8 页 × 8 = 64 像素
  ↓
```

- **x**：列地址，0 ~ 127
- **y**：页地址（行），0 ~ 7（每页 8 像素高）

## API

### 初始化与显示控制

#### `void OLED_Init(void)`

自包含初始化。内部完成：
1. I2C 实例注册（使用 SysConfig 生成的 I2C_OLED 配置）
2. SDA 总线状态检测与解锁
3. SSD1306 初始化命令序列

> 此函数由 `Board_Init()` 自动调用。

#### `void OLED_Clear(void)`

清屏（全部像素置 0 → 全黑）。

#### `void OLED_DisplayOn(void)` / `void OLED_DisplayOff(void)`

开启/关闭显示（关闭后进入休眠模式）。

#### `void OLED_ColorTurn(uint8_t mode)`

正常/反色显示切换。

| mode | 效果 |
|------|------|
| 0 | 正常显示（亮 = 1） |
| 1 | 反色显示（亮 = 0） |

#### `void OLED_DisplayTurn(uint8_t mode)`

屏幕旋转 180°。

| mode | 效果 |
|------|------|
| 0 | 正常方向 |
| 1 | 旋转 180° |

### 内容显示

#### `void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)`

在指定位置显示一个 ASCII 字符。

| 参数 | 说明 |
|------|------|
| `x` | 列地址（0 ~ 127） |
| `y` | 页地址（0 ~ 7） |
| `chr` | ASCII 字符 |
| `sizey` | 字体高度：8 或 16 |

- `sizey = 8`：6×8 像素字体
- `sizey = 16`：8×16 像素字体

#### `void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *str, uint8_t sizey)`

在指定位置显示字符串（自动换行到下一行，不会换页）。

| 参数 | 说明 |
|------|------|
| `str` | 以 `\0` 结尾的字符串 |

#### `void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)`

显示无符号整数。

| 参数 | 说明 |
|------|------|
| `num` | 要显示的数字 |
| `len` | 数字的总位数（不足时前导空格，超出时显示低位） |

#### `void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)`

显示汉字（从字库 `Hzk[]` 取点阵）。

| 参数 | 说明 |
|------|------|
| `no` | 汉字在 `Hzk` 字库数组中的索引 |
| `sizey` | 字体高度（目前仅支持 16） |

#### `void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])`

显示 BMP 图片（点阵数据）。

| 参数 | 说明 |
|------|------|
| `sizex` | 图片宽度（像素） |
| `sizey` | 图片高度（像素） |
| `BMP` | 图片点阵数据数组（模版格式） |

### 底层操作

#### `void OLED_WriteByte(uint8_t dat, uint8_t mode)`

向 SSD1306 写入一个字节。

| mode | 说明 |
|------|------|
| `OLED_CMD` (0) | 写入命令 |
| `OLED_DATA` (1) | 写入数据 |

#### `void OLED_SetPos(uint8_t x, uint8_t y)`

设置光标位置（列 + 页）。

## 依赖

- [I2C 模块](../../bsp/i2c/i2c.md)
- [Delay 模块](../../bsp/delay/delay.md)
- `ti_msp_dl_config.h`

## 使用示例

```c
#include "oled.h"

// 初始化（由 Board_Init 自动调用）
OLED_Init();

// 清屏
OLED_Clear();

// 显示 8px 高度的字符串
OLED_ShowString(0, 0, (uint8_t*)"Hello World!", 8);

// 显示 16px 高度的字符串
OLED_ShowString(0, 2, (uint8_t*)"TI Cup 2024", 16);

// 显示数字
OLED_ShowNum(0, 4, 12345, 5, 16);  // 显示 "12345"

// 显示汉字
OLED_ShowChinese(0, 6, 0, 16);  // 显示 Hzk[0] 对应的汉字

// 反色显示
OLED_ColorTurn(1);

// 画一个 BMP 图标
OLED_DrawBMP(100, 0, 28, 28, bmp_data);
```

## 添加自定义汉字

1. 使用字模提取软件（如 PCtoLCD2002）生成 16×16 点阵数据
2. 将点阵数据添加到 `oledfont.h` 的 `Hzk` 数组中
3. 调用 `OLED_ShowChinese(x, y, 索引, 16)` 显示
