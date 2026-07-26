# Grayscale 灰度传感器模块

## 概述

8 路红外灰度传感器模块，用于循迹（黑线检测）。通过并行 GPIO 读取 8 路传感器的电平状态，返回一个 8 位掩码表示各传感器的黑白检测结果。

## 文件

| 文件 | 说明 |
|------|------|
| `grayscale.h` | 模块头文件，宏定义与 API 声明 |
| `grayscale.c` | 模块实现 |

## 硬件说明

8 路灰度传感器对应 8 个 GPIO 输入引脚，连接到 `GRAYSCALE_PORT`。每个传感器输出数字信号：

| 信号电平 | 含义 |
|----------|------|
| 高电平 (1) | 白色区域 / 未压线 |
| 低电平 (0) | 黑色区域 / 压线 |

## 宏定义

```c
#define GRAYSCALE_PORT  GPIO_GRAYSCALEs_PORT   // 传感器 GPIO 端口
#define GRAYSCALE_NUM   8                       // 传感器数量（8 路）
```

## API

### `void Grayscale_Init(void)`

灰度传感器模块初始化。当前实现为空（GPIO 初始化由 SysConfig 完成），保留此接口用于未来扩展（如校准）。

### `uint8_t Grayscale_ReadAll(void)`

读取所有 8 路灰度传感器的原始电平状态。

| 返回值 | 说明 |
|--------|------|
| 8 位掩码 | `bit[0]` 对应第 1 号传感器（索引 0），`bit[7]` 对应第 8 号传感器（索引 7） |

**位含义：**
- `1` = 高电平（白色 / 未压线）
- `0` = 低电平（黑色 / 压线）

## 依赖

- `ti_msp_dl_config.h`（提供 GPIO 引脚宏）

## 使用示例

```c
#include "grayscale.h"

// 读取传感器
uint8_t mask = Grayscale_ReadAll();

// 检查第 3 号传感器是否压线（索引 2）
if (!(mask & (1 << 2))) {
    // 传感器 3 检测到黑线
}

// 获取压线传感器数量
uint8_t count = 0;
for (int i = 0; i < 8; i++) {
    if (!(mask & (1 << i))) count++;
}
```

## 配合循迹算法使用

`Grayscale_ReadAll()` 的返回值可以直接传给 [循迹模块](../../application/tracking/tracking.md) 的 `Tracking_CalcPosition()` 计算黑线位置：

```c
uint8_t mask = Grayscale_ReadAll();
float position = Tracking_CalcPosition(mask);
// position: [-1.0, +1.0]，负值偏左、正值偏右，0 居中，99.0 表示丢线
```

## 注意事项

- GPIO 输入模式（上拉/下拉）需在 SysConfig 中正确配置
- 返回值为传感器原始电平，未做滤波或消抖处理
- 灰度传感器对环境光敏感，必要时考虑在 `Grayscale_Init()` 中添加自适应阈值校准
