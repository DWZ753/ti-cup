# Buzzer 蜂鸣器模块

## 概述

有源蜂鸣器驱动模块，通过 GPIO 控制蜂鸣器发声。支持设定鸣叫时长，由 PIT Fast Tick（1ms 定时器）自动计时并在时间到后停止，无需手动管理延时。

## 文件

| 文件 | 说明 |
|------|------|
| `buzzer.h` | 模块头文件 |
| `buzzer.c` | 模块实现 |

## 核心机制

### 定时器回调驱动

蜂鸣器不依赖阻塞延时，而是注册到 **PIT Fast Tick**（1ms 中断），在中断回调中递减剩余鸣叫时间。当倒计时归零时自动关闭蜂鸣器（GPIO 置高 → 停止发声）。

### 有源蜂鸣器控制

有源蜂鸣器只需 GPIO 高低电平即可控制：
- **发声**：GPIO 输出低电平（`DL_GPIO_clearPins`）
- **停止**：GPIO 输出高电平（`DL_GPIO_setPins`）

## API

### `void Buzzer_Init(void)`

初始化蜂鸣器模块，注册 1ms 定时器回调。

> 系统启动时调用一次即可。此函数由 `Board_Init()` 自动调用。

### `void Buzzer_Beep(uint32_t time_ms)`

启动蜂鸣器并设置鸣叫时长。

| 参数 | 说明 |
|------|------|
| `time_ms` | 蜂鸣器鸣叫的持续时间（毫秒） |

**行为：**
- 若蜂鸣器已在鸣叫中，再次调用将**重置**鸣叫时间
- 鸣叫期间，1ms 定时器自动递减计时器
- 计时归零后自动停止

### `void Buzzer_Stop(void)`

立即停止蜂鸣器。清除激活状态，复位 GPIO 输出。

## 依赖

- `ti_msp_dl_config.h`（提供蜂鸣器 GPIO 引脚宏）
- [PIT Fast Tick 模块](../pit_tick/pit_tick.md)（1ms 定时器回调）

## 使用示例

```c
#include "buzzer.h"

// 初始化（由 Board_Init 自动调用）
Buzzer_Init();

// 蜂鸣 500ms
Buzzer_Beep(500);

// 立即停止
Buzzer_Stop();

// 蜂鸣 100ms 作为按键反馈
void on_key_pressed(uint8_t key_index) {
    Buzzer_Beep(100);
}
```

## 注意事项

- 蜂鸣器的 GPIO 引脚必须通过 SysConfig 配置为输出模式
- 鸣叫期间的重入调用是安全的（会重置计时器）
- 回调函数在 PIT ISR 中执行，只涉及简单变量操作（无阻塞）
