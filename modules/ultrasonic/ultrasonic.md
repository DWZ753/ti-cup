# Ultrasonic 超声波测距模块

## 概述

HC-SR04 超声波测距模块，**非阻塞状态机**设计。主循环周期性调用 `Ultrasonic_Update()` 自动管理触发→等待回波→冷却的全流程，每约 65ms 完成一次测距并更新滑动窗口均值。

复用 PIT Control Tick（TIMG0, 1MHz）做 1µs 精度脉宽计时，不占用额外定时器。

## 文件

| 文件 | 说明 |
|------|------|
| `ultrasonic.h` | 模块头文件，API 声明 |
| `ultrasonic.c` | 模块实现 |

## 硬件说明

| 引脚 | MSPM0 | 方向 | 说明 |
|------|-------|------|------|
| TRIG | PA16 | 输出 | 触发脉冲（15µs 高电平） |
| ECHO | PA8 | 输入（双边沿中断） | 回波脉宽信号 |
| VCC | 5V | — | 模块供电 |
| GND | GND | — | 共地 |

## 工作原理

```
TRIG: ──┐┌──┐                          ┌──
         └┘  └──────────────────────────┘
          ↑15µs

ECHO: ───────────┐              ┌──────
                  └──────────────┘
                  ↑ 脉宽 = 往返时间 ↑
```

测距公式：`distance_mm = pulse_width_us / 5.8`

## 量程

| 参数 | 值 |
|------|-----|
| 量程 | 2 ~ 400 cm |
| 精度 | ±3 mm |
| 测量周期 | ~65ms/次 |
| 滑动窗口 | 5 次均值 |

## API

### `void Ultrasonic_Init(void)`

初始化内部状态（GPIO 和定时器由 SysConfig 配置）。由 `Board_Init()` 自动调用。

### `void Ultrasonic_Update(void)`

状态机推进，主循环中周期性调用（≥1kHz）。内部自动管理三个状态：

| 状态 | 动作 |
|------|------|
| IDLE | 发送 15µs 触发脉冲 → TRIGGERED |
| TRIGGERED | 等待 ECHO 中断捕获脉宽 / 100ms 超时 → COOLDOWN |
| COOLDOWN | 等待 65ms 最短间隔 → IDLE |

### `float Ultrasonic_GetDistance(void)`

获取最新测距值（mm）。返回滑动窗口 5 次均值，尚无有效数据返回 `-1.0f`。

### `bool Ultrasonic_IsNewData(void)`

查询是否有新测距结果就绪（自清除）。

### `void Ultrasonic_ECHO_ISR(void)`

ECHO 双边沿中断处理，由 GPIOA 中断分发。上升沿记录起始时刻，下降沿计算脉宽。

## 依赖

- `ti_msp_dl_config.h`
- [PIT Control Tick 模块](../pit_tick/pit_tick.md)（复用 TIMG0 1MHz 计时）

## 使用示例

```c
#include "ultrasonic.h"
#include "board.h"

// 初始化（由 Board_Init 自动调用）
Ultrasonic_Init();

while (1)
{
    Ultrasonic_Update();  // 每圈调用，驱动状态机

    if (Ultrasonic_IsNewData())
    {
        float dist = Ultrasonic_GetDistance();
        // dist >= 0: 有效测距值 (mm)
    }
}
```
