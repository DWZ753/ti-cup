# Delay 延时模块

## 概述

基于 CPU 时钟周期的毫秒级阻塞延时模块，通过消耗 CPU 周期实现精确延时。适用于 MSPM0G3507 平台。

## 文件

| 文件 | 说明 |
|------|------|
| `delay.h` | 模块头文件 |
| `delay.c` | 模块实现 |

## API

### `void delay_ms(unsigned long ms)`

毫秒级阻塞延时。

| 参数 | 说明 |
|------|------|
| `ms` | 延时时间，单位毫秒 |

**实现原理：** 调用 `delay_cycles(ms * (CPUCLK_FREQ / 1000UL))`，其中 `CPUCLK_FREQ` 为系统时钟频率（由 SysConfig 生成的 `ti_msp_dl_config.h` 提供）。

**注意：**
- 该函数为**阻塞式**延时，调用期间 CPU 不能执行其他任务。
- 适用于短延时（毫秒级），不宜用于长时间等待。
- 在中断服务函数中应避免使用过长的阻塞延时。

## 依赖

- `ti_msp_dl_config.h`（提供 `CPUCLK_FREQ` 和 `delay_cycles()` 宏）

## 使用示例

```c
#include "delay.h"

// 延时 500ms
delay_ms(500);
```
