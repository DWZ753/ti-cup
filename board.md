# Board 板级初始化与系统管理

## 概述

`board.c` / `board.h` 是系统的**初始化入口**和**全局资源汇总点**。它负责按依赖顺序初始化各硬件模块，并提供系统滴答时钟和 UART 句柄等公共服务。

实际开发中，当你新增或移除模块时，`board.c` 和 `board.h` 是**需要同步修改的两个文件**。

## 文件

| 文件 | 说明 |
|------|------|
| `board.h` | 汇总各模块头文件 + 声明公共 API。新增模块时在这里加 `#include` |
| `board.c` | 初始化实现 + 全局资源。新增模块时在这里加 `_Init()` 调用 |

## 初始化流程

`Board_Init()` 在 `main.c` 中 `SYSCFG_DL_init()` 之后调用。按依赖关系分层初始化：

```
定时器 ─────────────────────────────────────────────
  PIT_Fast_Tick_Init()              // 1ms 滴答时钟
  PIT_Fast_Tick_RegisterCallback()  // 注册 IMU 滴答计数器
  PIT_Control_Tick_Init()           // 20ms 控制时钟

执行器 ─────────────────────────────────────────────
  Buzzer_Init()                     // 蜂鸣器
  TB6612_Init()                     // 电机驱动芯片
  Motor_Init()                      // 电机速度闭环
  Servo_Init()                      // 舵机

输入 ───────────────────────────────────────────────
  Key_Init()                        // 按键
  Grayscale_Init()                  // 灰度传感器

通信 ───────────────────────────────────────────────
  UART_Init(&uart_cfg)              // 串口（DMA+UART）

传感器 ─────────────────────────────────────────────
  IMU_Init()                        // BMI088 姿态传感器（SPI）

显示 ───────────────────────────────────────────────
  OLED_Init()                       // OLED 显示屏（I2C）
```

> **大原则**：定时器 → 执行器 → 输入 → 通信 → 传感器 → 显示。新增模块时按此原则插入合适位置。

## API

### `void Board_Init(void)`

板级初始化。调用前需确保 `SYSCFG_DL_init()` 已执行。

### `uint32_t Board_GetTickMs(void)`

获取系统 1ms 滴答计数，用于 `main.c` 中的非阻塞定时（`now - last >= period` 模式）。

### `UART_Handle* Board_GetUART(void)`

获取当前注册的 UART 句柄，用于 `UART_Printf()` 遥测输出。UART 通过 `UART_Config` 结构体配置（包含实例号、中断号、DMA 通道等）。

## 系统滴答时钟

1ms 计数器由 PIT Fast Tick（TIMG12）中断累加，`Board_GetTickMs()` 直接返回计数值：

```c
static volatile uint32_t imu_ticks;

static void imu_tick_cb(void) {
    imu_ticks++;
}
```

`imu_ticks` 是 `uint32_t`，连续运行约 49.7 天会回绕。`now - last >= N` 的无符号数比较对回绕是安全的。

## 新增模块时的修改清单

假设你写了一个新模块 `modules/xxx/`，需要：

1. **`board.h`**：添加 `#include "xxx.h"`
2. **`board.c`**：在 `Board_Init()` 中适当位置调用 `Xxx_Init()`
3. 如果新模块需要主循环中轮询，去 `main.c` 加逻辑

如果新模块依赖某个已有模块（比如新传感器用 SPI），确认依赖项在它**之前**初始化。

## 注意事项

- 初始化顺序有依赖关系：**定时器必须先于使用其回调的模块**，**通信层（SPI/I2C/UART）必须先于使用该总线的传感器**
- `board.h` 是几乎所有 `.c` 文件都会 include 的，新增的内容尽量精简
- `Board_GetUART()` 返回的是 `UART_Handle*`，如果添加了更多 UART 实例，按同样模式提供对应的 getter
