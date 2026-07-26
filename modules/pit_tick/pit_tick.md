# PIT Tick 定时器模块使用说明

## 概述

项目提供两个基于 PIT（Periodic Interval Timer）的周期性定时器模块，用于在中断上下文注册和执行回调任务。两个模块 API 完全相同，区别仅在于定时周期和目标用途。

| 模块 | 定时器 | 周期 | 频率 | 适用场景 |
|---|---|---|---|---|
| `pit_fast_tick` | TIMG12 | 1ms | 1kHz | 轻量快速任务（计数器、简单计时） |
| `pit_control_tick` | TIMG0 | 20ms | 50Hz | 控制算法（电机PID、按键扫描） |

## 核心机制

两个模块采用相同的回调槽位机制：各自维护一个最多 8 个回调的函数指针数组，ISR 触发时依次遍历调用。

```
PIT LOAD 中断 → ISR → for (i=0; i<count; i++) callback[i]();
```

## API 接口

两个模块接口一致，仅前缀不同（`PIT_Fast_Tick` / `PIT_Control_Tick`）。

### 1. Init()

初始化定时器中断，使能 NVIC。

```c
void PIT_Fast_Tick_Init(void);
void PIT_Control_Tick_Init(void);
```

必须在注册回调前调用，通常在 `SYSCFG_DL_init()` 之后。

### 2. RegisterCallback()

注册一个回调函数到该定时器的执行列表。

```c
bool PIT_Fast_Tick_RegisterCallback(PIT_Fast_Callback_t callback);
bool PIT_Control_Tick_RegisterCallback(PIT_Control_Callback_t callback);
```

- `callback`：`void (*)(void)` 类型的函数指针
- 返回 `true` 注册成功，`false` 表示槽位已满（>8 个）或指针为空
- 按注册顺序依次执行，先注册先调用

### 3. 回调类型

```c
typedef void (*PIT_Fast_Callback_t)(void);
typedef void (*PIT_Control_Callback_t)(void);
```

## 实际使用

当前项目中各模块自主注册到对应的 tick：

```c
// 1ms fast tick —— 轻量、高频
PIT_Fast_Tick_RegisterCallback(imu_tick_cb);       // IMU 时间基准计数
PIT_Fast_Tick_RegisterCallback(Buzzer_TickHandler); // 蜂鸣器计时

// 20ms control tick —— 控制算法
PIT_Control_Tick_RegisterCallback(Key_TickHandler);   // 按键状态扫描
PIT_Control_Tick_RegisterCallback(Motor_TickHandler); // 电机速度闭环
```

## 选择指南

| 任务特征 | 选择 |
|---|---|
| 纯计数、标志位翻转 | `pit_fast_tick` (1ms) |
| 需要精确毫秒级计时 | `pit_fast_tick` (1ms) |
| 电机PID、速度解算 | `pit_control_tick` (20ms) |
| 按键消抖、状态机轮询 | `pit_control_tick` (20ms) |
| 传感器数据读取 | `pit_control_tick` (20ms) |

原则：回调尽量简短（在中断上下文中执行），耗时任务放 control tick，简单计数放 fast tick。

## 注意事项

- 回调在 ISR 中同步执行，**不要放阻塞、延时或复杂浮点运算**
- 跨中断/主循环共享的变量用 `volatile` 修饰
- 最多注册 8 个回调，超出返回 `false`
- 两个 tick 的 ISR 由 SysConfig 生成，名称固定为 `PIT_FAST_TICK_INST_IRQHandler` 和 `PIT_CONTROL_TICK_INST_IRQHandler`，**不要手动调用**

## 添加自定义回调

```c
#include "pit_fast_tick.h"  // 或 pit_control_tick.h

static void My_TickHandler(void)
{
    // 周期执行的逻辑
}

void MyModule_Init(void)
{
    PIT_Fast_Tick_RegisterCallback(My_TickHandler);
}
```

## 相关文件

- `modules/pit_fast_tick.c/.h` — 1ms 快速 tick
- `modules/pit_control_tick.c/.h` — 20ms 控制 tick
- `Debug/ti_msp_dl_config.h` — SysConfig 生成的定时器硬件配置
- `empty.syscfg` — 定时器周期和时钟源配置入口
