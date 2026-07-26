# Motor 电机模块

## 概述

双电机控制模块，封装 TB6612 驱动 + 编码器测速，提供**统一的线速度接口**。支持：
- 以 mm/s 为单位设定目标速度（正值前进、负值后退）
- 编码器实时测速（RPM + 线速度）
- **窗口累积 + EMA 低通滤波**两阶段测速，消除编码器量化跳动
- 自动死区处理（低于最小占空比时自动制动）
- 通过 PIT Control Tick（20ms）周期性更新速度

## 文件

| 文件 | 说明 |
|------|------|
| `motor.h` | 模块头文件，参数宏与 API 声明 |
| `motor.c` | 模块实现 |

## 硬件参数

### 编码器

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR_ENCODER_PPR` | 11 | 编码器线数（每转脉冲数） |
| `MOTOR_ENCODER_GEAR_RATIO` | 30 | 减速比 |
| `MOTOR_ENCODER_PULSES_PER_OUTPUT_REV` | 330 | 输出轴每转脉冲数 = 11 × 30 |

编码器 A 相触发 GPIO 中断，B 相用于判断方向：
- A 相上升沿时，B 相为低 → 正向计数
- A 相上升沿时，B 相为高 → 反向计数

### 轮子

| 参数 | 值 | 说明 |
|------|-----|------|
| `WHEEL_RADIUS_MM` | 68.0 mm | 轮子半径 |
| `WHEEL_CIRCUMFERENCE_MM` | ~427.3 mm | 轮子周长 = 2πr |

### 速度参数

| 参数 | 值 | 说明 |
|------|-----|------|
| `MOTOR_MAX_RPM` | 300 RPM | 电机实测最高转速 |
| `MOTOR_MAX_SPEED_MM_S` | ~2136 mm/s | 最大线速度 = MaxRPM × 周长 / 60 |
| `MOTOR_MIN_DUTY` | 100 | 最低启动 PWM 占空比（低于此值电机不转） |
| `MOTOR_MAX_PWM_DUTY` | PWM_PERIOD_COUNT - 1 | 最大 PWM 占空比 |

### 测速滤波参数

两阶段滤波由 `Motor_TickHandler` 内部实现，参数在 `motor.h` 顶部定义：

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `MOTOR_SPEED_WINDOW_TICKS` | 3 | 测速窗口（Tick 数）。3 → 60ms，增大可提高精度但增加延迟 |
| `MOTOR_SPEED_EMA_GAIN` | 0.25f | EMA 滤波增益。1.0 = 无滤波，0.25 = 中度平滑，0.10 = 重度平滑 |

**阶段一（窗口累积）**：每个 20ms tick 读取编码器脉冲差值并累加，窗口期满后计算原始 RPM。
**阶段二（EMA 低通滤波）**：`new = old + (raw - old) × GAIN`，输出连续平滑的速度值。

滤波后的速度通过 `Motor_GetFilteredSpeedX()` / `Motor_GetFilteredRPMX()` 获取，适合直接喂给 PID 控制器。

## API

### 初始化与控制

#### `void Motor_Init(void)`

电机初始化。停止电机、清零编码器、使能 GPIO 中断、注册 PIT Control Tick 回调（20ms 速度更新）。

> 此函数由 `Board_Init()` 自动调用。

#### `void Motor_SetSpeed(float speed_mm_s)`

设置两个电机的目标线速度（统一接口）。

| 参数 | 说明 |
|------|------|
| `speed_mm_s` | 目标线速度 (mm/s)，正值 = 前进，负值 = 后退 |
| | 范围：`[-MOTOR_MAX_SPEED_MM_S, +MOTOR_MAX_SPEED_MM_S]` |

**内部处理：**
1. 将速度绝对值转换为 PWM 占空比：`duty = speed / MaxSpeed × MaxDuty`
2. 若 `duty < MOTOR_MIN_DUTY`：执行制动（死区保护）
3. 根据速度符号决定前进/后退方向

#### `void Motor_Brake(void)`

两个电机同时制动（快速停止）。

#### `void Motor_Stop(void)`

两个电机同时滑行停止（惯性停下）。

### 速度获取

| 函数 | 返回值 | 说明 |
|------|--------|------|
| `Motor_GetEncoder1RPM()` | `float` | 电机 1 原始转速 (RPM)，每窗口更新一次 |
| `Motor_GetEncoder2RPM()` | `float` | 电机 2 原始转速 (RPM) |
| `Motor_GetEncoder1Speed()` | `float` | 电机 1 原始线速度 (mm/s) |
| `Motor_GetEncoder2Speed()` | `float` | 电机 2 原始线速度 (mm/s) |
| `Motor_GetFilteredRPM1()` | `float` | 电机 1 滤波后转速 (RPM)，EMA 平滑 |
| `Motor_GetFilteredRPM2()` | `float` | 电机 2 滤波后转速 (RPM) |
| `Motor_GetFilteredSpeed1()` | `float` | 电机 1 滤波后线速度 (mm/s)，**推荐 PID 使用** |
| `Motor_GetFilteredSpeed2()` | `float` | 电机 2 滤波后线速度 (mm/s) |
| `Motor_GetEncoder1Pulse()` | `int32_t` | 电机 1 编码器脉冲累计值 |
| `Motor_GetEncoder2Pulse()` | `int32_t` | 电机 2 编码器脉冲累计值 |

### 其他

#### `void Motor_TickHandler(void)`

编码器转速更新函数（由 PIT Control Tick 中断调用，20ms 周期），实现两阶段测速：

**阶段一 — 窗口累积（每 `MOTOR_SPEED_WINDOW_TICKS` 次执行一次）**：
```
total_diff  = 窗口内各 tick 脉冲差值累加
raw_rpm     = total_diff × 100 / (11 × WINDOW_TICKS)
raw_speed   = raw_rpm × 轮子周长 / 60
```

**阶段二 — EMA 低通滤波**（窗口期满时执行）：
```
filtered = old_filtered + (raw - old_filtered) × MOTOR_SPEED_EMA_GAIN
```

首个窗口直接赋值跳过过渡过程，后续窗口逐步收敛。

#### `void Motor_ResetEncoder(void)`

清零编码器脉冲计数、原始/滤波速度值、窗口累积器及 EMA 状态。`Motor_Init()` 内部调用，急停/重启时可手动调用。

## 依赖

- [TB6612 模块](../tb6612/tb6612.md)（底层电机驱动）
- [PIT Control Tick 模块](../pit_tick/pit_tick.md)（20ms 速度更新定时器）
- `ti_msp_dl_config.h`

## 使用示例

```c
#include "motor.h"

// 初始化（由 Board_Init 自动调用）
Motor_Init();

// 让小车以 500mm/s 前进
Motor_SetSpeed(500.0f);

// 获取当前速度
float speed1 = Motor_GetEncoder1Speed();
float speed2 = Motor_GetEncoder2Speed();

// 倒退
Motor_SetSpeed(-300.0f);

// 制动
Motor_Brake();

// 滑行停止
Motor_Stop();
```

## 速度环 PID 控制

电机模块提供的 `Motor_GetFilteredSpeedX()` 已经过 EMA 平滑，可直接作为 [PID 控制器](../../application/pid/pid.md) 的反馈输入：

```c
PID_Controller speed_pid;

// 初始化 PID
PID_Init(&speed_pid, 2.0, 1.2, 0.0, 500.0, MOTOR_MAX_SPEED_MM_S);
PID_SetTarget(&speed_pid, 1500.0);  // 目标 1500mm/s

// 在控制循环中（每 PID_DT_MS 毫秒）
float avg_speed = (Motor_GetFilteredSpeed1() + Motor_GetFilteredSpeed2()) * 0.5f;
float output = PID_Compute(&speed_pid, avg_speed);
Motor_SetSpeed(output);
```

## 架构说明

小车采用**阿克曼转向**，由舵机控制前轮转角，`Motor_SetSpeed()` 统一驱动后轮两路电机。无需差速控制。
