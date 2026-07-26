# Servo 舵机模块

## 概述

舵机驱动模块，通过 PWM 定时器输出控制舵机角度。提供统一的 `-100 ~ +100` 控制值接口，内部自动映射到 PWM 脉宽。

## 文件

| 文件 | 说明 |
|------|------|
| `servo.h` | 模块头文件，硬件宏定义与 API 声明 |
| `servo.c` | 模块实现 |

## 硬件说明

使用一个定时器 PWM 通道输出舵机控制信号。在 SysConfig 中配置 PWM 输出后，通过 `servo.h` 中的硬件映射宏关联。

## 宏定义

### 硬件映射宏（端口更改时修改）

```c
#define SERVO_PWM_INST              PWM_SERVO_INST           // PWM 定时器实例
#define SERVO_PWM_CHANNEL           GPIO_PWM_SERVO_C0_IDX    // 捕获比较通道索引
#define SERVO_PWM_CLK_FREQ          PWM_SERVO_INST_CLK_FREQ  // PWM 时钟频率 (Hz)
```

### 舵机参数宏（更换舵机型号时修改）

```c
#define SERVO_PULSE_MIN_US          1000    // -100 对应脉宽 (μs)
#define SERVO_PULSE_MAX_US          2000    // +100 对应脉宽 (μs)
#define SERVO_PULSE_CENTER_US       1500    //    0 对应脉宽 (μs)，中心位置
```

这三个参数由**实际标定**得到，对应控制值 `-100 / 0 / +100` 的物理脉宽。最大值和最小值应留有安全余量，避免撞到机械限位。

## 控制值映射

控制值 `[-100, +100]` 线性映射到脉宽 `[MIN_US, MAX_US]`：

```
控制值 = +100 → 脉宽 = MAX_US（最大角度）
控制值 =    0 → 脉宽 = CENTER_US（中心位置）
控制值 = -100 → 脉宽 = MIN_US（最小角度）
```

映射分两步完成：先计算微秒脉宽，再转换为定时器比较值。

**第一步：控制值 → 微秒脉宽**
```
pulse_us = CENTER_US + (value * (MAX_US - MIN_US) / 200)
```

**第二步：微秒脉宽 → 定时器比较值**
```
compare_value = pulse_us * (SERVO_PWM_CLK_FREQ / 1,000,000)
```

`SERVO_PWM_CLK_FREQ / 1,000,000` 是编译期常量，表示每微秒对应的定时器计数值。当 PWM 时钟频率改变时，SysConfig 自动更新 `SERVO_PWM_CLK_FREQ` 宏，公式无需手动修改。

## API

### `void Servo_Init(void)`

舵机初始化。启动 PWM 定时器计数器，将舵机复位到中心位置（控制值 = 0）。

> 此函数由 `Board_Init()` 自动调用。

### `int32_t Servo_LimitValue(int32_t value)`

将控制值限制在有效范围 `[-100, 100]` 内。

| 参数 | 说明 |
|------|------|
| `value` | 原始控制值 |

| 返回值 | 限制后的值 |

### `void Servo_SetValue(int32_t value)`

设置舵机角度。

| 参数 | 说明 |
|------|------|
| `value` | 控制值，范围 `[-100, 100]`，0 为中心位置 |

内部自动调用 `Servo_LimitValue()` 限幅。

## 依赖

- `ti_msp_dl_config.h`（提供 PWM 定时器外设驱动）

## 使用示例

```c
#include "servo.h"

// 初始化（由 Board_Init 自动调用）
Servo_Init();

// 舵机居中
Servo_SetValue(0);

// 舵机左转（假设正值为右、负值为左）
Servo_SetValue(-50);

// 舵机右转
Servo_SetValue(50);

// 舵机最大角度
Servo_SetValue(100);
```

## 更换舵机/移植注意事项

1. **标定舵机脉宽**：将舵机安装在车上，测试 `-100 / 0 / +100` 三个位置对应的比较值，反推微秒值后更新 `SERVO_PULSE_MIN_US`、`SERVO_PULSE_MAX_US`、`SERVO_PULSE_CENTER_US`
2. **修改 PWM 通道**：在 SysConfig 中重新配置 PWM 输出引脚后，更新 `SERVO_PWM_INST` 和 `SERVO_PWM_CHANNEL`
3. **PWM 时钟频率**：`SERVO_PWM_CLK_FREQ` 由 SysConfig 自动生成，修改 clock divider / prescale 后无需手动更新。当前配置为 250Hz（周期 4ms），若更换为模拟舵机需在 SysConfig 中将 `timerCount` 调大至 50Hz（周期 20ms）
