# Tracking 循迹算法模块

## 概述

基于 8 路灰度传感器数据的黑线位置解算算法。使用**加权平均法**将传感器的离散电平值转换为连续的归一化位置值，供转向 PID 控制使用。

## 文件

| 文件 | 说明 |
|------|------|
| `tracking.h` | 模块头文件，宏定义与 API 声明 |
| `tracking.c` | 模块实现 |

## 算法原理

### 加权平均法

对检测到黑线（低电平）的传感器，以其索引为权重计算加权平均位置：

```
1. 取反掩码：bits = ~mask   （使 1 = 黑线/压线）
2. 加权求和：sum_weight = Σ(i × is_black[i])
3. 计数：    count = Σ(is_black[i])
4. 平均位置：center = sum_weight / count
5. 归一化：  position = (center - SENSOR_CENTER) / SENSOR_CENTER
```

其中 `SENSOR_CENTER = (8 - 1) / 2 = 3.5`。

### 返回值含义

| 返回值 | 含义 |
|--------|------|
| **-1.0** | 黑线在最左侧 |
| **0.0** | 黑线在正中间 |
| **+1.0** | 黑线在最右侧 |
| **99.0** | **丢线**：所有传感器均未检测到黑线 |

## 宏定义

```c
#define SENSOR_COUNT    8                        // 传感器数量
#define SENSOR_CENTER   ((SENSOR_COUNT - 1) / 2.0f)  // 中心索引 = 3.5
```

## API

### `float Tracking_CalcPosition(uint8_t mask)`

根据灰度传感器原始读数计算黑线的归一化位置。

| 参数 | 说明 |
|------|------|
| `mask` | 8 位掩码，由 `Grayscale_ReadAll()` 获取 |
| | bit[i]：1 = 白色/未压线，0 = 黑色/压线 |

| 返回值 | 说明 |
|--------|------|
| `[-1.0, +1.0]` | 归一化位置，负值偏左、正值偏右 |
| `99.0` | 所有传感器均未检测到黑线（丢线） |

## 依赖

- [Grayscale 灰度传感器模块](../../modules/grayscale/grayscale.md)（提供 `mask` 输入）

## 实际使用（main.c 中的集成方式）

### 配合 PID + Slew Rate 转向控制

```c
// 初始化循迹 PID
PID_Controller tracking_pid;
PID_Init(&tracking_pid, 100.0, 0.5, 0.0, 20.0, 100.0);
PID_SetTarget(&tracking_pid, 0.0f);     // 目标：黑线居中 → position = 0
int32_t last_servo = 0;

// 每 10ms 调用
uint8_t mask = Grayscale_ReadAll();
float position = Tracking_CalcPosition(mask);

if (position != 99.0f) {
    // 注意：取 -position，因为 position 正值（偏右）需要负舵机值（左转修正）
    float steering = PID_Compute(&tracking_pid, -position);
    int32_t servo_out = (int32_t)steering;

    // Slew rate 限幅：每次最多变化 TRACKING_SLEW_MAX
    int32_t delta = servo_out - last_servo;
    if (delta > 4)       servo_out = last_servo + 4;
    else if (delta < -4) servo_out = last_servo - 4;

    last_servo = servo_out;
    Servo_SetValue(servo_out);
} else {
    // 丢线：舵机缓慢回中，防止卡在上次极限角度跑飞
    if (last_servo > 4)       last_servo -= 4;
    else if (last_servo < -4) last_servo += 4;
    else                      last_servo = 0;
    Servo_SetValue(last_servo);
}
```

### 配合差速驱动

```c
// 舵机偏转角越大 → 两轮速度差越大
float diff = servo_out * ARC_DIFF_GAIN;
float left_speed  = SPEED_ARC + diff;
float right_speed = SPEED_ARC - diff;
Motor_SetSpeedLR(left_speed, right_speed);
```

### 丢线防抖（段切换检测）

```c
// 弧线段连续丢线 N 次 → 推进到下一段
#define LINE_LOST_DEBOUNCE 5
uint8_t lost_debounce = 0;

if (!on_line) {
    if (++lost_debounce >= LINE_LOST_DEBOUNCE) {
        lost_debounce = 0;
        StateMachine_SegmentDone();
    }
} else {
    lost_debounce = 0;
}
```

## 注意事项

- `mask` 中 bit 顺序与传感器物理排列顺序必须一致（bit[0] = 最左侧传感器）
- 丢线时返回 **99.0**，调用者需自行处理。建议舵机缓慢回中而非保持上一帧位置，防止跑飞
- 加权平均法对噪声较敏感，必要时可在位置输出端加低通滤波。当前版本通过 PID 积分项和 slew rate 限幅间接抑制噪声
- 如果只有 1 个传感器检测到黑线，位置即为该传感器的归一化索引
- PID 输入取 `-position`（符号反转），因为 position 的方向与舵机修正方向相反
- 当前循迹 PID 参数较大（`KP=100`），因为 position 范围仅 [-1,1]，需要足够的输出驱动舵机
