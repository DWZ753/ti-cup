# Angle 角度保持模块

## 概述

基于 IMU 偏航角（yaw）反馈的 PID 角度保持控制器。用于小车在**无黑线标记**的直线段/弦线段行驶时，自动修正航向偏差，保持目标方向。

## 文件

| 文件 | 说明 |
|------|------|
| `angle.h` | 模块头文件，参数宏与 API 声明 |
| `angle.c` | 模块实现 |

## 算法流程

```
Angle_Compute() 调用周期: 80ms（ANGLE_PID_DT_MS）

1. IMU_GetEuler(&yaw)
2. error = wrap_180(yaw - target)       // 角度缠绕归一化到 [-180°, 180°]
3. 死区判断：|error| < 2° → 舵机归零，清零积分，推进误差历史
4. 否则：PID_Compute(current_value)     // current_value = target - error
5. Slew rate 限幅：|Δservo| ≤ 15
6. Servo_SetValue(servo)
```

## 目标设置方式

### 相对角度模式（主用）

```c
Angle_SetTargetRelative(float delta_deg);
```

进入直线段时调用**一次**。以**当前 yaw** 为基准，偏转 `delta_deg` 度后锁定为新的目标航向。

- `delta_deg = 0`：保持当前朝向直走
- `delta_deg > 0`：左转后直走
- `delta_deg < 0`：右转后直走

yaw 的绝对漂移不影响相对偏转精度——因为基准是调用瞬间的实时读数。

### 绝对角度模式（调试用）

```c
Angle_SetTarget(float heading_deg);
```

设置绝对目标航向角（°）。一般在调试时使用，上赛道不建议（yaw 可能存在累积漂移）。

## main.c 中的集成方式

```c
/* 检测到进入直线段时 */
if (seg == SEG_STRAIGHT) {
    if (!Angle_IsEnabled()) {
        Angle_Enable(true);
        Angle_SetTargetRelative(StateMachine_GetDeltaDeg());  // 调用一次！
    }
    Motor_SetSpeed(SPEED_STRAIGHT);  // 固定速度，无差速
}

/* 离开直线段时 */
else if (seg == SEG_ARC) {
    Angle_Enable(false);  // 关闭角度环，舵机由循迹模块接管
}
```

**关键点**：`Angle_SetTargetRelative()` 在进入直线段时只调用一次，不要在循环中反复调用——否则每次都会用新的 yaw 覆盖 target，导致目标漂移。

## 设计要点

### 无 EMA 滤波

Mahony 互补滤波器输出的 yaw 已经是姿态融合结果（陀螺仪+加速度计），不需要再叠加一层 EMA 低通滤波。额外滤波只会增加滞后。

### 角度缠绕归一化

yaw 在 [-180°, 180°] 范围内，但 target 可以是任意值（如 -90°、90°）。`wrap_180()` 确保误差始终取最短路径。例如 target=170°, yaw=-170° → error=-20°（而不是 340°）。

### 死区处理

死区 (±2°) 内不调用 `PID_Compute()`，直接输出舵机归零。同时：
- 清零积分项（防止死区内积分累积）
- 手动推进误差历史（为将来可能添加的 D 项保持连续性）

### Slew rate 限幅

防止舵机在两次计算之间大幅跳变。`ANGLE_SLEW_MAX=15`，80ms 周期下，-50 到 +50 全程变化需约 5 次迭代（约 0.5s），既保证了响应速度又防止了突变。

### 模式切换

- `Angle_Enable(true)` → 调用 `Angle_Reset()`，清零积分和误差历史，以当前位置为中心重新开始控制
- `Angle_Enable(false)` → 舵机归零，PID 复位。后续舵机由循迹模块接管

## API 完整列表

| 函数 | 说明 |
|------|------|
| `Angle_Init()` | 初始化 PID 控制器和内部状态 |
| `Angle_Enable(bool)` | 启用/关闭角度保持 |
| `Angle_IsEnabled()` | 查询是否启用 |
| `Angle_SetTarget(float)` | 设置绝对目标航向角（调试用） |
| `Angle_SetTargetRelative(float)` | 以当前 yaw 为基准相对偏转（比赛用） |
| `Angle_Compute()` | 执行一次角度 PID 计算（每 80ms 调用） |
| `Angle_Reset()` | 清零积分/误差历史，舵机归零 |

### 遥测 Getter

| 函数 | 说明 |
|------|------|
| `Angle_GetTarget()` | 获取当前目标航向角 |
| `Angle_GetPIDOutput()` | 获取 PID 原始输出值 |
| `Angle_GetServoValue()` | 获取当前舵机值（slew rate 限幅后） |

## 可调参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `ANGLE_KP` | 2.0 | 每度偏差→舵机输出量。调大=更激进修正 |
| `ANGLE_KI` | 0.02 | 积分，消除长期漂移导致的稳态偏差 |
| `ANGLE_KD` | 0.0 | 微分，先不加（阿克曼转向天然有阻尼） |
| `ANGLE_INTEGRAL_LIMIT` | 25.0 | 积分上限 |
| `ANGLE_OUTPUT_LIMIT` | 50.0 | 输出上限（舵量），不打满方向盘 |
| `ANGLE_DEADBAND_DEG` | 2.0 | 死区大小（°） |
| `ANGLE_SLEW_MAX` | 15 | 每次舵机最大变化量 |

## 调参指南

1. **纯 P**：Ki=0, Kd=0，从 2.0 开始。手动偏转小车，观察舵机是否反向修正且不过冲
2. **死区**：确认 ±2° 内舵机归零
3. **Slew rate**：确认舵机变化平滑无跳变
4. **加 I**：仅在观察到长期稳态偏差（如小车持续偏左/右）时才加，从 0.01 起步

> **调参优先级（按效果）**：放慢控制频率（80ms） > 纯 P 起步 > 死区 ±2° > slew rate 限幅 > 最后才加小 I

## 依赖

- [PID 控制器模块](../pid/pid.md)
- [IMU 模块](../../modules/imu/imu.md)
- [Servo 舵机模块](../../modules/servo/servo.md)
