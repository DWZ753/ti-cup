# 云台控制子系统

## 概述

基于双环 PID 的二维云台角度伺服控制模块。负责驱动 ZDT 闭环步进电机（Roll + Yaw），通过 RS485 总线进行位置/速度闭环控制。

子模块：
- **gimbal** — 双环 PID 控制（位置外环 + 速度内环），GimbalAxis 结构体封装单轴，含综合测试函数 `GimbalTest_Run()`

## 文件

| 文件 | 说明 |
|------|------|
| `gimbal.h` | GimbalAxis 结构体、电机 ID、PID 默认参数、API 声明、测试函数声明 |
| `gimbal.c` | 双环 PID 控制 + 9 阶段综合测试 |

## 硬件

| 组件 | 型号 | 接口 | ID |
|------|------|------|----|
| Roll 轴 | ZDT X42S | RS485 @115200 | 1 |
| Yaw 轴 | ZDT X42S | RS485 @115200 | 2 |

两电机共享一路 RS485（PA0/PA1），通过 ID 寻址。

## 控制架构

```
target_angle → [外环 位置PID] → vel_cmd → [内环 速度PID] → Vel_Control()
                   ↑                            ↑
             read_angle()                read_velocity()
```

- 外环：角度误差走最短路径后进位置 PID
- 内环：速度误差进速度 PID → 电机方向+速度输出
- PID 为模块内部静态变量，调参通过 `GimbalAxis_TunePosPID()` / `GimbalAxis_TuneVelPID()`
- 角度缠绕：`feedback = target - error` 作为等价反馈

## API（gimbal.h）

### GimbalAxis 结构体

```c
typedef struct {
    uint8_t  motor_id;          // ZDT 电机地址
    bool     enabled;           // 使能状态
    float    current_angle;     // 当前角度 (°)
    float    current_vel;       // 当前转速 (RPM)
    float    target_angle;      // 目标角度 (°)
    float    pos_error;         // 位置误差 (°)
    float    vel_cmd;           // 速度指令 (RPM)
    float    vel_error;         // 速度误差 (RPM)
    float    motor_output;      // 最终输出
    uint16_t max_vel;           // 最大速度限制
    float    pos_tolerance;     // 到位容差 (°)
    uint16_t control_period_ms; // 控制周期
    uint8_t  _slot;             // 内部 PID 池槽位（勿动）
} GimbalAxis;
```

### 函数

| 函数 | 说明 |
|------|------|
| `GimbalAxis_Init(axis, motor_id)` | 注册到内部 PID 池，填充默认参数 |
| `GimbalAxis_TunePosPID(axis, kp, ki, kd)` | 实时调位置环 PID，内部清零积分 |
| `GimbalAxis_TuneVelPID(axis, kp, ki, kd)` | 实时调速度环 PID |
| `GimbalAxis_GetPosPID(axis, &kp, &ki, &kd)` | 读取当前位置环参数 |
| `GimbalAxis_GetVelPID(axis, &kp, &ki, &kd)` | 读取当前速度环参数 |
| `GimbalAxis_SetTarget(axis, angle)` | 设置目标角度，自动归一化到 [-180,180] |
| `GimbalAxis_Enable(axis, en)` | 使能/禁用电机 |
| `GimbalAxis_Update(axis)` | 执行一轮双环控制（读反馈 → PID → 输出） |
| `GimbalAxis_IsAtTarget(axis)` | 判断是否到位 |
| `GimbalAxis_Stop(axis)` | 急停 + 清零 PID |
| `GimbalAxis_Reset(axis)` | 仅清零 PID 历史状态 |

## PID 默认参数

| 参数 | 位置环 | 速度环 |
|------|--------|--------|
| Kp | 8.0 | 1.5 |
| Ki | 0.1 | 0.3 |
| Kd | 0.0 | 0.0 |
| 积分限幅 | 50.0 | 100.0 |
| 输出限幅 | 200.0 | 800.0 |

> 上赛道后根据实际负载调整。

## 在 empty.c 中的用法

```c
GimbalAxis roll, yaw;

GimbalAxis_Init(&roll, GIMBAL_ROLL_ID);
GimbalAxis_Init(&yaw,  GIMBAL_YAW_ID);

GimbalAxis_TunePosPID(&roll, 2.0f, 0.05f, 0.0f);
GimbalAxis_Enable(&roll, true);

GimbalAxis_SetTarget(&roll, 30.0f);

while (1) {
    GimbalAxis_Update(&roll);
    if (GimbalAxis_IsAtTarget(&roll)) { /* ... */ }
}
```

## PID Helper 联调

通过 [pid_helper](../pid_helper/pid_helper.md) 模块连接桌面调参工具，实时监控和修改 PID 参数。

## 依赖

- [PID 算法库](../pid/pid.h)
- [ZDT Motor 模块](../../modules/motor/zdt_motor.h)
- [UART 模块](../../bsp/uart/uart.h)（RS485 通信）
- [delay 模块](../../bsp/delay/delay.h)
