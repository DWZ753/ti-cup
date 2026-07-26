# main.c — 主程序

## 概述

`main.c` 是整个项目的入口和**比赛任务主控循环**。当前版本实现了完整的双模式行驶控制：

- **弧线段（SEG_ARC）**：灰度传感器循迹 + PID 转向 + 差速驱动
- **直线段（SEG_STRAIGHT）**：IMU 角度保持 + 阿克曼转向，无黑线标记行驶

任务流程由 [State Machine](application/state_machine/state_machine.md) 管理，按键触发后自动按路径序列执行。

## 初始化顺序

```c
int main(void) {
    SYSCFG_DL_init();       // 1. 固定：SysConfig 生成的时钟和外设初始化
    Board_Init();           // 2. 基本固定：所有模块初始化
    StateMachine_Init();    // 3. 状态机初始化（按键回调注册）

    // 初始化循迹 PID（弧线段使用）
    PID_Controller tracking_pid;
    PID_Init(&tracking_pid, TRACKING_KP, TRACKING_KI, TRACKING_KD,
             TRACKING_INT_LIMIT, TRACKING_OUT_LIMIT);
    PID_SetTarget(&tracking_pid, 0.0f);

    // 初始化角度 PID（直线段使用）
    Angle_Init();

    // 进入主循环
}
```

## 控制周期

| 任务 | 周期 | 说明 |
|------|------|------|
| `IMU_Update()` | 10ms | 姿态解算（Mahony 互补滤波） |
| 循迹 PID 计算 | 10ms | 灰度→位置→PID→slew rate→舵机 |
| 角度 PID 计算 | 80ms | yaw→缠绕→死区→PID→slew rate→舵机 |
| UART 遥测 | 100ms | CSV 格式调试数据输出 |

## 双模式控制架构

### 弧线段：循迹 + 差速

```
灰度传感器(mask) → Tracking_CalcPosition() → PID_Compute(-position)
    → slew rate 限幅 → Servo_SetValue()
    → 差速：left = SPEED_ARC + diff, right = SPEED_ARC - diff
```

- 舵机偏转角越大 → 两轮速度差越大（`ARC_DIFF_GAIN = 7.3`）
- 丢线时舵机**缓慢回中**（而非保持上次角度），防止跑飞
- PID 输入取 `-position`：position 正值（偏右）→ PID 输出负值 → 舵机左转修正

### 直线段：角度保持 + 阿克曼

```
进入时：Angle_SetTargetRelative(delta_deg) → 锁定 yaw + delta_deg
每 80ms：Angle_Compute() → 读 yaw → wrap_180(error) → 死区 → PID → slew rate → 舵机
```

- 进入直线段时立即以**当前 yaw + 路径指定的相对偏转角**为目标
- yaw 的绝对漂移不影响相对偏转精度
- 死区内（±2°）舵机归零，清零积分
- 直线段固定速度 `SPEED_STRAIGHT = 450 mm/s`（无差速）

## 段切换与终点检测

### 弧线段 → 下一段
灰度传感器**连续 5 次全白（丢线）** → `StateMachine_SegmentDone()` 推进

### 直线段 → 下一段
- 步骤 1：先等待**离开起点黑线**（`StateMachine_NeedLeaveFirst()` → 传感器全白 → `StateMachine_LeftLine()`）
- 步骤 2：灰度传感器**重新压线** → `StateMachine_SegmentDone()` 推进

### 终点停车
`StateMachine_IsFinished()` 返回 `true` → `Motor_Brake()` 刹车

## 可调参数

### 循迹 PID（弧线段）

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `TRACKING_KP` | 100.0 | position∈[-1,1]，KP=100 时边缘压线输出约 ±100 |
| `TRACKING_KI` | 0.5 | 消除稳态偏差 |
| `TRACKING_KD` | 0.0 | 用 slew rate 替代 D 项 |
| `TRACKING_INT_LIMIT` | 20.0 | 积分上限 |
| `TRACKING_OUT_LIMIT` | 100.0 | 输出上限（舵量满偏） |
| `TRACKING_SLEW_MAX` | 4 | 每次调用舵机最大变化量 |

### 速度与差速

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `SPEED_ARC` | 1000.0 | 弧线段固定速度（mm/s） |
| `SPEED_STRAIGHT` | 450.0 | 直线段固定速度（mm/s） |
| `ARC_DIFF_GAIN` | 7.3 | 差速增益：servo × gain = 两轮速度差 |

### 丢线防抖

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `LINE_LOST_DEBOUNCE` | 5 | 连续丢线次数阈值（×10ms = 50ms） |

### 角度 PID（直线段）

见 [Angle 角度保持模块](application/angle/angle.md)。

## UART 遥测格式

每 100ms 输出一行 CSV：

```
seg, yaw, angle_target, angle_pid_out, angle_servo, on_line, lost_debounce, mask, track_pos, track_pid_out, track_servo
```

| 列 | 说明 | 单位 |
|----|------|------|
| seg | 当前段类型（0=ARC, 1=STRAIGHT, 2=STOP） | 枚举 |
| yaw | 当前偏航角 | ° |
| angle_target | 角度 PID 目标值 | ° |
| angle_pid_out | 角度 PID 输出 | — |
| angle_servo | 角度环舵机值 | — |
| on_line | 是否压线（1=压线, 0=丢线） | bool |
| lost_debounce | 丢线防抖计数 | — |
| mask | 灰度传感器原始读数（8bit） | hex |
| track_pos | 循迹位置（-1~+1, 99=丢线） | — |
| track_pid_out | 循迹 PID 输出 | — |
| track_servo | 循迹环舵机值 | — |

## 主循环调度伪代码

```c
while (1) {
    now = Board_GetTickMs();

    // ---- 10ms: IMU 更新 ----
    if (now - last_imu >= 10) { IMU_Update(); }

    // ---- 按键检测 & 任务启动 ----
    if (state changed) { StateMachine_StartTask(st); }

    // ---- 当前段控制 ----
    if (seg == SEG_ARC) {
        // 循迹模式：PID + slew rate + 差速
        if (now - last_tracking >= 10) {
            position = Tracking_CalcPosition(mask);
            steering = PID_Compute(&tracking_pid, -position);
            steering = slew_limit(steering);
            Servo_SetValue(steering);
            Motor_SetSpeedLR(SPEED_ARC + diff, SPEED_ARC - diff);
        }
    } else if (seg == SEG_STRAIGHT) {
        // 角度保持模式
        if (!angle_enabled) { Angle_SetTargetRelative(delta_deg); }
        Motor_SetSpeed(SPEED_STRAIGHT);
    } else {
        Motor_Brake();
    }

    // ---- 80ms: 角度 PID ----
    if (now - last_angle_pid >= 80) { Angle_Compute(); }

    // ---- 段切换检测 ----
    if (seg == SEG_ARC && 连续丢线) { StateMachine_SegmentDone(); }
    if (seg == SEG_STRAIGHT && 重新压线) { StateMachine_SegmentDone(); }

    // ---- 终点停车 ----
    if (StateMachine_IsFinished()) { Motor_Brake(); }

    // ---- 100ms: 遥测 ----
    // 输出 CSV 格式调试数据
}
```

## 注意事项

- 主循环使用**忙等轮询**（busy-wait polling），无 RTOS
- 角度 PID 调用周期（80ms）远慢于 IMU 更新（10ms），这是刻意设计：阿克曼转向是积分型被控对象，高频控制反而容易震荡
- 循迹 PID 的 `position` 取反后输入（`-position`），因为 position 正值（偏右）需要负舵机值（左转修正）
- 丢线时舵机缓慢回中到 0，而非保持上次值，防止卡在极限角度跑飞
- `uint32_t` 滴答计数连续运行约 49.7 天会回绕，但 `now - last >= N` 的无符号比较对回绕是安全的
- 上赛道后需要根据实际抓地力调整 `SPEED_ARC`、`SPEED_STRAIGHT` 和 `ARC_DIFF_GAIN`
