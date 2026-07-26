# State Machine 状态机模块

## 概述

比赛任务状态机，负责两件事：
1. **按键触发任务切换**（KEY1~KEY4 → TASK1~TASK4）
2. **路径段序列管理**（自动按预设路径推进弧线段/直线段，TASK4 自动跑 4 圈）

## 文件

| 文件 | 说明 |
|------|------|
| `state_machine.h` | 模块头文件，状态枚举、段类型定义与 API 声明 |
| `state_machine.c` | 模块实现，路径序列表定义 |

## 状态定义

### 题目状态

```c
typedef enum {
    STATE_IDLE = 0,   // 空闲，等待按键
    STATE_TASK1,      // 任务 1（A→B 左边直线）
    STATE_TASK2,      // 任务 2（A→C→D→B→A 两弧两直）
    STATE_TASK3,      // 任务 3（A→C→B→D→A 弦线+弧线交替）
    STATE_TASK4       // 任务 4（同 TASK3 路径，跑 4 圈）
} QuestionState_t;
```

### 路径段类型

```c
typedef enum {
    SEG_ARC,         // 循迹段（沿黑线弧线走，灰度传感器 + PID 转向）
    SEG_STRAIGHT,    // 角度保持段（直线/弦线，无黑线，IMU 航向锁定）
    SEG_STOP,        // 停车（任务完成或无路径）
} SegmentType_t;

typedef struct {
    SegmentType_t type;
    float         delta_deg;     // SEG_STRAIGHT 时的相对偏转角（°）
                                 // 0=直走, >0=左转, <0=右转
} PathSegment_t;
```

## 路径序列表

每条路径是一个 `PathSegment_t` 数组，在 `StateMachine_StartTask()` 时加载。

### TASK1: A → B（左边直线）

```
[SEG_STRAIGHT, 0°]
```
保持当前朝向直走，直到重新压线停车。

### TASK2: A → C(上半圆弧) → D(右边直线) → B(下半圆弧) → A(左边直线)

```
[SEG_ARC,      —]    AC 上半圆弧循迹
[SEG_STRAIGHT, -30°]  CD 弦直线，出弧后右转 30°
[SEG_ARC,      —]    DB 下半圆弧循迹
[SEG_STRAIGHT, -30°]  BA 弦直线，出弧后右转 30°
```

> **注意**：delta_deg 为负值 = 右转。上赛道后需根据实际几何关系实测调整。

### TASK3 / TASK4: A → C(弦) → B(弧) → D(弦) → A(弧)

```
[SEG_STRAIGHT, 0°]   AC 弦直线，保持当前朝向
[SEG_ARC,      —]     弧线段循迹
[SEG_STRAIGHT, 0°]    弦直线
[SEG_ARC,      —]     弧线段循迹
```

TASK4 与 TASK3 使用同一路径表，通过圈数计数器自动循环 4 圈。

## 核心机制

### 按键 → 状态映射

模块在 `StateMachine_Init()` 时注册按键回调（通过 [Key 模块](../../modules/key/key.md)），按键索引直接映射到状态：

| 按键 | key_index | 切换到的状态 |
|------|-----------|-------------|
| KEY1 | 0 | `STATE_TASK1` |
| KEY2 | 1 | `STATE_TASK2` |
| KEY3 | 2 | `STATE_TASK3` |
| KEY4 | 3 | `STATE_TASK4` |

每次按键触发时，LED 翻转作为视觉反馈。

### 任务启动

`main.c` 中检测到状态变化（非 IDLE）时调用 `StateMachine_StartTask(task)`：
- 加载对应路径数组
- 重置段索引、圈数、完成标志
- 若路径首段为 `SEG_STRAIGHT`，设置 `s_waiting_leave = true`

### 段推进

段切换由 `main.c` 中的传感器检测逻辑触发，**不**由状态机内部自动推进：

| 当前段 | 推进条件 | 调用 |
|--------|---------|------|
| SEG_ARC | 灰度传感器连续 5 次全白（丢线） | `StateMachine_SegmentDone()` |
| SEG_STRAIGHT | 先离开起点黑线 → 重新压线 | `StateMachine_SegmentDone()` |

`StateMachine_SegmentDone()` 将段索引 +1。若到达数组末尾：
- TASK4 且圈数 < 4：圈数 +1，段索引重置为 0
- 其他：设置 `s_finished = true`

### 弦线段 debounce（"先离开再回来"）

弦线段起点在黑线上，小车需要**先离开黑线**再**重新遇到黑线**才表示到达终点。直接压线不能触发推进（那可能是起点线）。

```
StateMachine_NeedLeaveFirst() → true  → main.c 等待传感器全白
    → StateMachine_LeftLine()          → s_waiting_leave = false
        → main.c 等待传感器重新压线
            → StateMachine_SegmentDone()
```

### TASK4 圈数

`StateMachine_GetLapCount()` 返回当前圈数（1~4），可用于 OLED 显示。

## API

### 初始化

```c
void StateMachine_Init(void);
```
注册按键回调。在 `main.c` 中 `Board_Init()` 之后调用。

### 状态查询

```c
QuestionState_t StateMachine_GetState(void);
```
获取当前题目状态。`main.c` 每轮循环调用以检测变化。

### 任务控制

```c
void StateMachine_StartTask(QuestionState_t task);
```
启动指定任务，加载路径序列。仅在状态从 IDLE 切换到非 IDLE 时调用一次。

### 路径段查询（主循环每轮调用）

```c
SegmentType_t StateMachine_GetCurrentSegment(void);
```
获取当前段的类型。任务完成或无路径时返回 `SEG_STOP`。

```c
float StateMachine_GetDeltaDeg(void);
```
获取当前直线段的相对偏转角（°）。仅 `SEG_STRAIGHT` 时有效，其他段返回 0。

### 段推进

```c
void StateMachine_SegmentDone(void);
```
当前段完成，推进到下一段。TASK4 自动处理圈数循环。

### 终点与圈数

```c
bool StateMachine_IsFinished(void);
uint8_t StateMachine_GetLapCount(void);
```

### 弦线段 debounce

```c
bool StateMachine_NeedLeaveFirst(void);   // 是否需要先离开起点黑线
void StateMachine_LeftLine(void);         // 通知已离开起点黑线
```

## 使用示例

```c
#include "state_machine.h"

// 初始化
StateMachine_Init();

// 主循环
while (1) {
    QuestionState_t st = StateMachine_GetState();

    // 检测状态变化 → 启动任务
    if (st != last_state) {
        if (st != STATE_IDLE) {
            StateMachine_StartTask(st);
        }
    }

    // 根据当前段类型选择控制模式
    SegmentType_t seg = StateMachine_GetCurrentSegment();

    if (seg == SEG_ARC) {
        // 循迹模式
        if (连续丢线) {
            StateMachine_SegmentDone();
        }
    } else if (seg == SEG_STRAIGHT) {
        // 角度保持模式
        float delta = StateMachine_GetDeltaDeg();
        if (StateMachine_NeedLeaveFirst()) {
            if (全白) StateMachine_LeftLine();
        } else {
            if (压线) StateMachine_SegmentDone();
        }
    } else {
        Motor_Brake();
    }

    // 终点停车
    if (StateMachine_IsFinished()) {
        Motor_Brake();
    }
}
```

## 依赖

- [Key 按键模块](../../modules/key/key.md)（按键回调注册）

## 扩展指南

### 添加新任务

1. 在 `state_machine.c` 中定义新的 `PathSegment_t` 数组
2. 在 `StateMachine_StartTask()` 的 `switch` 中添加 case
3. 如需新的段类型，在 `SegmentType_t` 枚举中添加，并在 `main.c` 中添加对应的控制逻辑

### 调整路径

直接修改 `state_machine.c` 中对应任务的 `PathSegment_t` 数组：
- 增删段：修改数组元素
- 调整偏转角：修改 `delta_deg` 值（上赛道实测后填入）
- 调整段顺序：重新排列数组元素

```c
// 示例：修改 TASK2 的 CD 段偏转角
static const PathSegment_t path_task2[] = {
    { SEG_ARC,        0.0f },
    { SEG_STRAIGHT, -35.0f },   // 从 -30° 改为 -35°
    { SEG_ARC,        0.0f },
    { SEG_STRAIGHT, -35.0f },
};
```
