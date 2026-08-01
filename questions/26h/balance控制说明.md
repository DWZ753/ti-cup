# 球-梁平衡控制 — 原理、实现与调参

> 写给需要理解和使用 balance 模块的队员。

---

## 一、物理模型

```
        钢球 (~1cm)
          ○
  ════════════════════  PPR水管 (25cm, 内壁光滑)
     ▲           ▲
     │           │
  ZDT电机    合页支点 (距车面5cm)
  控制倾角
```

### 核心方程

球在光滑管内纯滚动，加速度由摆杆倾角决定：

$$\ddot{x}_{ball} = \frac{5}{7} \cdot g \cdot \sin(\theta_{beam})$$

小角度（θ < 15°）简化：

$$\ddot{x}_{ball} \approx \frac{5}{7} \cdot 9.8 \cdot \theta \approx 7.0 \cdot \theta \quad (\text{m/s}^2,\ \theta \text{ 为弧度})$$

**例**：摆杆倾斜 1°（≈0.017 rad）→ 球加速度 ≈ 0.12 m/s²。

### "双积分"系统

```
θ_beam（摆杆角度）
    ↓  ×7.0
球加速度 ẍ
    ↓  ∫dt
球速度 ẋ
    ↓  ∫dt
球位置 x
```

从角度到位置经过**两次积分**。这意味着：
- 纯比例控制（P）必然**等幅振荡**——球在目标点附近来回滚，永远停不住
- 必须加微分项（D）提供**阻尼**，才能收敛

### 底盘运动的干扰

| 底盘运动 | 对球的影响 | 等效加速度 |
|----------|-----------|-----------|
| 直线加速 | 球相对向后滚 | `-a_car` |
| 直线减速 | 球相对向前滚 | `+a_car` |
| 转弯（半径 R，速度 v） | 球向外侧甩 | `v²/R` |

补偿思路：提前让摆杆向相反方向倾斜，抵消惯性力。

---

## 二、控制架构

### 完整框图

```
                         ┌──────────────┐
                         │ 底盘加速度前馈 │
                         │ encoder→速度差分│
                         │ ÷g → 补偿倾角  │
                         └───────┬──────┘
                                 │ θ_ff
                                 ▼
ball_pos_mm ──→ [低通] ──→ [PD] ──→ ⊕ ──→ θ_cmd ──→ ZDT 电机
  (Pi@50Hz)        │         ▲                │
                   │         │                ▼
                   └──→ [速度估计]        摆杆角度 → 球物理
                        (一阶差分+LPF)
```

### PD 控制律（[balance.c:193-215](../application/balance/balance.c#L193-L215)）

```c
pos_error = target - ball_pos;              // 位置偏差 (mm)
angle_cmd = KP * pos_error + KD * ball_vel; // PD 输出 (°)
angle_cmd += chassis_feedforward;            // 叠加底盘前馈
angle_cmd = clamp(angle_cmd, ±10°);         // 安全钳位
Set_Angle(angle_cmd);                       // 发给 ZDT 电机
```

### 三个控制项的作用

| 项 | 公式 | 作用 | 类比 |
|----|------|------|------|
| **P** | `KP × pos_error` | 球偏离越远，摆杆倾角越大 | 弹簧：把球拉回目标 |
| **D** | `KD × ball_vel` | 球滚得越快，反向倾角越大 | 阻尼：把球"刹车"停住 |
| **FF** | `5.8 × car_accel` | 车加速时提前补偿 | 预判：不等球动就先调摆杆 |

**P 和 D 的配合**：
- 只有 P → 永远振荡（弹簧没有阻尼）
- P + D → 收敛到目标（带阻尼的弹簧）
- 加 FF → 底盘扰动被预先抵消，P+D 只需处理残余误差

---

## 三、数据流

```
Pi 摄像头 → 球位置 (mm) → UART COBS 帧 (50Hz)
    ↓
main.c → Balance_Update(pos, vel, conf)
    ↓
balance.c 内部：
  1. 低通滤波球位置（POS_FILTER_ALPHA = 0.3）
     s_ball_pos = 0.7*s_ball_pos + 0.3*raw_pos
  2. 一阶差分估计球速度 + 低通（VEL_FILTER_ALPHA = 0.2）
     raw_vel = (pos - last_pos) / dt
     s_ball_vel = 0.8*s_ball_vel + 0.2*raw_vel
  3. PD 计算 → 倾角指令
  4. Set_Angle() → ZDT_Motor_QPos_Control() → 电机转动
    ↓
摆杆倾角变化 → 球受力滚动 → Pi 读到新位置 → 闭环
```

### Pi 发送的球位置协议

```
帧: [COBS(0x25 + pos_mm(int16,BE) + confidence(uint8) + XOR)] 0x00
```

| 字段 | 类型 | 说明 |
|------|------|------|
| cmd | uint8 = 0x25 | CMD_BALL_POS |
| pos_mm | int16, 大端 | 球相对 O 点位置 (mm)，正=右，负=左 |
| confidence | uint8 | 0=LOST(冻结), 1=LOW(降KP), 2=GOOD(正常) |

Pi 端 `protocol.py` 已有 `CMD_BALL_POS = 0x25`，在 `main.py` 检测循环中调用：

```python
uart.send(CMD_BALL_POS, struct.pack('>hB', pos_mm, confidence))
```

---

## 四、两种工作模式

### 模式 A：静态平衡（要求 3）— 双模式（闭环优先 + 开环降级）

车静止。`Balance_Start()` 启动 → 主循环调 `Balance_SeqUpdate()` 推进。

**优先使用闭环模式**（Pi 在线）：通过 `Balance_Update()` 接收 Pi 的球位置反馈，
PD 实时控制摆杆倾角使球移动到目标 mm 位置再停留。

**自动降级到开环模式**（Pi 离线）：如果超过 `PI_TIMEOUT_MS`(200ms) 无 Pi 帧，
自动切换到开环角度序列表（`balance_config.h` 中的 `OPEN_LOOP_SEQ_*` 宏）：

| 步骤 | 倾角 | 时长 | 效果 |
|------|------|------|------|
| 0 | +5° | 600ms | 球加速滚向 +5cm |
| 1 | -4° | 250ms | 反向倾角减速刹车 |
| 2 | 0° | 1500ms | 停在 +5cm 处 |
| 3 | -5° | 600ms | 球加速滚向 -5cm |
| 4 | +4° | 250ms | 减速刹车 |
| 5 | 0° | 1500ms | 停在 -5cm 处 |

各步骤角度和时间可在 `balance_config.h` 中独立调整。

### 模式 B：PD 闭环（要求 4/5/6）— 需要 Pi 送球位置

Pi 每 ~50Hz 发 `CMD_BALL_POS` → MCU 调 `Balance_Update()` → PD 实时计算倾角。
球速度由 MCU 端宽窗差分估计（3 帧间隔，~40ms 延迟），不依赖 Pi 提供速度数据。

**confidence 处理策略**：
| 值 | 含义 | MCU 行为 |
|----|------|----------|
| 2 (GOOD) | 正常 | 全 PD 控制 |
| 1 (LOW) | 低置信 | PD 控制，KP×0.5 |
| 0 (LOST) | 丢球 | 冻结摆杆角度，不更新 |

---

## 五、参数表（[balance_config.h](../application/balance/balance_config.h)）

### ZDT 电机

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `BALANCE_MOTOR_ID` | 1 | ZDT 电机地址 |
| `BALANCE_WORK_VEL         70 | 工作转速 (RPM) |
| `BALANCE_MAX_ANGLE_DEG` | 10.0 | 摆杆最大倾角 (°)，安全限幅 |
| `BALANCE_SKIP_HOMING` | 0 | 0=硬停回零，1=跳过(合页未到时用) |
| `BALANCE_ZERO_DIR` | 0 | 回零方向：0=CW下转碰地，1=CCW（首次上电实测） |
| `BALANCE_HOME_OFFSET_DEG  -45.5 | 回零后上抬到水平的偏移角度(°)，⚠️ 必须实测标定 |
| `BALANCE_ZERO_VEL` | 30 | 回零速度 (RPM) |
| `BALANCE_ZERO_CUR_MA        800 | 回零碰撞检测电流 (mA) |
| `BALANCE_ZERO_TIME_MS` | 50 | 回零碰撞检测时间 (ms) |

### PD 参数

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `BALANCE_KP` | 0.3 | °/mm | 1mm 位置误差 → 多少度倾角 |
| `BALANCE_KD` | 0.5 | °/(mm/s) | 1mm/s 球速 → 多少度反向倾角 |

### 前馈

| 参数 | 默认值 | 单位 | 含义 |
|------|--------|------|------|
| `FF_ACCEL_GAIN` | 5.8 | °/(m/s²) | 1m/s² 底盘加速度 → 多少度补偿 |

### 滤波器

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `POS_FILTER_ALPHA` | 0.3 | 位置低通系数，越大响应越快但越抖 |
| `VEL_FILTER_ALPHA` | 0.2 | 速度低通系数，越大越抖但跟得上快变化 |

---

## 六、调参步骤

### 第 0 步：确认机械参数（合页到后首次上电）

1. 确认 `BALANCE_SKIP_HOMING = 0`、`BALANCE_HOME_OFFSET_DEG  -45.5`
2. 上电 → 电机下转碰地 → 电流尖峰 → 自动停止 → 上抬 25°
3. 如果电机**上转**而不是下转 → 改 `BALANCE_ZERO_DIR`（0→1 或 1→0）
4. 上抬后摆杆不够水平 → 加大 `BALANCE_HOME_OFFSET_DEG`；太高 → 减小。反复几次直到肉眼水平
5. 遥控 `CMD_BEAM` 正负验证：正值摆杆右倾→球往右滚。反了就把 `BALANCE_PULSE_PER_DEG` 改成负值

### 第 1 步：静态开环调时序（要求 3，车不动）

用小倾角（2~3°）测试球的大概滚动速度：

| 现象 | 调法 |
|------|------|
| 球滚得慢，没到 ±5cm | 加大 `STATIC_SEQ_ANGLE` 或加长 `STATIC_SEQ_TIME` |
| 球冲过 ±5cm | 加大减速阶段倾角（如 -4° → -6°） |
| 球没到就开始减速 | 减小减速倾角，或加长加速阶段时间 |

### 第 2 步：调 KP（位置响应）

从 0.1 开始。可用遥控把球拨到一边，然后设 target=0 观察。

| 现象 | 调法 |
|------|------|
| 球回正太慢 | 加大 KP（每次 +0.05） |
| 球冲过 O 点大幅振荡 | 减小 KP，或加大 KD |
| 球停在 O 点附近但来回微滚 | 加大 KD |

### 第 3 步：调 KD（阻尼/刹车）

从 0 开始，逐步加到球到达目标后不再滚过头：

| 现象 | 调法 |
|------|------|
| KD 太小 | 球到了 O 点停不住，继续滚到另一侧 |
| KD 太大 | 摆杆反应过激，高频抖动 |
| 合适 | 球到 O 点后轻微超调 2~3mm，1~2 秒内稳定 |

### 第 4 步：调低通滤波器

一般不用调。Pi 画面稳定的话默认值就行。

| 问题 | 调法 |
|------|------|
| Pi 摄像头抖动严重 | 减小 `POS_FILTER_ALPHA`（更平滑但滞后） |
| 球速估计噪声大 | 减小 `VEL_FILTER_ALPHA` |

### 第 5 步：底盘前馈（车上赛道后）

在 `main.c` 循迹循环中加入（每 10ms）：

```c
static float s_last_speed = 0;
float current_speed = Motor_GetSpeed();  // 左右轮平均速度
float accel = (current_speed - s_last_speed) / 0.01f;  // mm/s²
s_last_speed = current_speed;
Balance_ChassisFF(accel * 0.001f);  // 转为 m/s²
```

`FF_ACCEL_GAIN = 5.8` 是理论值（1/g × 180/π）。

| 现象 | 调法 |
|------|------|
| 车加速时球往后滚 | 加大 FF_GAIN |
| 车加速时球往前冲 | 减小 FF_GAIN |

---

## 七、调试流程（推荐顺序）

```
1. 遥控测 ZDT 电机
   ↓  选 "7.Remote"，按 1-5 数字键设摆杆角度
   ↓  确认方向和幅度正确
   ↓
2. 跑静态平衡
   ↓  选 "3.Ball Balance"，KEY1 启动
   ↓  观察球是否完成 O→+5→-5 往返
   ↓  调整时序参数直到球能到位
   ↓
3. 接 Pi 闭环
   ↓  Pi 连 UART + 摄像头
   ↓  选 "4.Run+Bal AB" 或 "5.Run+Bal 1Lap"
   ↓  PD 接管，调 KP/KD/FF
```

---

## 八、文件索引

| 文件 | 内容 |
|------|------|
| [balance_config.h](../application/balance/balance_config.h) | 全部可调参数 |
| [balance.h](../application/balance/balance.h) | API 声明（9 个函数） |
| [balance.c](../application/balance/balance.c) | 实现：ZDT 控制 + PD 控制律 + 静态序列 |
| [board.c:37](../board.c#L37) | `Balance_Init()` 调用位置 |
| [main.c:26-27](../main.c#L26-L27) | Pi 遥控命令码定义 |
| [main.c:191-215](../main.c#L191-L215) | `Balance_Update()` 调用入口（需要加 Pi 帧处理） |
| [mcu开发指引.md](mcu开发指引.md) | 整体开发任务清单 |
