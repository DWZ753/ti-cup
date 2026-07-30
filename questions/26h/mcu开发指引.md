# 26H MCU 侧开发指引 — 球-梁平衡控制系统

> **重点**：本文档涵盖7月30日需要完成的所有代码任务。
> 先读完全文再动手，不要跳过任何部分。

---

## 一、需要做什么（总览）

MSPM0 小车端已有功能：
- ✅ 灰度循迹 + 舵机转向（[main.c](../../main.c) + [chassis_config.h](../../application/chassis/chassis_config.h)）
- ✅ 直流电机差速驱动
- ✅ ZDT 闭环步进电机云台模块（[application/gimbal/](../../application/gimbal/gimbal.h)）
- ✅ PID 控制器（[lib/application/pid/](../../../lib/mspm0/application/pid/pid.h)）
- ✅ UART COBS 通信协议（[lib/modules/protocol/](../../../lib/mspm0/modules/protocol/)）

**需要新增：**

| 优先级 | 内容 | 文件 | 预计时间 |
|--------|------|------|----------|
| 🔴 P0 | **球位置 PID 控制器**（外环） | `application/balance/balance.c/h` | 2h |
| 🔴 P0 | **Pi→MSPM0 通信接收**（解析 CMD_BALL_POS） | 改 `main.c` + `board.c` | 1h |
| 🟡 P1 | **底盘加速度前馈**（编码器→速度差分→补偿倾角） | `application/balance/balance.c` | 1h |
| 🟡 P1 | **ZDT 回零方向实机测试** | 改 `application/gimbal/gimbal.c` | 30min |
| 🟢 P2 | **PID 调参**（上赛道后） | `application/balance/balance_config.h` | 持续调 |
| 🟢 P2 | **UART 遥测输出**（调试用） | `main.c` | 30min |

---

## 二、球-梁系统动力学

### 2.1 物理模型

```
        钢球 (~1cm)
          ○
  ════════════════════  PPR水管 (25cm, 内壁光滑)
     ▲           ▲
     │           │
  ZDT电机    支点O (距车面5cm)
  控制倾角
```

**核心方程**（球在光滑管内滚动）：

$$ \ddot{x}_{ball} = \frac{5}{7} \cdot g \cdot \sin(\theta_{beam}) $$

简化（小角度，θ < 15°）：

$$ \ddot{x}_{ball} \approx \frac{5}{7} \cdot 9.8 \cdot \theta \approx 7.0 \cdot \theta \quad (\text{m/s}^2) $$

**这意味着**：摆杆倾斜 1°（≈0.017 rad），球获得约 0.12 m/s² 的加速度。

### 2.2 "双积分"系统

```
θ_beam（摆杆角度）
    ↓  ×7.0
球加速度 ẍ
    ↓  ∫dt
球速度 ẋ
    ↓  ∫dt
球位置 x
```

从角度到位置经过两次积分，纯粹的比例控制（P）会产生**等幅振荡**——球在目标点附近来回滚动。需要微分项（D）提供阻尼。

### 2.3 底盘运动的干扰

当小车加速/减速/转弯时，球会受到惯性力：

| 底盘运动 | 对球的影响 | 等效加速度 |
|----------|-----------|-----------|
| 直线加速 | 球相对向后滚 | `a_ball = -a_car` |
| 直线减速 | 球相对向前滚 | `a_ball = +a_car`（刹车时球往前冲） |
| 转弯（半径 R，速度 v） | 球向外侧甩 | `a_ball = v²/R`（离心力） |

**补偿思路**：提前让摆杆向相反方向倾斜，抵消惯性力。

$$ \theta_{comp} = \arctan\left(\frac{a_{disturbance}}{g}\right) \approx \frac{a_{disturbance}}{9.8} \quad (\text{rad}) $$

例如：车以 0.5 m/s² 加速 → 补偿倾角 ≈ 0.05 rad ≈ 2.9°

---

## 三、控制架构设计

### 3.1 完整控制框图

```
                         ┌──────────────┐
                         │ 底盘加速度前馈 │
                         │ encoder→速度差分│
                         │ ÷g → 补偿倾角  │
                         └───────┬──────┘
                                 │ θ_ff
                                 ▼
ball_pos_mm ──→ [低通] ──→ [PD] ──→ ⊕ ──→ θ_cmd ──→ gimbal.SetAngle()
  (Pi@50Hz)        │         ▲                │
                   │         │                ▼
                   └──→ [速度估计]         ZDT电机 → 摆杆物理
                        (一阶差分+LPF)
```

### 3.2 PD 控制律（核心代码）

```c
// 每收到一次 Pi 的位置更新（50Hz）运行一次

// 1. 低通滤波（抑制摄像头抖动）
float filtered_pos = s_ball_pos * 0.7f + raw_pos * 0.3f;

// 2. 位置误差
float pos_error = target_pos_mm - filtered_pos;

// 3. 速度估计（一阶差分 + 低通）
float raw_vel = (filtered_pos - s_last_ball_pos) / dt;
float ball_vel = s_ball_vel * 0.8f + raw_vel * 0.2f;

// 4. PD 输出 → 角度指令
float angle_p = KP_BALL_POS * pos_error;      // 比例：偏离越大，倾角越大
float angle_d = KD_BALL_VEL * ball_vel;       // 微分：球往哪滚，反向倾
float angle_cmd = angle_p + angle_d;

// 5. 叠加底盘前馈
angle_cmd += chassis_feedforward();

// 6. 钳位 + 发送
angle_cmd = CLAMP(angle_cmd, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
Gimbal_SetAngle(angle_cmd);
```

### 3.3 参数物理意义

| 参数 | 物理意义 | 初始值建议 | 调大效果 |
|------|---------|-----------|---------|
| `KP_BALL_POS` | 1mm 位置误差 → 多少度倾角 | 0.3°/mm | 响应更快，但容易过冲 |
| `KD_BALL_VEL` | 1mm/s 球速 → 多少度反向倾角 | 0.5°/(mm/s) | 阻尼更强，但响应变慢 |
| `FF_ACCEL_GAIN` | 1m/s² 底盘加速度 → 多少度补偿 | 5.8°/(m/s²) | 惯性补偿力度 |

**KP_BALL_POS = 0.3 的含义**：球偏了 10mm，摆杆倾 3°。球获得的加速度 ≈ 7.0 × sin(3°) ≈ 0.37 m/s² ≈ 370 mm/s²，能在约 0.23s 内开始回正。

### 3.4 前馈设计

球-梁系统有两个需要前馈补偿的环节：

**a) 球速度前馈（即 PD 的 D 项）**

球自身的运动速度是反馈控制无法提前感知的——当传感器读到球正在向右滚时，它已经有速度了。纯 P 控制要等到位置误差变大才加大倾角，必然滞后。

D 项的作用：`angle_d = KD_BALL_VEL × ball_vel`，球速度越大，反向倾角越大。相当于提前"刹车"，不必等到位置偏离后才反应。

**b) 底盘加速度前馈**

见 2.3 节的分析。底盘加速/减速/转弯时，球受惯性力相对管子滑动。这是可测量的外部扰动，不需要等球位置变化后再修正。

实现：在 10ms 循迹周期中对编码器速度做差分得到底盘加速度，换算为补偿倾角叠加到 PD 输出上。

```c
float chassis_feedforward(void)
{
    // 编码器速度差分（10ms 周期）
    float car_accel = (current_speed - last_speed) / 0.01f;  // mm/s²
    last_speed = current_speed;

    // 转为补偿倾角
    // θ = atan(a/g) ≈ a/g（小角度）
    float accel_m_s2 = car_accel * 0.001f;
    return FF_ACCEL_GAIN * accel_m_s2;  // °
}
```

**前馈 vs 反馈的关系**：

```
反馈(PD) ──→ 纠正已经发生的位置偏差（滞后但保证稳态精度）
前馈(FF) ──→ 预判即将发生的扰动并提前抵消（超前但依赖模型精度）
```

两者叠加使用：前馈抵消大部分可预测扰动，反馈处理剩余误差和建模不准的部分。

---

## 四、现有代码基础

### 4.1 gimbal 模块（直接调用，不需要改）

位置：[application/gimbal/gimbal.h](../../application/gimbal/gimbal.h)

```c
Gimbal_Init();           // 上电自动归零（阻塞 ~3s）
Gimbal_SetAngle(5.0f);   // 摆杆上抬 5°
Gimbal_IsIdle();         // 查询是否到位（当前未完整实现，见 gimbal.md 待办）
Gimbal_Stop();           // 紧急停止
```

**需要先做的实机测试**：确认回零方向（`o_dir`），见 [gimbal.md](../../application/gimbal/gimbal.md) 待实测表。上电跑一遍 `Gimbal_Init()`，看电机是向下转（碰地）还是向上转。向上转就把 `o_dir` 从 0 改成 1。

### 4.2 PID 模块

位置：[lib/application/pid/pid.h](../../../lib/mspm0/application/pid/pid.h)

```c
PID_Controller ball_pid;
PID_Init(&ball_pid, KP_BALL_POS, 0.0f, KD_BALL_VEL, INT_LIMIT, OUT_LIMIT);
PID_SetTarget(&ball_pid, 0.0f);  // 目标：球在 O 点

// 每次 Pi 更新时：
float output = PID_Compute(&ball_pid, ball_pos_mm);
// output 就是 angle_cmd
```

**但注意**：标准 PID 的 D 项是对 error 微分，而我们需要的是对 ball_vel 的独立 D 项（不是 error 的 D）。建议自己写 PD 而不是用现成的 PID_Compute，或者把 target 设 0、current_value 设为 `(position_error + ball_vel * kd_ratio)`。

### 4.3 通信协议

RPi 通过 UART 以 50Hz 频率发送：

```
[COBS(0x25 + ball_pos_mm(int16,BE) + confidence(uint8) + XOR)] 0x00
```

- `ball_pos_mm`：球相对 O 点的位置（mm），int16_t，大端序
- `confidence`：0=丢球，1=低置信，2=正常

需要在 `main.c` 或新的 `balance.c` 中完成：
1. 注册 UART 接收回调（protocol 模块已有帧解析）
2. 收到 `CMD_BALL_POS (0x25)` → 解析 payload → 更新 `s_ball_pos`
3. 当 `confidence == 0`（丢球）→ 冻结控制输出，摆杆保持原位

---

## 五、代码任务清单

### 任务 1：新建 `application/balance/` 模块

**文件结构**：
```
application/balance/
├── balance.h          ← API：Init / SetTarget / Update / 状态查询
├── balance.c          ← 实现：PD 控制 + 底盘前馈
└── balance_config.h   ← 所有可调参数（KP/KD/FF_GAIN/角度限幅...）
```

**balance.h 最小 API**：

```c
void Balance_Init(void);                    // 初始化（内部调 Gimbal_Init）
void Balance_SetTarget(float pos_mm);       // 设置球目标位置（0=O点）
void Balance_Update(float ball_pos_mm,      // Pi 更新（50Hz 调用）
                    float ball_vel_mm_s,
                    uint8_t confidence);
void Balance_ChassisFF(float accel_m_s2);   // 底盘加速度前馈（可单独调）
float Balance_GetAngle(void);               // 查询当前摆杆角度
```

### 任务 2：实现底盘加速度估计

在 `main.c` 的循迹循环中，利用编码器速度做差分：

```c
// 在 10ms 循迹周期中
static float s_last_speed = 0;
float current_speed = Motor_GetSpeed();  // 左右轮平均速度 (mm/s)
float accel = (current_speed - s_last_speed) / 0.01f;  // mm/s²
s_last_speed = current_speed;

// 传给 balance 模块做前馈
Balance_ChassisFF(accel * 0.001f);  // 转为 m/s²
```

**注意 M0+ 无 FPU**：浮点运算可以接受（控制周期 50Hz，不需要很快），但如果发现 CPU 负载过高，可以考虑用定点（速度用 mm/s 整数，加速度用 Q16.16 定点）。

### 任务 3：对接 Pi 通信

在 `board.c` 中注册 Pi UART 实例，在 `main.c` 中处理接收：

```c
// 帧回调（protocol 模块在后台收帧，回调在 main 循环上下文执行）
void on_pi_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    if (cmd == 0x25 && len >= 3) {  // CMD_BALL_POS
        int16_t pos_mm = ((int16_t)payload[0] << 8) | payload[1];
        uint8_t conf   = payload[2];
        float pos = (float)pos_mm;  // mm
        float vel = 0.0f;           // 后续加差分估计
        Balance_Update(pos, vel, conf);
    }
}
```

### 任务 4：main.c 模式切换

当前 `main.c` 只有循迹模式。需要增加：

```c
typedef enum {
    MODE_IDLE,         // 待机
    MODE_TRACKING,     // 纯循迹（要求 2）
    MODE_BALANCE,      // 静态平衡（要求 3，车不动）
    MODE_COMBINED,     // 循迹+平衡（要求 4/5/6）
} RunMode;

// KEY1: 切换模式
// KEY2: 在 BALANCE 模式下切换目标位置（O=0, 左=-50, 右=+50）
```

### 任务 5：调参

**参数配置文件** `balance_config.h`：

```c
// 球位置 PD
#define BALANCE_KP          0.3f    // °/mm，位置误差→倾角
#define BALANCE_KD          0.5f    // °/(mm/s)，球速→反向倾角
#define BALANCE_MAX_ANGLE   10.0f   // 摆杆最大倾角（°），安全限幅

// 底盘前馈
#define FF_ACCEL_GAIN       5.8f    // °/(m/s²)

// 滤波器
#define POS_FILTER_ALPHA    0.3f    // 位置低通（越大越灵敏）
#define VEL_FILTER_ALPHA    0.2f    // 速度低通（越大噪声越大）

// 通信
#define PI_TIMEOUT_MS       200     // 超时判丢球
```

**上赛道后的调参顺序**：
1. 先做静态平衡（车不动，要求 3）→ 只调 KP_BALL_POS 和 KD_BALL_VEL
2. KP 从 0.1 开始，逐步加到球能快速回正但不振荡
3. KD 从 0 开始，逐步加到球停住后不再来回滚
4. 再做行驶平衡（要求 4）→ 加入底盘前馈
5. FF_ACCEL_GAIN 从理论值 5.8 开始，观察直道加速/减速时球是否被甩动

---

## 六、Cortex-M0+ 注意事项

1. **没有 FPU**：浮点运算走软件模拟，50Hz 下没问题，但不要放在 ISR 里
2. **ZDT 指令间隔**：连续发两条 ZDT 命令之间至少 `delay_ms(10)`，防止粘包
3. **Gimbal_Init 是阻塞的**（~3 秒），在 `Board_Init()` 中调用，不要在运行时调用
4. **UART 接收缓冲**：`protocol` 模块的帧解析在后台完成，`main()` 循环中只需处理回调
5. **IMU 可选**：如果你需要摆杆真实角度做反馈（而非只信任 ZDT 编码器），可以用 BMI088 的 pitch 角做冗余校验。但优先用 ZDT 编码器（精度更高、无漂移）

---

## 七、与 Pi 的交互协议总结

| 方向 | 命令 | 频率 | 内容 |
|------|------|------|------|
| Pi → MSPM0 | CMD_BALL_POS (0x25) | 50Hz | ball_pos_mm(int16) + confidence(uint8) |
| MSPM0 → Pi | 可扩展 | — | 当前摆杆角度、底盘速度等（调试用） |

Pi 发来的 confidence 处理策略：
- `2 (GOOD)`：正常，持续更新 PD 控制
- `1 (LOW)`：正常，但降低 KP（×0.5），减少激进控制
- `0 (LOST)`：冻结摆杆角度，不更新 PD；超过 500ms 连续丢球 → 摆杆回水平

---

## 八、测试验证清单

- [ ] ZDT 回零方向正确（电机下转碰地）
- [ ] `Gimbal_SetAngle()` 正负方向正确（正=上抬，负=下压）
- [ ] Pi 通信正常（能收到 0x25 帧，position 值合理）
- [ ] 静态平衡：球从 +50mm 回到 0mm，稳定在 ±1cm 内，时间 ≤5s
- [ ] 底盘加速度估计：编码器速度差分值符号正确（加速=正，减速=负）
- [ ] 前馈方向：车加速时摆杆前倾（下压），车减速时摆杆后仰（上抬）
- [ ] 行驶平衡：直道 AB 段球保持在 O 点 ±1cm
