# Mahony 互补滤波器

## 概述

Mahony 互补滤波器是一种基于四元数的姿态解算算法，通过**加速度计测量值修正陀螺仪积分漂移**。相比扩展卡尔曼滤波（EKF），Mahony 滤波器计算量更小，适合 Cortex-M0+ 等无 FPU 的 MCU。

## 算法原理

### 核心思想

陀螺仪角速度积分 → 得到快速但会漂移的姿态。  
加速度计测量重力方向 → 得到无漂移但有噪声的姿态参考。  
互补滤波器将两者融合：低频信任陀螺仪积分，高频信任加速度计修正。

### 算法流程

```
1. 加速度归一化
   normalise = invSqrt(ax² + ay² + az²)
   acc *= normalise

2. 计算加速度与重力方向的叉积误差
   ex = acc.y × R[2][2] - acc.z × R[2][1]
   ey = acc.z × R[2][0] - acc.x × R[2][2]
   ez = acc.x × R[2][1] - acc.y × R[2][0]

3. 积分误差累加
   exInt += Ki × ex × dt

4. PI 修正陀螺仪
   gyro += Kp × error + errorInt

5. 一阶龙格库塔法更新四元数
   halfT = dt / 2
   q0 += (-q1×gx - q2×gy - q3×gz) × halfT
   q1 += ( q0×gx + q2×gz - q3×gy) × halfT
   q2 += ( q0×gy - q1×gz + q3×gx) × halfT
   q3 += ( q0×gz + q1×gy - q2×gx) × halfT

6. 四元数归一化

7. 更新旋转矩阵 → 解算欧拉角
   pitch = -asin(R[2][0]) × RAD2DEG
   roll  =  atan2(R[2][1], R[2][2]) × RAD2DEG
   yaw   =  atan2(R[1][0], R[0][0]) × RAD2DEG
```

## 参数配置

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `Kp` | 18.0 | 比例增益。越大 → 收敛越快但噪声更敏感 |
| `Ki` | 0.002 | 积分增益。补偿陀螺仪常值零偏 |
| `dt` | 0.002 | 采样周期（秒），对应 500Hz / 2ms 间隔 |

### 参数选择原则

- **Kp**：控制加速度计对姿态修正的"力度"。Kp 太大 → 姿态抖动；Kp 太小 → 收敛慢
- **Ki**：消除陀螺仪常值零偏。Ki 太大 → 超调；Ki 太小 → 零偏消除慢
- **dt**：必须与实际调用间隔一致，否则积分项计算错误

## 类型定义

### `Mahony_t` — 滤波器句柄

```c
typedef struct Mahony_t {
    float Kp, Ki, dt;               // 滤波器参数
    Axis3f gyro, acc;               // 传感器原始数据
    float exInt, eyInt, ezInt;      // 积分误差
    float q0, q1, q2, q3;          // 四元数
    float rMat[3][3];              // 旋转矩阵
    float pitch, roll, yaw;        // 欧拉角 (°)

    // 方法指针（由 Mahony_Init 绑定）
    void (*init)(...);
    void (*input)(...);
    void (*update)(...);
    void (*output)(...);
    void (*update_rotation_matrix)(...);
} Mahony_t;
```

## API

### `void Mahony_Init(Mahony_t *mf, float Kp, float Ki, float dt)`

初始化滤波器，配置增益与采样周期。四元数初始化为 `(1, 0, 0, 0)`，旋转矩阵初始化为单位阵。

### `void Mahony_Input(Mahony_t *mf, Axis3f gyro, Axis3f acc)`

输入最新陀螺仪（rad/s）和加速度计（m/s²）数据。

### `void Mahony_Update(Mahony_t *mf)`

执行一次滤波更新（核心算法，见上文流程）。

### `void Mahony_Output(Mahony_t *mf)`

从旋转矩阵解算欧拉角（pitch, roll, yaw），单位度。

## 数学工具 (deewaz_math.h)

```c
typedef struct { float x, y, z; } Axis3f;    // 三维向量

#define DEG2RAD  0.0174532925f               // 度 → 弧度
#define RAD2DEG  57.29577951f                // 弧度 → 度

static inline float invSqrt(float x) {       // 快速平方根倒数（归一化用）
    if (x <= 0.0f) return 1.0f;
    return 1.0f / sqrtf(x);
}
```

> 在 Cortex-M0+ 上实际调用 `1.0/sqrtf(x)`（标准库实现），因为该 MCU 不支持快速平方根倒数的位操作技巧（需要 FPU）。

## 使用示例

```c
#include "mahony.h"

Mahony_t mf;

// 初始化：Kp=18.0, Ki=0.002, dt=0.002s (500Hz)
Mahony_Init(&mf, 18.0f, 0.002f, 0.002f);

// 每 2ms 循环
Axis3f acc  = {ax, ay, az};  // 加速度计 m/s²
Axis3f gyro = {gx, gy, gz};  // 陀螺仪 rad/s

Mahony_Input(&mf, gyro, acc);
Mahony_Update(&mf);
Mahony_Output(&mf);

printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",
       mf.roll, mf.pitch, mf.yaw);
```

## 注意事项

- **Yaw 漂移**：Mahony 滤波器仅用加速度计修正 Roll/Pitch（重力参考），Yaw 角会随时间漂移。需要磁力计才能消除 Yaw 漂移
- **dt 一致性**：`dt` 参数必须与实际调用间隔匹配。如果调用频率不稳定，滤波器性能会下降
- **初始对准**：启动后需要几秒钟让滤波器收敛到正确姿态。期间加速度计应处于静止状态
- **高动态运动**：加速度计在高加速度运动时（如急转弯），重力方向测量会被干扰，导致姿态估计短暂偏差
