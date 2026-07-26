# IMU 惯性测量单元模块

## 概述

IMU 姿态解算模块，封装 [BMI088 六轴传感器](bmi088/bmi088.md)（加速度计 + 陀螺仪）的 SPI 驱动和 [Mahony 互补滤波器](bmi088/mahony.md)，对外输出欧拉角（Roll/Pitch/Yaw）、四元数和原始传感器数据。

## 文件

| 文件 | 说明 |
|------|------|
| `imu.h` | IMU 模块头文件 |
| `imu.c` | IMU 模块实现（组装 BMI088 + Mahony） |
| `bmi088/bmi088.h` | BMI088 传感器 SPI 驱动 |
| `bmi088/bmi088.c` | BMI088 传感器实现 |
| `bmi088/mahony.h` | Mahony 互补滤波器 |
| `bmi088/mahony.c` | Mahony 滤波器实现 |

## 硬件说明

| 信号 | 引脚 | 说明 |
|------|------|------|
| SPI_SCK | PA12 | SPI 时钟 |
| SPI_MOSI | PA14 | 主机→传感器 |
| SPI_MISO | PA13 | 传感器→主机 |
| ACCEL_CS | PB13 | 加速度计片选 |
| GYRO_CS | PB15 | 陀螺仪片选 |

**传感器规格：** 加速度计 ±6g（0.001795 m/s²/LSB），陀螺仪 ±2000°/s（0.001065 rad/s/LSB）。

## API

### `void IMU_Init(void)`

初始化 IMU 子系统。内部依次：注册 SPI 实例 → BMI088 软复位 → Mahony 滤波器配置（Kp=18.0, Ki=0.002, dt=0.002s）。

> 由 `Board_Init()` 自动调用。

### `void IMU_Update(void)`

执行一次采样 + 姿态解算：读取加速度计/陀螺仪 → Mahony 互补滤波 → 更新四元数/旋转矩阵/欧拉角。

> 建议每 **2ms** 调用一次（与滤波器 dt=0.002s 一致）。Cortex-M0+ 无硬件 FPU，调用频率不宜超过 500Hz。

### `void IMU_GetEuler(float *roll, float *pitch, float *yaw)`

获取欧拉角（度）。

### `void IMU_GetQuaternion(float *q0, float *q1, float *q2, float *q3)`

获取四元数各分量。

### `void IMU_GetAccel(float accel[3])`

获取最新加速度计数据 `[x, y, z]`，单位 m/s²。

### `void IMU_GetGyro(float gyro[3])`

获取最新陀螺仪数据 `[x, y, z]`，单位 rad/s。

## 依赖

- [SPI 模块](../../bsp/spi/spi.md)
- [BMI088 驱动](bmi088/bmi088.md)
- [Mahony 滤波器](bmi088/mahony.md)
- `ti_msp_dl_config.h`

## 使用示例

```c
#include "imu.h"

// 初始化（由 Board_Init 自动调用）
IMU_Init();

// 每 2ms 更新
uint32_t last = Board_GetTickMs();
while (1)
{
    if (Board_GetTickMs() - last >= 2)
    {
        last = Board_GetTickMs();
        IMU_Update();
    }
}

// 读取姿态
float roll, pitch, yaw;
IMU_GetEuler(&roll, &pitch, &yaw);

// 读取原始数据
float accel[3], gyro[3];
IMU_GetAccel(accel);
IMU_GetGyro(gyro);
```

## 注意事项

- **Yaw 漂移**：Mahony 滤波器仅用加速度计修正 Roll/Pitch（重力参考），Yaw 角无磁力计校正会随时间漂移
- **采样一致性**：`IMU_Update()` 调用间隔应稳定在 2ms，偏差过大会降低滤波精度
- **初始对准**：启动后需静止几秒让滤波器收敛
