# 底盘控制子系统

## 概述

底盘运动控制模块（待实现）。当前包含：

- **tracking** — 灰度传感器循迹转向
- **angle** — 角度保持控制

## 文件

| 文件 | 说明 |
|------|------|
| `chassis.h` | 占位（底盘综合控制 API） |
| `chassis.c` | 占位 |
| `tracking.h` | 循迹转向控制 |
| `tracking.c` | 灰度传感器位置解析 + PID 转向输出 |
| `angle.h` | 角度保持控制 |
| `angle.c` | IMU yaw → PID → 舵机 |
| `angle.md` | 角度模块文档 |
| `tracking.md` | 循迹模块文档 |

## 依赖

- [PID 算法库](../pid/pid.h)
- [灰度传感器模块](../../modules/grayscale/grayscale.h)（循迹）
- [IMU 模块](../../modules/imu/imu.h)（角度保持）
