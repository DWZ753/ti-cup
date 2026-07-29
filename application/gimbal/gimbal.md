# Gimbal 单轴云台模块

## 概述

基于 ZDT 闭环步进电机（Emm_V5.0 协议）的单轴云台控制模块。
控制 pitch 轴（上下俯仰），搭载摄像头 + 机械臂（末端电磁铁）。

## 硬件参数

| 项目 | 值 | 说明 |
|------|-----|------|
| 电机型号 | ZDT 闭环步进（Y42） | RS485/UART 通信 |
| 电机 ID | 1 | 默认地址 |
| 细分 | 16 | 3200 pulse/圈 |
| 通信 | UART, 115200-8-N-1 | 协议见 zdt_motor.h |
| 归零方式 | 硬停回零（mode=2） | 机械臂碰地面 → 电流尖峰 → 自动停止 |

## 安全设计

| 保护 | 实现 | 参数 |
|------|------|------|
| 上电归零 | `Gimbal_Init()` 自动触发硬停回零 | 速度 30RPM, 电流 500mA, 检测 50ms |
| 过流保护 | ZDT 驱动板 OCP（`Modify_Otocp`） | 800mA, 触发时间 200ms |
| 角度上限 | `Gimbal_SetAngle()` 软件钳位 | [0, 45°] |
| 紧急停止 | `Gimbal_Stop()` | 立即停转 |

## API

| 函数 | 说明 | 阻塞 |
|------|------|------|
| `Gimbal_Init()` | 初始化 + 自动归零 | 是（~3s） |
| `Gimbal_SetAngle(deg)` | 设置 pitch 角度 | 否 |
| `Gimbal_IsIdle()` | 是否到达目标 | 否 |
| `Gimbal_Stop()` | 紧急停止 | 否 |
| `Gimbal_Rehome()` | 手动重新归零 | 是（~3s） |

## 角度定义

```
            0° ──── 机械臂触地（归零位置）
            ↑
          正值 = 向上抬起
          max = GIMBAL_MAX_ANGLE_DEG (45°)
```

## 依赖

- [ZDT Motor 模块](../../modules/zdt_motor/zdt_motor.h) — 电机底层通信
- [delay](../../bsp/delay/delay.h) — 延时等待

## 待实测

| 项目 | 当前值 | 说明 |
|------|--------|------|
| `o_dir` | 0 (CW) | 回零方向，需上电试：0=下转碰地，否则改为1 |
| `GIMBAL_MAX_ANGLE_DEG` | 45° | 根据线长和机械限位调整 |
| 回零等待时间 | 3000ms | 可能过长，实测后可缩短 |
| `GIMBAL_ZERO_CUR_MA` | 500mA | 太小可能误触发，太大可能撞坏 |
| `Gimbal_IsIdle()` | — | 需实现 ZDT 返回数据解析 |
