# TI Cup 2026 模拟 — 智能物流搬运系统

双 MCU 协作系统：**移动搬运小车** + **无线计分显示装置**

## 项目结构（双 CCS 项目）

```
ti_cup/                       ← Git 仓库根目录
├── car/                      ← CCS 项目1: 移动搬运小车
│   ├── .cproject             ← 小车项目配置
│   ├── empty.syscfg          ← 小车引脚配置（SysConfig）
│   ├── main.c                ← 小车主逻辑
│   ├── board.c / board.h     ← 小车板级初始化
│   └── application/          ← 小车控制算法
├── display/                  ← CCS 项目2: 计分显示装置
│   ├── .cproject             ← 显示装置项目配置
│   ├── empty.syscfg          ← 显示装置引脚配置
│   ├── main.c                ← 显示装置主逻辑
│   ├── board.c / board.h     ← 显示装置板级初始化
│   └── application/          ← 显示装置控制
├── bsp/                      ← 共享 BSP 层（外设抽象）
├── modules/                  ← 共享功能模块
├── shared/                   ← 两 MCU 共享协议定义
│   └── protocol.h
└── questions/                ← 赛题文档
```

## 赛题要求

见 [questions/26模拟/模拟赛题.md](questions/26模拟/模拟赛题.md)

### 系统组成

| 装置 | MCU | 功能 |
|------|-----|------|
| 移动搬运小车 | MSPM0G3507 | 循迹、视觉识别、电磁铁控制、航向保持、电机驱动 |
| 计分显示装置 | MSPM0G3507 | 按键输入、数码管显示、LED 指示、蜂鸣器、无线通信 |
| 视频监控终端 | 外设（≥6"屏） | 显示小车摄像头画面 + 钢球标注 |

### 通信

小车 ↔ 计分显示装置 通过短距离无线通信模块交换指令和状态，协议定义见 [shared/protocol.h](shared/protocol.h)。

## 在 CCS Theia 中打开项目

本仓库包含**两个独立 CCS 项目**，需分别导入：

1. 用 CCS Theia 打开 `car/` 目录 → 小车项目（`ti_cup_car`）
2. 用 CCS Theia 打开 `display/` 目录 → 显示装置项目（`ti_cup_display`）

两个项目共享根目录的 `bsp/`、`modules/` 和 `shared/`，通过 `.cproject` 中的 `../` 相对路径引用。

## 硬件平台

| 组件 | 型号 | 接口 |
|------|------|------|
| MCU | MSPM0G3507 (Cortex-M0+, 80MHz) | — |

## 构建

- **IDE**: CCS Theia
- **编译器**: tiarmclang 4.0.4.LTS
- **SDK**: MSPM0-SDK 2.10.0.04
- **SysConfig**: 1.26.2
- **调试器**: SEGGER J-Link

1. 打开项目 → 运行 SysConfig → Build → 烧录
