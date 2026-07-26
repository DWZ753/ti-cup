# TI Cup 2025E — 双轴云台控制平台

基于 TI MSPM0G3507 (ARM Cortex-M0+) 的二维云台控制项目。双环 PID 角度伺服，ZDT 闭环步进电机驱动，PID Helper 串口联调。

## 硬件

| 组件 | 型号 | 接口 |
|------|------|------|
| MCU | MSPM0G3507 (Cortex-M0+, 80MHz) | — |
| 云台 Roll | ZDT X42S 闭环步进电机 | RS485, ID=1 |
| 云台 Yaw | ZDT X42S 闭环步进电机 | RS485, ID=2 |
| IMU | IMU660RA 六轴姿态模块 | SPI0 |

### 接线

```
MSPM0G3507                     RS485 总线
  PA0 (TX) ──┬── 云台1 (Roll, ID=1)
  PA1 (RX) ──┤
             ├── 云台2 (Yaw, ID=2)
             └── GND（共地）

电机供电：12-24V 独立电源（勿从 LaunchPad 取电）
```

两电机共享一路 RS485（115200-8-N-1），通过 ID 寻址。驱动板自带 RS485 自动方向控制。

## 软件架构

```
application/          ← 控制算法
  pid/                ← 共享 PID 算法库
  pid_helper/         ← PID Helper 串口联调（通用）
  gimbal/             ← 云台控制子系统（gimbal + gimbal_test）
  chassis/            ← 底盘控制子系统（chassis + tracking + angle）
modules/              ← 功能模块（zdt_motor, imu, oled, motor, ...）
bsp/                  ← 外设抽象（uart, spi, i2c, delay）
board.c / board.h     ← 板级初始化 + SysTick 滴答
empty.c               ← 主程序入口
```

三层结构，上层依赖下层。模块自注册：`_Init()` 内部调用 BSP 注册外设，不从 main.c 传入句柄。

## 控制架构

```
target_angle → [外环 位置PID] → vel_cmd → [内环 速度PID] → Vel_Control()
                    ↑                            ↑
              read_angle()                read_velocity()
```

- 外环：角度误差走最短路径后进位置 PID
- 内环：速度误差进速度 PID → 电机输出
- PID 为模块内部静态变量（参照 Angle 模块模式），调参通过 `GimbalAxis_TunePosPID()` / `GimbalAxis_TuneVelPID()`

## 引脚分配

| 外设 | 实例 | 引脚 | 用途 |
|------|------|------|------|
| UART (RS485) | UART0 | PA0 (TX), PA1 (RX) | ZDT 电机 |
| UART (PID Helper) | UART1 | PA8 (TX), PA9 (RX) | 上位机联调 |
| SPI (IMU) | SPI0 | PA12 (SCLK), PA14 (MOSI), PA13 (MISO), PB15 (CS) | IMU660RA |

## 快速开始

1. CCS Theia 打开项目
2. SysConfig → Save → Build (Ctrl+B)
3. J-Link 烧录运行

上电初始化后进入双环 PID 控制循环，同时可通过 PID Helper 上位机实时调参。

## 编译环境

- **编译器**: tiarmclang 4.0.4.LTS
- **SDK**: MSPM0-SDK 2.11.00.07
- **SysConfig**: 1.26.2
- **优化**: `-O0`（Debug），软浮点 ABI（Cortex-M0+ 无硬件 FPU）
