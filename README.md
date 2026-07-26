# TI Cup 2024 — 智能小车控制平台

基于 TI MSPM0G3507 (ARM Cortex-M0+) 的竞赛机器人小车项目，集成 IMU 姿态解算、双电机 PID 速度闭环、灰度循迹、OLED 显示及 UART 遥测。

## 硬件平台

| 组件 | 型号 | 接口 |
|---|---|---|
| MCU | MSPM0G3507 (Cortex-M0+, 80MHz) | — |
| IMU | BMI088 (六轴) | SPI |
| 电机驱动 | TB6612 (双 H 桥) | GPIO + PWM |
| 编码器 | 双电机霍尔编码器 | GPIO 定时器捕获 |
| 舵机 | 模拟/数字舵机 | PWM |
| 灰度传感器 | 8 路巡线模块 | GPIO 并行输入 |
| OLED | SSD1306 128×64 | I2C |
| 蜂鸣器 | 有源蜂鸣器 | GPIO |

## 软件架构

```
application/          # 上层算法（PID 控制、状态机、循迹解算）
modules/              # 功能模块（电机、舵机、IMU、OLED、按键、蜂鸣器、定时器）
bsp/                  # 外设抽象层（SPI、I2C、UART、Delay）
```

**模块自注册模式**：各模块在 `_Init()` 中自行调用 BSP 注册外设实例，不依赖 main.c 传入句柄。

**定时器回调系统**：

| 模块 | 定时器 | 周期 | 用途 |
|---|---|---|---|
| `pit_fast_tick` | TIMG12 | 1ms | 轻量快速任务（IMU 滴答、蜂鸣器计时） |
| `pit_control_tick` | TIMG0 | 20ms | 控制任务（电机速度闭环、按键扫描） |

每个 tick 支持最多 8 个回调，ISR 中按注册顺序执行。详见 [PIT Tick 模块](modules/pit_tick/pit_tick.md)。

## 📚 模块文档

### 系统级

| 文档 | 说明 |
|------|------|
| [main.md](main.md) | 主程序入口，初始化流程与主循环调度 |
| [board.md](board.md) | 板级初始化，系统滴答时钟，全局资源管理 |

### BSP 层（外设抽象）

| 模块 | 文档 | 说明 |
|------|------|------|
| Delay | [bsp/delay/delay.md](bsp/delay/delay.md) | 毫秒级阻塞延时 |
| I2C | [bsp/i2c/i2c.md](bsp/i2c/i2c.md) | I2C 主机通信 + SDA 总线解锁 |
| SPI | [bsp/spi/spi.md](bsp/spi/spi.md) | SPI 主机通信（全双工） |
| UART | [bsp/uart/uart.md](bsp/uart/uart.md) | 串口通信（阻塞/DMA 发送 + 中断接收） |

### Modules 层（功能模块）

| 模块 | 文档 | 说明 |
|------|------|------|
| Buzzer | [modules/buzzer/buzzer.md](modules/buzzer/buzzer.md) | 有源蜂鸣器（定时自动停止） |
| Grayscale | [modules/grayscale/grayscale.md](modules/grayscale/grayscale.md) | 8 路灰度传感器读取 |
| IMU | [modules/imu/imu.md](modules/imu/imu.md) | BMI088 + Mahony 姿态解算 |
| Key | [modules/key/key.md](modules/key/key.md) | 按键输入（FSM 消抖 + 回调） |
| Motor | [modules/motor/motor.md](modules/motor/motor.md) | 双电机控制（速度接口 + 编码器测速） |
| OLED | [modules/oled/oled.md](modules/oled/oled.md) | SSD1306 OLED 显示（字符/汉字/图片） |
| PIT Tick | [modules/pit_tick/pit_tick.md](modules/pit_tick/pit_tick.md) | 1ms/20ms 定时器回调系统 |
| Servo | [modules/servo/servo.md](modules/servo/servo.md) | 舵机 PWM 控制 |
| TB6612 | [modules/tb6612/tb6612.md](modules/tb6612/tb6612.md) | TB6612 双 H 桥底层驱动 |

### Application 层（控制算法）

| 模块 | 文档 | 说明 |
|------|------|------|
| PID | [application/pid/pid.md](application/pid/pid.md) | 位置式 PID 控制器（积分/输出双限幅） |
| State Machine | [application/state_machine/state_machine.md](application/state_machine/state_machine.md) | 按键触发任务状态切换 |
| Tracking | [application/tracking/tracking.md](application/tracking/tracking.md) | 8 路灰度加权插值循迹算法 |

## VSCode 配置

用 VSCode 打开本项目可获得代码高亮和 IntelliSense 补全。由于 SDK 和编译器路径每人不同，需要手动配置：

1. 在项目根目录创建 `.vscode/c_cpp_properties.json`，内容如下：

```json
{
    "configurations": [
        {
            "name": "MSPM0G3507_Config",
            "includePath": [
                "你的SDK路径/mspm0_sdk_2_10_00_04/source",
                "你的SDK路径/mspm0_sdk_2_10_00_04/source/third_party/CMSIS/Core/Include",
                "你的CCS路径/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/include",
                "${workspaceFolder}",
                "${workspaceFolder}/Debug",
                "${workspaceFolder}/**"
            ],
            "defines": [
                "__MSPM0G3507__"
            ],
            "compilerPath": "你的CCS路径/ccs/tools/compiler/ti-cgt-armllvm_4.0.4.LTS/bin/tiarmclang.exe",
            "cStandard": "c11",
            "cppStandard": "c++11",
            "intelliSenseMode": "windows-clang-arm"
        }
    ],
    "version": 4
}
```

2. 把"你的SDK路径"和"你的CCS路径"替换为实际安装路径。例如默认安装：
   - SDK：`D:/ti/ccs2050/mspm0_sdk_2_10_00_04`
   - CCS：`D:/ti/ccs2050/ccs`

3. 如需排除 VSCode 配置文件，可在 `.gitignore` 中保留 `.vscode/` 规则

## 构建与烧录

本项目使用 **CCS Theia**（TI Eclipse IDE），无 Makefile/CMake。

1. 用 CCS Theia 打开项目目录
2. 运行 SysConfig（`empty.syscfg`）生成 `ti_msp_dl_config.c/.h`
3. 点击 Build（Ctrl+B）编译，输出 `Debug/ti_cup_2024h.out`
4. 通过 J-Link 烧录到 LP-MSPM0G3507 LaunchPad

**关键依赖**：

- **编译器**: `tiarmclang` 4.0.4.LTS
- **SDK**: MSPM0-SDK 2.10.0.04
- **SysConfig**: 1.26.2
- **优化**: `-O0`（Debug），软浮点 ABI（Cortex-M0+ 无硬件 FPU）

## 引脚分配

见 `empty.syscfg`（SysConfig 图形化配置），主要外设：

| 外设 | 实例 | 关键引脚 |
|---|---|---|
| SPI (BMI088) | SPI0 | PB13 (ACCEL_CS), PB15 (GYRO_CS) |
| I2C (OLED) | I2C0 | PA11 (SDA), PA10 (SCL) |
| UART | UART0 | PA9 (TX), PA8 (RX) |
| PWM (电机) | TIMG2/6 | PB4/PB7 (M1), PA24/PA25 (M2) |
| PWM (舵机) | TIMG4 | PB22 |
| GPIO (灰度) | GPIOB | PB8–PB14 (IN1–IN7), PB0 (IN8) |
| GPIO (按键) | GPIOA | PA26 (KEY1), PA27 (KEY2), PA28 (KEY3) |

## 已知问题

- **Yaw 漂移**：Mahony 滤波器仅用加速度计修正 Roll/Pitch，Yaw 角随时间漂移（无磁力计）
- **Cortex-M0+ 无硬件 FPU**：浮点运算为软件模拟，IMU_Update 含大量浮点运算，注意控制频率
