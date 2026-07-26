# PID Helper 串口联调模块

## 概述

基于二进制帧协议的串口联调模块，实现下位机（MSPM0）与 [PID Helper](../../references/PID%20Helper.md) 桌面工具的实时通信。支持：

- **实时采样上传**：36 字节定长帧，含 timestamp + actual/target/input/error + Kp/Ki/Kd
- **命令下发**：17 字节定长控制帧，支持 PID 参数修改 / 模式切换 / 目标值设置
- **ACK 确认**：20 字节应答帧，每命令必回复
- **双环切换**：位置环（外环）与速度环（内环）可运行时切换
- **双轴选择**：编译时通过宏选择 Roll 或 Yaw 轴

## 文件

| 文件 | 说明 |
|------|------|
| `pid_helper.h` | 模块头文件，协议常量、API 声明 |
| `pid_helper.c` | 模块实现（帧解析、采样发送、命令处理） |

## 硬件连接

| MSPM0 引脚 | USB-TTL 模块 | 说明 |
|-----------|-------------|------|
| TX（SysConfig 中 PIDHELPER 分配的引脚） | RX | 下位机 → 上位机 |
| RX（SysConfig 中 PIDHELPER 分配的引脚） | TX | 上位机 → 下位机 |
| GND | GND | 共地 |

> 引脚以 `empty.syscfg` 中 PIDHELPER UART 实例的实际分配为准。波特率固定 115200-8-N-1。

## 协议帧格式

### 上行采样帧（MCU → PC）

```
[timestamp_ms: u32LE] [actual: f32LE] [target: f32LE] [input: f32LE]
[error: f32LE] [kp: f32LE] [ki: f32LE] [kd: f32LE] [00 00 80 7f]
```

36 字节，帧尾 `00 00 80 7f`（float +Inf 的位模式）。

### 上行 ACK 帧（MCU → PC）

```
[cmdId: f32LE] [status: f32LE] [value: f32LE] [seq: f32LE] [00 00 80 7f]
```

20 字节，status=1 成功，0 失败。

### 下行命令帧（PC → MCU）

```
AA 55 [cmd: u8] [seq: u8] [arg0: f32LE] [arg1: f32LE] [arg2: f32LE] [sum: u8]
```

17 字节，sum = 前 16 字节累加取低 8 位。

### 命令一览

| cmd | 命令 | 参数 |
|-----|------|------|
| 1 | PID | arg0=Kp, arg1=Ki, arg2=Kd |
| 2 | MODE | arg0=1.0→开环, 0.0→闭环 |
| 3 | OUT | arg0=开环输出值(0~1) |
| 4 | SP | arg0=目标值(setpoint) |
| 5 | APPMODE | arg0=0.0→速度环, 1.0→位置环 |

## API

### `void PH_Init(UART_Handle *uart, GimbalAxis *roll, GimbalAxis *yaw)`

初始化模块，绑定 UART 和两个云台轴。UART 需事先通过 `Board_PIDHelper_UART_Init()` 注册。

### `void PH_Process(void)`

处理接收缓冲。在主循环中周期性调用，内部解析 17 字节命令帧并自动分发。

### `void PH_SendSample(void)`

发送一帧采样数据。根据当前 APPMODE 自动选择数据源：

| APPMODE | target | actual | input | error | kp/ki/kd 来源 |
|---------|--------|--------|-------|-------|--------------|
| 位置环 | 目标角度 | 当前角度 | 速度指令 | pos_error | 位置环 PID |
| 速度环 | 速度指令 | 当前转速 | 电机输出 | vel_error | 速度环 PID |

### `void PH_SendAck(uint8_t cmd, uint8_t status, float value, uint8_t seq)`

发送 ACK 帧，内部命令处理自动调用，一般不需要手动调用。

### `void PH_SetActiveAxis(uint8_t axis_index)`

运行时切换活跃轴。`PH_DEFAULT_AXIS_ROLL`(0) 或 `PH_DEFAULT_AXIS_YAW`(1)。

## 在 empty.c 中的用法

```c
#include "board.h"

int main(void)
{
    GimbalAxis roll, yaw;
    UART_Handle *pid_uart;

    SYSCFG_DL_init();
    Board_Init();

    /* 初始化电机 */
    GimbalAxis_Init(&roll, GIMBAL_ROLL_ID);
    GimbalAxis_Init(&yaw,  GIMBAL_YAW_ID);
    GimbalAxis_Enable(&roll, true);
    GimbalAxis_Enable(&yaw,  true);

    /* 初始化 PID Helper */
    pid_uart = Board_PIDHelper_UART_Init();
    PH_Init(pid_uart, &roll, &yaw);

    while (1)
    {
        PH_Process();              /* 处理下发命令 */
        /* … 控制循环 … */
        PH_SendSample();           /* 上传采样数据 */
    }
}
```

## 调参流程

1. 编译烧录 → 打开 PID Helper 桌面工具 → 选择 COM 口 → 115200 → 打开串口
2. 选择应用背景（位置环 / 速度环）
3. 使用"自动辨识"获取 PID 初值
4. 使用"阶跃响应测试"观察闭环表现
5. 使用"闭环自动优化"搜索更优参数

### 切换调参轴

修改 `pid_helper.h` 顶部宏：

```c
#define PH_DEFAULT_AXIS  PH_DEFAULT_AXIS_ROLL   // Roll
// #define PH_DEFAULT_AXIS  PH_DEFAULT_AXIS_YAW  // Yaw
```

改后重新编译即可。

## 依赖

- `ti_msp_dl_config.h`（提供 `PIDHELPER_INST`、`PIDHELPER_INST_INT_IRQN`）
- [UART 模块](../../bsp/uart/uart.md)（bsp/uart 实例注册）
- [GimbalControl 模块](../gimbal/gimbal.h)（提供轴数据和 PID 读写接口）

## 编译控制

不需要 PID Helper 时，从 CCS 编译列表中排除 `pid_helper.c` 即可，不会影响电机控制功能。

## UART 配置（SysConfig）

在 `empty.syscfg` 中需配置第二个 UART 实例（PIDHELPER），参数：

- 外设：UART1（或任意空闲 UART）
- 波特率：115200
- 数据位：8，停止位：1，无校验
- RX 中断：启用

SysConfig 保存后生成 `PIDHELPER_INST` 等宏，供 `board.c` 中的 `Board_PIDHelper_UART_Init()` 使用。
