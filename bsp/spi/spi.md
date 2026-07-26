# SPI 通信模块

## 概述

SPI 主机通信模块，提供实例注册机制和单字节全双工传输功能。支持最多 2 路 SPI 外设，每路 SPI 可挂载多个从设备（通过独立 CS 引脚选择）。当前项目用于 BMI088 IMU 传感器通信。

## 文件

| 文件 | 说明 |
|------|------|
| `spi.h` | 模块头文件，类型定义与 API 声明 |
| `spi.c` | 模块实现 |

## 核心机制

### 实例注册表

模块内部维护一个**静态句柄池**（`g_handle_pool`），每个 SPI 外设（如 SPI0、SPI1）只能注册一次。调用 `SPI_Init()` 注册实例，返回句柄指针。

### 单字节全双工传输

`SPI_Transfer()` 发送一个字节的同时接收一个字节，实现标准的 SPI 全双工通信。发送完成后等待 RX FIFO 非空 → 读取接收数据 → 等待总线空闲 → 返回接收字节。

## 类型定义

### `SPI_Config` — 初始化配置结构体

```c
typedef struct {
    SPI_Regs *spi;   // SPI 外设基址（如 SPI0）
} SPI_Config;
```

### `SPI_Handle` — 运行时句柄

```c
typedef struct {
    SPI_Regs *spi;   // SPI 外设基址
} SPI_Handle;
```

## API

### `SPI_Handle* SPI_Init(const SPI_Config *config)`

注册并初始化一个 SPI 实例。

| 参数 | 说明 |
|------|------|
| `config` | 初始化配置结构体指针（当前仅需指定 SPI 外设基址） |

| 返回值 | 说明 |
|--------|------|
| 非 NULL | 句柄指针，注册成功 |
| NULL | 注册失败（重复注册 / 硬件索引无效 / 实例已满） |

### `uint8_t SPI_Transfer(SPI_Handle *h, uint8_t tx_data)`

单字节全双工 SPI 传输（阻塞式）。

| 参数 | 说明 |
|------|------|
| `h` | SPI 句柄指针 |
| `tx_data` | 待发送的 8 位数据 |

| 返回值 | 说明 |
|--------|------|
| `uint8_t` | 同时接收到的 8 位数据 |

**调用流程：** 发送数据 → 等待 RX FIFO 非空 → 读取接收数据 → 等待总线空闲 → 返回。

## 依赖

- `ti_msp_dl_config.h`（提供 SPI 外设驱动 API）

## 使用示例

```c
#include "spi.h"

// 1. 注册 SPI 实例
SPI_Config spi_cfg = { .spi = SPI0 };
SPI_Handle *spi = SPI_Init(&spi_cfg);

// 2. 读取传感器寄存器（标准 SPI 读操作：最高位 = 1 表示读）
//    发送寄存器地址（读命令），接收到的第一个字节通常为 dummy byte
SPI_Transfer(spi, 0x00 | 0x80);   // 发送读命令
uint8_t data = SPI_Transfer(spi, 0xFF);  // 发送 dummy，接收数据
```

## 实际应用（BMI088 传感器）

本项目中 SPI 模块被 BMI088 IMU 传感器使用，通过两路独立 CS 引脚（PB13 加速度计、PB15 陀螺仪）区分设备。参见 [IMU 模块](../../modules/imu/imu.md) 和 [BMI088 驱动](../../modules/imu/bmi088/bmi088.md)。
