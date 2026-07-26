# UART 串口模块

## 概述

UART 串口通信模块，基于句柄注册机制支持最多 4 路 UART 实例（UART0~UART3）。提供两套收发模式：

- **字符串模式**：行缓冲接收（遇 `\n` 置完成标志），阻塞/DMA 发送，适合 printf 调试
- **原始模式**：环形缓冲持续接收，阻塞字节数组发送，适合二进制协议

每个实例独立配置，互不干扰。ISR 自动分发到已注册的句柄。

## 文件

| 文件 | 说明 |
|------|------|
| `uart.h` | 模块头文件，硬件映射宏、类型定义与 API 声明 |
| `uart.c` | 模块实现 |

## API

### 实例管理

#### `UART_Handle* UART_Init(const UART_Config *config)`

注册并初始化一个 UART 实例。

| 参数 | 说明 |
|------|------|
| `config->uart` | UART 外设基址（`UART0` ~ `UART3`） |
| `config->irqNum` | NVIC 中断号 |
| `config->dmaTxChanId` | DMA TX 通道号 |
| `config->dmaTxTrigger` | DMA 发送触发源 |

返回句柄指针，失败（重复注册 / 已满）返回 `NULL`。调用后 NVIC 自动使能。

### 字符串发送

| 函数 | 说明 |
|------|------|
| `UART_SendStr(h, str)` | 阻塞发送字符串 |
| `UART_Printf(h, fmt, ...)` | 阻塞格式化输出（内部 vsprintf + SendStr，缓冲 256 字节） |
| `UART_SendStrDMA(h, str, len)` | DMA 发送（非阻塞），等待 `h->txDMADone == 1` 确认完成 |
| `UART_PrintfDMA(h, fmt, ...)` | DMA 格式化输出（非阻塞） |

### 二进制发送

| 函数 | 说明 |
|------|------|
| `UART_SendByte(h, data)` | 阻塞发送单字节 |
| `UART_SendData(h, data, len)` | 阻塞发送字节数组 |

### 行模式接收

#### `void UART_StartReceive(UART_Handle *h)`

启动行模式接收。ISR 收到 `\n` 后置 `h->rxDone = 1`，数据在 `h->rxBuf` 中（含结束符）。主循环轮询 `h->rxDone` 即可。

### 原始模式接收（二进制协议用）

| 函数 | 说明 |
|------|------|
| `UART_StartReceiveRaw(h)` | 切换到原始模式，ISR 持续写入环形缓冲 |
| `UART_RawRxAvailable(h)` | 可读字节数 |
| `UART_ReadRawByte(h)` | 读取一个字节（空时返回 0） |
| `UART_FlushRawRx(h)` | 清空环形缓冲 |

## 依赖

- `ti_msp_dl_config.h`（SysConfig 生成的 UART/DMA 配置宏）

## 使用示例

### 字符串模式（调试输出）

```c
#include "uart.h"
#include "board.h"

UART_Handle *uart = Board_GetUART();

UART_Printf(uart, "Hello %d\r\n", 42);

// 行模式接收
UART_StartReceive(uart);
while (1) {
    if (uart->rxDone) {
        UART_Printf(uart, "Echo: %s\r\n", uart->rxBuf);
        UART_StartReceive(uart);
    }
}
```

### 原始模式（二进制协议）

```c
UART_Handle *uart = Board_GetUART_PI();
UART_StartReceiveRaw(uart);

while (1) {
    while (UART_RawRxAvailable(uart) > 0) {
        uint8_t b = UART_ReadRawByte(uart);
        // 处理字节...
    }
}

// 发送二进制帧
uint8_t frame[] = {0xAA, 0x55, 0x01, 0x02};
UART_SendData(uart, frame, sizeof(frame));
```
