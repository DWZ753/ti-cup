# Protocol 通信协议模块

## 概述

基于 **COBS（Consistent Overhead Byte Stuffing）帧分隔 + XOR 校验** 的二进制通信协议。只提供传输层 API，不定义具体命令字——命令码由各应用层自行定义。

**设计目标：** 低开销、自动帧同步、MCU 侧零解析成本。

## 线上帧格式

```
 [   COBS 编码后载荷    ]
 [cmd + payload + checksum]  ──COBS编码──→  [ ... ] [0x00] 帧尾
```

| 层级 | 内容 | 说明 |
|------|------|------|
| 裸帧 | `[cmd(1B)] [payload(0~N B)] [checksum(1B)]` | 校验 = 全帧字节 XOR，结果应为 0 |
| 线上 | `COBS(裸帧) + 0x00` | 0x00 作帧尾，数据中绝不出 0x00 |

**COBS 的作用：** 把数据中出现的 0x00 全部替换掉，保证 0x00 只出现在帧尾。失步时遇到下一个 0x00 即自动重同步，不需要从头扫描。

**示例：** 发送命令 `0x01` + payload `"OK"`：

```
裸帧:  [01][4F 4B][65]   (cmd=0x01, payload="OK", checksum=0x01^0x4F^0x4B=0x65)
COBS:  [04 01 4F 4B 65]  (开销字节 04 = 后面 3 字节不含零 + 1)
线上:  [04 01 4F 4B 65 00] (末尾追加帧尾 0x00)
```

## API

```c
// ===== protocol.h =====

/* 回调：收到完整帧时触发（在 Protocol_Update 内同步调用） */
typedef void (*Protocol_RxCallback)(uint8_t cmd,
                                    const uint8_t *payload, uint8_t len);

void Protocol_Init(Protocol_RxCallback rx_cb, UART_Handle *uart);
void Protocol_Update(void);
void Protocol_SendFrame(uint8_t cmd, const uint8_t *payload, uint8_t len);
```

| 函数 | 调用位置 | 说明 |
|------|---------|------|
| `Protocol_Init()` | `main()` 初始化阶段 | 绑定回调 + UART，内部调用 `UART_StartReceiveRaw` |
| `Protocol_Update()` | `while(1)` 每圈 | 从 UART 环形缓冲读字节 → 拼帧 → COBS 解码 → 校验 → 回调 |
| `Protocol_SendFrame()` | 任意位置 | COBS 编码 + 计算校验 + `UART_SendData` 发出 |

## 用法示例

```c
#include "protocol.h"

/* ---- 1. 定义应用层命令码 ---- */
#define CMD_MY_ACTION  0x10  /* payload: 自定义格式 */

/* ---- 2. 定义收到帧时的回调 ---- */
static void on_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd)
    {
    case CMD_MY_ACTION:
        /* 处理 payload ... */
        break;

    default:
        break;
    }
}

/* ---- 3. 初始化 ---- */
int main(void)
{
    SYSCFG_DL_init();
    Board_Init();

    Protocol_Init(on_frame, Board_GetUART_PI());

    while (1)
    {
        Protocol_Update();   // 收帧
        // ... 其他非阻塞逻辑 ...
    }
}

/* ---- 4. 发帧 ---- */
void send_data(void)
{
    uint8_t buf[] = {0x01, 0x02};
    Protocol_SendFrame(CMD_MY_ACTION, buf, sizeof(buf));
}
```

## 增加新命令

命令码在**应用层**定义（`main.c` 或应用头文件中），不在 `protocol.h` 中。

```c
/* 应用层定义 */
#define CMD_OLED_SHOW   0x13  /* 第1字节=行号, 后续=文本 */
```

在回调的 `switch` 中追加对应 `case` 即可。

## 设计约束

| 项目 | 限制 | 说明 |
|------|------|------|
| 单帧最大载荷 | 64 字节 | 由 `PROTOCOL_MAX_PAYLOAD` 控制 |
| 帧分隔符 | `0x00` | COBS 保证载荷中不出现此值 |
| 校验 | XOR（单字节） | 检测单比特翻转，不检测多比特错误 |
| 失步恢复 | 自动 | 遇下一个 `0x00` 重同步 |
| 回调上下文 | ISR 外、同步调用 | 在 `Protocol_Update()` 中触发，可安全调用其他模块 |
| 线程安全 | 单线程 | 无锁，不可在 ISR 和主循环间共享 |

## 常见问题

**Q: 为什么不是 Modbus / protobuf / 其他标准协议？**
A: Modbus 偏寄存器模型，不适合指令交互；protobuf 需要代码生成，电赛场景体量过大。COBS + XOR 全网最少代码实现帧分隔 + 校验。

**Q: XOR 校验够不够？**
A: 电赛场地串口线 < 30cm，误码率极低。XOR 检测单比特翻转足以覆盖 99% 的实际问题。如需更强保护，改用 CRC8（1 字节）即可，不影响帧格式。

**Q: 缓冲够不够？**
A: 最大载荷 64 字节，COBS 编码后最大约 66 字节。内部缓冲按公式预留 `payload + payload/254 + 2 + 帧尾`，足够。

## 文件清单

```
modules/protocol/
├── cobs.h          # COBS API
├── cobs.c          # COBS 实现 (~60 行)
├── protocol.h      # 传输层 API
├── protocol.c      # 帧解析 + 校验 (~85 行)
└── protocol.md     # 本文档
```
