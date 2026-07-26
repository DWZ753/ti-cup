# Protocol 通信协议模块

## 概述

基于 **COBS（Consistent Overhead Byte Stuffing）帧分隔 + XOR 校验** 的二进制通信协议，用于 MSPM0 与树莓派之间的指令交互和传感器数据上报。

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

**示例：** 发送 `CMD_STATUS` + `"OK"`：

```
裸帧:  [2F][4F 4B][65]   (cmd=0x2F, payload="OK", checksum=0x2F^0x4F^0x4B=0x65)
COBS:  [04 2F 4F 4B 65]   (开销字节 04 = 后面 3 字节不含零 + 1)
线上:  [04 2F 4F 4B 65 00] (末尾追加帧尾 0x00)
```

## 命令字一览

### RPi → MCU（控制指令）

| 宏 | 值 | 载荷 | 说明 |
|----|-----|------|------|
| `CMD_LINE_OFFSET` | `0x10` | `int16 dx` (mm) | 线偏差（预留） |
| `CMD_TARGET_ANGLE` | `0x11` | `int16 angle` (0.01°) | 目标角度（预留） |
| `CMD_SPEED_CMD` | `0x12` | `int16 vL, int16 vR` (mm/s) | 左右轮线速度 |
| `CMD_EMERGENCY_STOP` | `0x1F` | 无 | 急停 |

### MCU → RPi（传感器上报）

| 宏 | 值 | 载荷 | 说明 |
|----|-----|------|------|
| `CMD_IMU_DATA` | `0x20` | `int16 roll, pitch, yaw` (0.01°) | IMU 姿态 |
| `CMD_ENCODER_DATA` | `0x21` | `int16 left_rpm, right_rpm` | 编码器转速（预留） |
| `CMD_ULTRASONIC` | `0x22` | `int16 dist_mm` | 超声波距离 |
| `CMD_STATUS` | `0x2F` | 字符串 | 状态/启动通知 |
| `CMD_ERROR` | `0xFF` | 字符串 | 错误消息 |

### 数值编码约定

- 所有多字节整数为**大端序**（网络字节序），通过 `put_i16()` / `get_i16()` 读写
- 角度单位 `0.01°`（例如 `9000` = 90.00°）
- 距离单位 `mm`

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
#include "motor.h"
#include "imu.h"

/* ---- 1. 定义收到帧时的回调 ---- */
static void on_frame(uint8_t cmd, const uint8_t *payload, uint8_t len)
{
    switch (cmd)
    {
    case CMD_SPEED_CMD:
        if (len == 4)
        {
            int16_t vL = get_i16(&payload[0]);
            int16_t vR = get_i16(&payload[2]);
            Motor_SetSpeedLR((float)vL, (float)vR);
        }
        break;

    case CMD_EMERGENCY_STOP:
        Motor_Brake();
        break;

    default:
        Protocol_SendFrame(CMD_ERROR, (const uint8_t *)"bad cmd", 7);
        break;
    }
}

/* ---- 2. 初始化 ---- */
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

/* ---- 3. 发帧 ---- */
void send_imu(float roll, float pitch, float yaw)
{
    uint8_t buf[6];
    int16_t r = (int16_t)(roll  * 100.0f);
    int16_t p = (int16_t)(pitch * 100.0f);
    int16_t y = (int16_t)(yaw   * 100.0f);

    put_i16(&buf[0], r);
    put_i16(&buf[2], p);
    put_i16(&buf[4], y);
    Protocol_SendFrame(CMD_IMU_DATA, buf, 6);
}
```

## 增加新命令

以增加一个"OLED 显示文本"命令为例，RPi 可以远程让 MCU 在 OLED 上显示文字。

### 步骤 1：定义命令字 (`protocol.h`)

```c
/* 在 "RPi → MCU" 区域找个未用的值 */
#define CMD_OLED_SHOW   0x13  /* 字符串：第1字节=行号, 后续=文本 */
```

**取值规则：**
- `0x10~0x1F`：RPi → MCU 控制指令
- `0x20~0x2F`：MCU → RPi 传感器上报
- 避免与已有的冲突，保持编号紧凑

### 步骤 2：实现回调逻辑 (`main.c`)

```c
/* 在 on_frame() 的 switch 中追加 */
case CMD_OLED_SHOW:
    if (len >= 2)
    {
        uint8_t line    = payload[0];
        uint8_t text[17] = {0};                         // OLED 一行最多 16 字符
        uint8_t copy_len = len - 1;
        if (copy_len > 16) copy_len = 16;
        memcpy(text, &payload[1], copy_len);
        OLED_ShowString(0, line, text, 16);
    }
    break;
```

别忘了在 `main.c` 顶部 `#include "oled.h"`。

### 步骤 3：Python 侧发送 (`tools/protocol_test.py`)

```python
# 在 CMD_NAMES 字典中注册名称（可选，方便调试）
CMD_OLED_SHOW = 0x13
CMD_NAMES[CMD_OLED_SHOW] = "OLED_SHOW"

# 发送
ser.write(build_frame(CMD_OLED_SHOW, b'\x02' + b'Hello RPi'))
```

### 步骤 4（可选）：MCU 侧回复确认

```c
/* 处理完命令后回复 ACK */
Protocol_SendFrame(CMD_STATUS, (const uint8_t *)"OLED OK", 7);
```

## 设计约束

| 项目 | 限制 | 说明 |
|------|------|------|
| 单帧最大载荷 | 32 字节 | 由 `PROTOCOL_MAX_PAYLOAD` 控制 |
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

**Q: 64 字节缓冲够不够？**
A: 最长的有效帧（IMU 6 字节 payload）COBS 编码后 ~9 字节。32 字节 MAX_PAYLOAD 编码后 ~34 字节。缓冲 2× 余量，足够。

**Q: Python 侧需要装什么？**
A: 仅 `pyserial`（`pip install pyserial`）。COBS 和帧解析已在脚本内实现，零额外依赖。

## 文件清单

```
modules/protocol/
├── cobs.h          # COBS API
├── cobs.c          # COBS 实现 (~60 行)
├── protocol.h      # 协议 API + 命令字
├── protocol.c      # 帧解析 + 校验 (~85 行)
└── protocol.md     # 本文档

tools/
└── protocol_test.py  # Python 测试脚本
```
