# 树莓派侧开发提示词

将此文件内容提供给负责树莓派视觉+决策开发的 AI 助手。

---

## 你的任务

为 2026 电赛"智能物流搬运系统"的树莓派端编写控制程序。

**已有代码：** 项目 `E:\ti\lib\raspberrypi\projects\2026-sim-car\main.py` 实现了钢球视觉检测（HoughCircles + 轮廓 fallback），检测到钢球后在画面上绘制标记。当前 **没有串口通信**——检测结果只显示在推流画面上。

**你需要实现：** 在现有视觉检测基础上，添加串口通信层和搬运决策状态机。

---

## 硬件架构

```
  USB摄像头 (640x480)
        │
    树莓派  ←─ UART ─→  MSPM0 小车
        │                  │
   视觉检测+决策         电机/舵机/云台/电磁铁
```

---

## MSPM0 通信协议

### 传输层

- 物理接口：UART，115200-8-N-1
- Python 库：`pyserial`
- 帧格式：**COBS（Consistent Overhead Byte Stuffing）+ XOR 校验**

### COBS 编码规则

帧分隔符为 `0x00`。编码过程将数据中所有 `0x00` 替换为"跳转标记"，保证 `0x00` 只出现在帧尾。

Python 实现：
```python
def cobs_encode(data: bytes) -> bytes:
    """COBS 编码，返回不含末尾 0x00 的编码数据"""
    result = bytearray(len(data) + 2)
    code_idx = 0
    code_val = 1
    for i, b in enumerate(data):
        if b == 0:
            result[code_idx] = code_val
            code_idx = i + 1
            code_val = 1
        else:
            result[i + 1] = b
            code_val += 1
            if code_val == 255:
                result[code_idx] = code_val
                code_idx = i + 1
                code_val = 1
    result[code_idx] = code_val
    return bytes(result[:len(data) + 1])

def cobs_decode(data: bytes) -> bytes:
    """COBS 解码"""
    result = bytearray()
    i = 0
    while i < len(data) - 1:
        code = data[i]
        if code == 0:
            break
        i += 1
        for _ in range(code - 1):
            if i >= len(data):
                break
            result.append(data[i])
            i += 1
        if code < 255 and i < len(data):
            result.append(0)
    return bytes(result)
```

### 帧封装

```python
def make_frame(cmd: int, payload: bytes) -> bytes:
    """构造一个完整帧：cmd(1B) + payload + XOR_checksum(1B) → COBS → 追加 0x00"""
    raw = bytes([cmd]) + payload
    checksum = 0
    for b in raw:
        checksum ^= b
    raw += bytes([checksum])
    encoded = cobs_encode(raw)
    return encoded + b'\x00'

def parse_frame(data: bytes) -> tuple[int, bytes] | None:
    """解析接收到的帧，返回 (cmd, payload) 或 None"""
    decoded = cobs_decode(data)
    if len(decoded) < 3:
        return None
    # 校验
    xor = 0
    for b in decoded:
        xor ^= b
    if xor != 0:
        return None
    cmd = decoded[0]
    payload = decoded[1:-1]  # 去掉 checksum
    return cmd, payload
```

### 发送/接收

```python
import serial

ser = serial.Serial('/dev/ttyAMA0', 115200, timeout=0.01)

def send_frame(cmd: int, payload: bytes = b''):
    frame = make_frame(cmd, payload)
    ser.write(frame)

def recv_frames() -> list[tuple[int, bytes]]:
    """读取所有完整帧（非阻塞）"""
    frames = []
    while True:
        # 需要实现一个环形缓冲 + 0x00 分隔的逻辑
        # 或使用已有的帧接收实现
        ...
    return frames
```

---

## Pi → MSPM0 命令

| 命令 | 值 | Payload | 用途 |
|------|-----|---------|------|
| CMD_GUIDE | 0x20 | speed(int8), diff(int8) | 锁定球后持续引导 |
| CMD_STOP | 0x21 | 无 | 丢锁/误检/到位 |
| CMD_MAGNET_ON | 0x22 | 无 | 电磁铁吸合 |
| CMD_MAGNET_OFF | 0x23 | 无 | 电磁铁释放 |
| CMD_GIMBAL | 0x24 | angle_deg(int16, 大端) | 云台 pitch |
| CMD_RESUME_LINE | 0x2F | 无 | 回到循迹模式 |

**CMD_GUIDE 参数说明：**
- `speed`: -100~+100，前进速度百分比（负=后退）
- `diff`: -100~+100，左右差速（负=左转, 正=右转）
- MSPM0 侧执行: `left = speed + diff, right = speed - diff`

**CMD_GIMBAL 参数说明：**
- `angle_deg`: 云台 pitch 角度，0=机械臂触地，正值=向上抬起
- 有效范围：[0, 45°]
- 大端序：`struct.pack('>h', angle_deg)`

---

## MSPM0 → Pi 反馈

| 命令 | 值 | Payload | 含义 |
|------|-----|---------|------|
| CMD_GIMBAL_OK | 0x30 | 无 | 云台到位 |
| CMD_FAULT | 0x3F | error_code(uint8) | 故障 |

故障码：1=串口超时, 2=云台过流, 3=云台堵转

---

## 视觉确认策略

### 锁定（避免误检）：连续 5 帧确认

```python
LOCK_FRAMES = 5
candidate = None
candidate_count = 0

for frame in camera:
    balls = detect_balls(frame)  # 已有的检测函数

    best = find_best_ball(balls)  # 选最近/最中心的球

    if best is not None and is_same_ball(best, candidate, threshold=30):
        candidate_count += 1
        if candidate_count >= LOCK_FRAMES:
            locked = True
            target_ball = best
    else:
        candidate = best
        candidate_count = 1
```

### 丢锁（避免跟丢）：连续 3 帧

```python
LOST_THRESHOLD = 3
lost_count = 0

# 在锁定引导循环中
if best is None:
    lost_count += 1
    if lost_count >= LOST_THRESHOLD:
        send_frame(CMD_STOP)
        # 进入恢复逻辑
else:
    lost_count = 0
```

### 接近判断

根据球的大小（半径 r 像素数）判断距离：
- r < 15: 很远 → speed=50
- r 15~30: 中等 → speed=30
- r > 30: 很近 → speed=10, 准备吸取
- r > 40: 几乎在下方 → 停车, 吸取

### 超时保护

```python
import time
start_time = time.time()
APPROACH_TIMEOUT = 5.0  # 5秒

while approaching:
    if time.time() - start_time > APPROACH_TIMEOUT:
        # 超时：可能是误检或球不可达
        send_frame(CMD_STOP)
        # 放弃当前球，搜索下一个
        break
```

---

## 控制律：球的画面坐标 → speed/diff

摄像头 640×480，画面中心 (320, 240) 为"正前方"。

```python
CW, CH = 640, 480
CX, CY = CW // 2, CH // 2  # 320, 240

def ball_to_guide(ball_cx, ball_cy, ball_r):
    """球的画面坐标 → speed/diff"""
    # 水平偏差 → 差速修正
    x_err = ball_cx - CX  # 正=球在右边
    diff = int(x_err / CX * 100)  # 归一化到 [-100, 100]
    diff = max(-80, min(80, diff))  # 限幅

    # 距离估计 → 速度
    if ball_r > 35:
        speed = 0    # 已到，准备吸取
    elif ball_r > 25:
        speed = 15   # 很近
    elif ball_r > 15:
        speed = 30   # 中等
    else:
        speed = 50   # 远

    return speed, diff
```

---

## 任务状态机

```
IDLE ──(检测到球,5帧确认)──> APPROACHING
                                  │
                    ┌─────────────┼─────────────┐
                    │             │             │
                丢锁3帧      球够近(r>35)    超过5秒
                    │             │             │
                    ▼             ▼             ▼
                RECOVERING     PICKUP       GIVE_UP
                    │             │             │
              2秒内恢复    吸取完成        重搜下个球
                    │             │             │
                    ▼             ▼             │
              APPROACHING   RETURN_LINE <───────┘

RETURN_LINE ──(引导车回循迹线)──> 发 CMD_RESUME_LINE → FOLLOW_LINE

FOLLOW_LINE ──(到轨道入口)──> DROP_BALL ──(释放)──> 下一个球 → IDLE
                                                       (全部完成) → DONE
```

---

## 注意事项

1. **Pi 负责所有决策**，MSPM0 只执行指令。不要试图让 MSPM0 自己判断位置。
2. **CMD_GUIDE 频率**：建议 50ms 发一次（20Hz）。太快 MSPM0 来不及处理，太慢控制不稳定。
3. **CMD_STOP 后不要立即发 CMD_GUIDE**：等 50ms 让车完全停下。
4. **云台控制**：发 CMD_GIMBAL 后等 CMD_GIMBAL_OK 再发下一条（或超时重试）。
5. **机械臂遮挡**：画面中始终能看到机械臂，检测球时注意屏蔽掉机械臂区域（当前代码已屏蔽顶部 20%）。
6. **光照变化**：比赛场地光照可能不均匀，HoughCircles 参数可能需要现场调整。
7. **UART 超时**：如果连续 500ms 没收到任何帧反馈，可能串口断开，应尝试重连或报警。
