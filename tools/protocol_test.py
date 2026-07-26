"""
树莓派侧协议测试脚本 — COBS 帧分隔 + XOR 校验

用法:
    python protocol_test.py              (使用默认串口)
    python protocol_test.py <串口>        (指定串口)
    python protocol_test.py <串口> --test (发送测试指令)

协议格式:
    线上: [COBS-encode(cmd + payload + checksum)] 0x00
    裸帧: [cmd(1B)] [payload(N B)] [checksum(1B, XOR)]
"""

import serial
import struct
import sys
import time
from dataclasses import dataclass

# ===== 在此修改默认串口 =====
DEFAULT_PORT = "/dev/serial0"   # 树莓派
# DEFAULT_PORT = "COM6"         # Windows

# ========== 帧常量 ==========
FRAME_DELIMITER = 0x00
FRAME_RAW_MAX   = 64

# RPi -> MCU
CMD_LINE_OFFSET    = 0x10
CMD_TARGET_ANGLE   = 0x11
CMD_SPEED_CMD      = 0x12
CMD_EMERGENCY_STOP = 0x1F

# MCU -> RPi
CMD_IMU_DATA    = 0x20
CMD_ENCODER_DATA = 0x21
CMD_ULTRASONIC  = 0x22
CMD_STATUS      = 0x2F
CMD_ERROR       = 0xFF

CMD_NAMES = {
    CMD_LINE_OFFSET:    "LINE_OFFSET",
    CMD_TARGET_ANGLE:   "TARGET_ANGLE",
    CMD_SPEED_CMD:      "SPEED_CMD",
    CMD_EMERGENCY_STOP: "EMERGENCY_STOP",
    CMD_IMU_DATA:       "IMU_DATA",
    CMD_ENCODER_DATA:   "ENCODER_DATA",
    CMD_ULTRASONIC:     "ULTRASONIC",
    CMD_STATUS:         "STATUS",
    CMD_ERROR:          "ERROR",
}


# ========== COBS 编解码 ==========

def cobs_encode(data: bytes) -> bytes:
    """编码，返回不含帧尾的 COBS 数据"""
    out = bytearray()
    code_idx = 0
    code_val = 1
    out.append(0)  # 占位开销字节

    for byte in data:
        if byte == 0x00:
            out[code_idx] = code_val
            code_idx = len(out)
            code_val = 1
            out.append(0)
        else:
            out.append(byte)
            code_val += 1
            if code_val == 0xFF:
                out[code_idx] = code_val
                code_idx = len(out)
                code_val = 1
                out.append(0)

    out[code_idx] = code_val
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """解码，返回原始数据（含 COBS 尾零）"""
    out = bytearray()
    read_pos = 0
    while read_pos < len(data):
        code = data[read_pos]
        read_pos += 1
        if code == 0x00:
            return None  # 非法帧
        copy_count = code - 1
        if read_pos + copy_count > len(data):
            return None
        out.extend(data[read_pos:read_pos + copy_count])
        read_pos += copy_count
        if code != 0xFF:
            out.append(0x00)
    return bytes(out)


# ========== 协议帧 ==========

def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    """构建线上帧：COBS 编码 + 0x00 帧尾"""
    raw = bytearray([cmd])
    raw.extend(payload)
    # XOR 校验
    checksum = 0
    for b in raw:
        checksum ^= b
    raw.append(checksum)
    return cobs_encode(bytes(raw)) + bytes([FRAME_DELIMITER])


class FrameParser:
    """接收侧帧解析器：喂字节，遇帧尾自动解码并回调"""

    def __init__(self, callback):
        self._cb = callback
        self._buf = bytearray()

    def feed(self, byte: int):
        if byte == FRAME_DELIMITER:
            if len(self._buf) > 0:
                self._on_frame(bytes(self._buf))
                self._buf.clear()
        else:
            if len(self._buf) >= FRAME_RAW_MAX:
                self._buf.clear()
            self._buf.append(byte)

    def _on_frame(self, encoded: bytes):
        raw = cobs_decode(encoded)
        if raw is None:
            print(f"[!] COBS decode failed")
            return
        if len(raw) < 2:
            print(f"[!] frame too short: {len(raw)}")
            return
        # 去 COBS 尾零
        raw = raw[:-1]
        # 校验
        checksum = 0
        for b in raw:
            checksum ^= b
        if checksum != 0:
            print(f"[!] checksum fail")
            return
        cmd = raw[0]
        payload = raw[1:-1]  # 去掉 cmd + checksum
        self._cb(cmd, payload)


# ========== 命令展示 ==========

def on_rx(cmd: int, payload: bytes):
    name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
    if cmd == CMD_IMU_DATA and len(payload) >= 6:
        roll, pitch, yaw = struct.unpack('>hhh', payload[:6])
        print(f"[RX] {name}: roll={roll/100:.2f}° pitch={pitch/100:.2f}° yaw={yaw/100:.2f}°")
    elif cmd == CMD_ULTRASONIC and len(payload) >= 2:
        dist, = struct.unpack('>h', payload[:2])
        print(f"[RX] {name}: dist={dist}mm")
    elif cmd == CMD_ENCODER_DATA and len(payload) >= 4:
        l_rpm, r_rpm = struct.unpack('>hh', payload[:4])
        print(f"[RX] {name}: left={l_rpm}rpm right={r_rpm}rpm")
    elif cmd == CMD_STATUS or cmd == CMD_ERROR:
        msg = payload.decode('ascii', errors='replace')
        print(f"[RX] {name}: {msg}")
    else:
        print(f"[RX] {name}: payload={payload.hex()} ({len(payload)}B)")


# ========== 主 ==========

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    do_test = "--test" in sys.argv

    ser = serial.Serial(port, 115200, timeout=0.1)
    parser = FrameParser(on_rx)
    print(f"[*] 连接 {port} @ 115200 ...")

    # 等待 MCU 启动帧
    t0 = time.time()
    while time.time() - t0 < 3.0:
        b = ser.read(1)
        if b:
            parser.feed(b[0])

    if do_test:
        print("\n=== 发送测试指令 ===")
        print("[TX] SPEED_CMD: vL=100, vR=100")
        ser.write(build_frame(CMD_SPEED_CMD, struct.pack('>hh', 100, 100)))
        time.sleep(2)
        print("[TX] EMERGENCY_STOP")
        ser.write(build_frame(CMD_EMERGENCY_STOP))

    print("\n=== 监听中 (Ctrl+C 退出) ===")
    try:
        while True:
            b = ser.read(1)
            if b:
                parser.feed(b[0])
    except KeyboardInterrupt:
        print("\n[*] 退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
