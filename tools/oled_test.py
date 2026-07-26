"""
OLED 协议测试 — 向 MSPM0 发送显示指令

用法:
    python oled_test.py              (使用默认串口)
    python oled_test.py <串口>        (指定串口)
"""

import serial
import sys
import struct

# ===== 在此修改默认串口 =====
DEFAULT_PORT = "/dev/serial0"   # 树莓派
# DEFAULT_PORT = "COM6"         # Windows

FRAME_DELIMITER = 0x00

CMD_OLED_SHOW = 0x13
CMD_STATUS    = 0x2F
CMD_ERROR     = 0xFF

CMD_NAMES = {CMD_STATUS: "STATUS", CMD_ERROR: "ERROR"}


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = 0
    code_val = 1
    out.append(0)
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


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    rp = 0
    while rp < len(data):
        code = data[rp]
        rp += 1
        if code == 0x00:
            return None
        n = code - 1
        if rp + n > len(data):
            return None
        out.extend(data[rp:rp + n])
        rp += n
        if code != 0xFF:
            out.append(0x00)
    return bytes(out)


def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    raw = bytearray([cmd])
    raw.extend(payload)
    cs = 0
    for b in raw:
        cs ^= b
    raw.append(cs)
    return cobs_encode(bytes(raw)) + bytes([FRAME_DELIMITER])


def feed(ser: serial.Serial) -> None:
    """读取串口并解析帧，打印结果"""
    buf = bytearray()
    while ser.in_waiting:
        byte = ser.read(1)[0]
        if byte == FRAME_DELIMITER:
            if len(buf):
                raw = cobs_decode(bytes(buf))
                buf.clear()
                if raw is None or len(raw) < 2:
                    continue
                raw = raw[:-1]  # 去 COBS 尾零
                cs = 0
                for b in raw:
                    cs ^= b
                if cs != 0:
                    continue
                cmd = raw[0]
                pl = raw[1:-1]
                name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")
                msg = pl.decode("ascii", errors="replace")
                print(f"[RX] {name}: {msg}")
        else:
            if len(buf) < 128:
                buf.append(byte)


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    ser = serial.Serial(port, 115200, timeout=0.05)

    # 等启动帧
    print(f"[*] 连接 {port} ...")
    import time
    t0 = time.time()
    while time.time() - t0 < 2:
        feed(ser)

    print("\n用法: 输入 '<行号> <文本>' 回车发送, Ctrl+C 退出")
    print("      行号 0~3, 最多 16 个英文字符")
    print("      也可直接发十六进制: \\x13\\x00Hello")
    print()

    try:
        while True:
            line = input("> ").strip()
            if not line:
                continue

            # 支持直接输入十六进制帧
            if line.startswith("\\x"):
                # 解析 \x13\x00Hello 格式
                parts = []
                i = 0
                while i < len(line):
                    if line[i] == '\\' and i + 3 < len(line) and line[i + 1] == 'x':
                        parts.append(int(line[i + 2:i + 4], 16))
                        i += 4
                    else:
                        parts.append(ord(line[i]))
                        i += 1
                ser.write(bytes(parts))
                ser.flush()
            else:
                # 普通文本: "<行号> <内容>"
                parts = line.split(" ", 1)
                row = int(parts[0])
                text = parts[1] if len(parts) > 1 else ""
                payload = bytes([row]) + text.encode("ascii")
                ser.write(build_frame(CMD_OLED_SHOW, payload))
                ser.flush()

            # 等回复
            time.sleep(0.1)
            feed(ser)

    except KeyboardInterrupt:
        print("\n[*] 退出")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
