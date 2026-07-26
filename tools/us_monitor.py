"""
超声波数据监听 — 接收 MCU 上报的距离

用法:
    python us_monitor.py              (使用默认串口)
    python us_monitor.py <串口>        (指定串口)
    python us_monitor.py --debug      (打印收到的原始字节)
"""

import serial
import sys
import struct
import time

DEFAULT_PORT = "/dev/serial0"

FRAME_DELIMITER = 0x00

CMD_STATUS     = 0x2F
CMD_ULTRASONIC = 0x22
CMD_NAMES = {CMD_STATUS: "STATUS", CMD_ULTRASONIC: "US"}


def cobs_decode(data: bytes) -> bytes | None:
    out = bytearray()
    rp = 0
    while rp < len(data):
        code = data[rp]; rp += 1
        if code == 0x00: return None
        n = code - 1
        if rp + n > len(data): return None
        out.extend(data[rp:rp + n]); rp += n
        if code != 0xFF: out.append(0x00)
    return bytes(out)


def main():
    debug = "--debug" in sys.argv
    args  = [a for a in sys.argv[1:] if not a.startswith("--")]
    port  = args[0] if args else DEFAULT_PORT

    ser = serial.Serial(port, 115200, timeout=0.5)
    print(f"[*] 监听 {port} @ 115200 ...")
    if debug:
        print("[*] 调试模式：会打印所有收到的原始字节")
    print()

    buf = bytearray()
    last_us = 0

    while True:
        try:
            # 一次读完当前缓冲中的所有字节
            raw = ser.read(ser.in_waiting or 1)
            if not raw:
                continue

            for b in raw:
                byte = b  # int

                if debug:
                    print(f"  [{byte:02X}]", end=" ", flush=True)

                if byte == FRAME_DELIMITER:
                    if len(buf):
                        if debug:
                            print(f"\n  -> 帧: {buf.hex()} ({len(buf)}B)")

                        decoded = cobs_decode(bytes(buf))
                        buf.clear()

                        if decoded is None or len(decoded) < 3:
                            if debug: print("  -> COBS 解码失败")
                            continue

                        decoded = decoded[:-1]  # 去 COBS 尾零

                        # XOR 校验
                        cs = 0
                        for b2 in decoded:
                            cs ^= b2
                        if cs != 0:
                            if debug: print("  -> 校验失败")
                            continue

                        cmd = decoded[0]
                        pl  = decoded[1:-1]
                        name = CMD_NAMES.get(cmd, f"0x{cmd:02X}")

                        if cmd == CMD_ULTRASONIC and len(pl) >= 2:
                            dist, = struct.unpack('>h', pl[:2])
                            now = time.time()
                            if now - last_us > 0.5:  # 每秒换行
                                print()
                            last_us = now
                            bar = '#' * min(int(dist / 20), 50)
                            print(f"\r[{time.strftime('%H:%M:%S')}] {dist:>4} mm |{bar:<50}|", end='', flush=True)
                        elif cmd == CMD_STATUS:
                            msg = pl.decode('ascii', errors='replace')
                            print(f"\n[{name}] {msg}")
                        else:
                            print(f"\n[{name}] {pl.hex()} ({len(pl)}B)")

                        if debug:
                            print()

                else:
                    if len(buf) < 128:
                        buf.append(byte)
                    elif debug:
                        print(f"\n  -> 缓冲溢出，丢弃")
                        buf.clear()

        except KeyboardInterrupt:
            print("\n[*] 退出")
            break

    ser.close()


if __name__ == "__main__":
    main()
