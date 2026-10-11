#!/usr/bin/env python3
"""
VT03/VT13 图传遥控串口协议监听与链路状态监测工具。

协议实现参考：components/devices/remote/vt/vt_remote.c
"""

from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple

try:
    import serial  # type: ignore
except ImportError as exc:  # pragma: no cover
    raise SystemExit(
        "未安装 pyserial，请先执行: pip install pyserial"
    ) from exc


FRAME_SIZE = 21
HEADER0 = 0xA9
HEADER1 = 0x53
CH_CENTER = 1024


KEY_NAMES = [
    "W",
    "S",
    "A",
    "D",
    "SHIFT",
    "CTRL",
    "Q",
    "E",
    "R",
    "F",
    "G",
    "Z",
    "X",
    "C",
    "V",
    "B",
]

GEAR_NAMES = {
    0: "C",
    1: "N",
    2: "S",
}


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def crc16_ccitt_reflected(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = ((crc >> 1) ^ 0x8408) & 0xFFFF
            else:
                crc = (crc >> 1) & 0xFFFF
    return crc


def verify_frame(frame: bytes) -> bool:
    if len(frame) != FRAME_SIZE:
        return False
    if frame[0] != HEADER0 or frame[1] != HEADER1:
        return False

    calc_false = crc16_ccitt_false(frame[:-2])
    calc_ref = crc16_ccitt_reflected(frame[:-2])
    rx_le = frame[-2] | (frame[-1] << 8)
    rx_be = frame[-1] | (frame[-2] << 8)
    return (
        calc_false == rx_le
        or calc_false == rx_be
        or calc_ref == rx_le
        or calc_ref == rx_be
    )


def frame_crc_info(frame: bytes) -> Tuple[int, int, int, bool]:
    calc_false = crc16_ccitt_false(frame[:-2])
    calc_ref = crc16_ccitt_reflected(frame[:-2])
    rx_le = frame[-2] | (frame[-1] << 8)
    rx_be = frame[-1] | (frame[-2] << 8)
    ok = (
        (calc_false == rx_le)
        or (calc_false == rx_be)
        or (calc_ref == rx_le)
        or (calc_ref == rx_be)
    )
    return calc_false, rx_le, rx_be, ok


def get_bits_u16(buf: bytes, start_bit: int, bit_len: int) -> int:
    out = 0
    for i in range(bit_len):
        bit_index = start_bit + i
        byte_index = bit_index >> 3
        bit_in_byte = bit_index & 0x07
        bit = (buf[byte_index] >> bit_in_byte) & 0x01
        out |= bit << i
    return out


def to_i16(v: int) -> int:
    return v - 0x10000 if v & 0x8000 else v


def ch_percent(raw: int) -> float:
    # 经验范围来自固件头文件: [364, 1684]，中心 1024
    return (raw - CH_CENTER) / 660.0 * 100.0


def format_key_list(keys: List[str]) -> str:
    return ",".join(keys) if keys else "-"


def dump_fields(parsed: Dict[str, int | List[str]], tag: str = "DATA") -> None:
    ch0 = int(parsed["ch0"])
    ch1 = int(parsed["ch1"])
    ch2 = int(parsed["ch2"])
    ch3 = int(parsed["ch3"])
    dial = int(parsed["dial"])
    ch0_c = int(parsed["ch0_c"])
    ch1_c = int(parsed["ch1_c"])
    ch2_c = int(parsed["ch2_c"])
    ch3_c = int(parsed["ch3_c"])
    dial_c = int(parsed["dial_c"])
    mouse_x = int(parsed["mouse_x"])
    mouse_y = int(parsed["mouse_y"])
    mouse_z = int(parsed["mouse_z"])
    keyboard = int(parsed["keyboard"])
    keys = parsed["keys"]

    print(
        f"[{tag}] gear={parsed['gear_name']}({int(parsed['gear'])}) "
        f"pause={int(parsed['pause'])} customL={int(parsed['custom_l'])} "
        f"customR={int(parsed['custom_r'])} trigger={int(parsed['trigger'])}"
    )
    print(
        f"[{tag}] CH0(右X): raw={ch0:4d} centered={ch0_c:+4d} ({ch_percent(ch0):+6.2f}%) | "
        f"CH1(右Y): raw={ch1:4d} centered={ch1_c:+4d} ({ch_percent(ch1):+6.2f}%)"
    )
    print(
        f"[{tag}] CH2(左Y): raw={ch2:4d} centered={ch2_c:+4d} ({ch_percent(ch2):+6.2f}%) | "
        f"CH3(左X): raw={ch3:4d} centered={ch3_c:+4d} ({ch_percent(ch3):+6.2f}%)"
    )
    print(
        f"[{tag}] DIAL(拨轮): raw={dial:4d} centered={dial_c:+4d} ({ch_percent(dial):+6.2f}%)"
    )
    print(
        f"[{tag}] mouse(dx,dy,dz)=({mouse_x},{mouse_y},{mouse_z}) "
        f"mouse_btn(L/R/M)=({int(parsed['mouse_l'])}/{int(parsed['mouse_r'])}/{int(parsed['mouse_m'])})"
    )
    print(f"[{tag}] keyboard=0x{keyboard:04X} keys={format_key_list(keys)}")


def parse_frame(frame: bytes) -> Dict[str, int | List[str]]:
    ch0 = get_bits_u16(frame, 16, 11)
    ch1 = get_bits_u16(frame, 27, 11)
    ch2 = get_bits_u16(frame, 38, 11)
    ch3 = get_bits_u16(frame, 49, 11)
    dial = get_bits_u16(frame, 65, 11)

    keyboard = get_bits_u16(frame, 136, 16)
    pressed_keys = [name for i, name in enumerate(KEY_NAMES) if (keyboard >> i) & 0x01]

    return {
        "ch0": ch0,
        "ch1": ch1,
        "ch2": ch2,
        "ch3": ch3,
        "dial": dial,
        "ch0_c": ch0 - CH_CENTER,
        "ch1_c": ch1 - CH_CENTER,
        "ch2_c": ch2 - CH_CENTER,
        "ch3_c": ch3 - CH_CENTER,
        "dial_c": dial - CH_CENTER,
        "gear": get_bits_u16(frame, 60, 2),
        "gear_name": GEAR_NAMES.get(get_bits_u16(frame, 60, 2), "UNK"),
        "pause": get_bits_u16(frame, 62, 1),
        "custom_l": get_bits_u16(frame, 63, 1),
        "custom_r": get_bits_u16(frame, 64, 1),
        "trigger": get_bits_u16(frame, 76, 1),
        "mouse_x": to_i16(get_bits_u16(frame, 80, 16)),
        "mouse_y": to_i16(get_bits_u16(frame, 96, 16)),
        "mouse_z": to_i16(get_bits_u16(frame, 112, 16)),
        "mouse_l": 1 if get_bits_u16(frame, 128, 2) == 1 else 0,
        "mouse_r": 1 if get_bits_u16(frame, 130, 2) == 1 else 0,
        "mouse_m": 1 if get_bits_u16(frame, 132, 2) == 1 else 0,
        "keyboard": keyboard,
        "keys": pressed_keys,
    }


@dataclass
class LinkStats:
    total_frames: int = 0
    valid_frames: int = 0
    invalid_frames: int = 0
    bytes_in: int = 0
    resync_drop_bytes: int = 0
    header_frames: int = 0
    last_valid_ts: float = 0.0
    first_valid_ts: float = 0.0

    def on_valid(self, now: float) -> None:
        self.total_frames += 1
        self.valid_frames += 1
        self.last_valid_ts = now
        if self.first_valid_ts == 0.0:
            self.first_valid_ts = now

    def on_invalid(self) -> None:
        self.total_frames += 1
        self.invalid_frames += 1

    @property
    def crc_ok_rate(self) -> float:
        if self.total_frames == 0:
            return 0.0
        return self.valid_frames / self.total_frames

    def fps(self, now: float) -> float:
        if self.first_valid_ts == 0.0:
            return 0.0
        dt = max(now - self.first_valid_ts, 1e-6)
        return self.valid_frames / dt


def extract_frames(rx_buf: bytearray, stats: LinkStats) -> List[Tuple[bytes, bool, int, int, int]]:
    frames: List[Tuple[bytes, bool, int, int, int]] = []

    while True:
        if len(rx_buf) < 2:
            break

        if rx_buf[0] != HEADER0 or rx_buf[1] != HEADER1:
            idx = rx_buf.find(bytes([HEADER0, HEADER1]))
            if idx == -1:
                drop = max(0, len(rx_buf) - 1)
                del rx_buf[:drop]
                stats.resync_drop_bytes += drop
                break
            if idx > 0:
                del rx_buf[:idx]
                stats.resync_drop_bytes += idx

        if len(rx_buf) < FRAME_SIZE:
            break

        candidate = bytes(rx_buf[:FRAME_SIZE])
        calc, rx_le, rx_be, ok = frame_crc_info(candidate)
        stats.header_frames += 1
        if ok:
            frames.append((candidate, True, calc, rx_le, rx_be))
            del rx_buf[:FRAME_SIZE]
        else:
            stats.on_invalid()
            frames.append((candidate, False, calc, rx_le, rx_be))
            del rx_buf[0]
            stats.resync_drop_bytes += 1

    return frames


def main() -> int:
    parser = argparse.ArgumentParser(description="VT 串口协议解析与链路监测")
    parser.add_argument("--port", required=True, help="串口号，如 /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=921600, help="波特率，默认 921600")
    parser.add_argument("--serial-timeout", type=float, default=0.02, help="串口读超时(秒)")
    parser.add_argument("--link-timeout", type=float, default=0.20, help="链路离线阈值(秒)")
    parser.add_argument("--report-interval", type=float, default=0.50, help="状态打印间隔(秒)")
    parser.add_argument("--verbose", action="store_true", help="打印每帧解析结果")
    parser.add_argument("--show-raw", action="store_true", help="verbose 模式下显示原始十六进制")
    parser.add_argument("--show-invalid", action="store_true", help="显示 CRC 失败帧的解析(诊断用)")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.serial_timeout)
    print(f"[INFO] 打开串口: {args.port}, baud={args.baud}")
    print(f"[INFO] 链路离线阈值: {args.link_timeout:.3f}s")

    stats = LinkStats()
    rx_buf = bytearray()
    online = False
    last_report_ts = time.monotonic()
    last_parsed: Dict[str, int | List[str]] | None = None

    try:
        while True:
            chunk = ser.read(512)
            now = time.monotonic()

            if chunk:
                rx_buf.extend(chunk)
                stats.bytes_in += len(chunk)

            frames = extract_frames(rx_buf, stats)
            for frame, ok, calc, rx_le, rx_be in frames:
                if ok:
                    stats.on_valid(now)
                    if not online:
                        online = True
                        print(f"[LINK] ONLINE (t={now:.3f})")

                parsed = parse_frame(frame)
                if ok:
                    last_parsed = parsed

                if args.verbose and ok:
                    line = (
                        f"[FRAME] gear={parsed['gear_name']}({parsed['gear']}) pause={parsed['pause']} "
                        f"ch(c)=({parsed['ch0_c']:+d},{parsed['ch1_c']:+d},{parsed['ch2_c']:+d},{parsed['ch3_c']:+d}) "
                        f"dial={parsed['dial_c']:+d} mouse=({parsed['mouse_x']},{parsed['mouse_y']},{parsed['mouse_z']}) "
                        f"btn(LRM)=({parsed['mouse_l']},{parsed['mouse_r']},{parsed['mouse_m']}) "
                        f"keys={format_key_list(parsed['keys'])}"
                    )
                    print(line)
                    if args.show_raw:
                        print("       raw:", frame.hex(" "))

                if args.show_invalid and (not ok):
                    print(
                        "[BAD ] "
                        f"CRC_FAIL calc=0x{calc:04X} rxLE=0x{rx_le:04X} rxBE=0x{rx_be:04X} "
                        f"gear={parsed['gear_name']} keys={format_key_list(parsed['keys'])}"
                    )
                    dump_fields(parsed, tag="BADD")
                    if args.show_raw:
                        print("       raw:", frame.hex(" "))

            if online and (now - stats.last_valid_ts) > args.link_timeout:
                online = False
                print(f"[LINK] OFFLINE (>{args.link_timeout:.3f}s 无有效帧)")

            if (now - last_report_ts) >= args.report_interval:
                last_report_ts = now
                age = (now - stats.last_valid_ts) if stats.last_valid_ts > 0 else float("inf")
                print(
                    "[STAT] "
                    f"online={1 if online else 0} "
                    f"valid={stats.valid_frames} invalid={stats.invalid_frames} "
                    f"header={stats.header_frames} "
                    f"crc_ok={stats.crc_ok_rate * 100:.1f}% "
                    f"fps={stats.fps(now):.1f} "
                    f"last_valid_age={age:.3f}s "
                    f"bytes={stats.bytes_in} drop={stats.resync_drop_bytes}"
                )

                if last_parsed is not None:
                    dump_fields(last_parsed, tag="DATA")

    except KeyboardInterrupt:
        print("\n[INFO] 用户中断，退出。")
    finally:
        ser.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
