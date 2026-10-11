#!/usr/bin/env python3
"""RM2026 云台自瞄「基本跟随」验证桩。

替代上位机视觉栈：收下位机遥测帧（姿态），在虚拟世界系里放一个目标，
把目标相对云台当前指向的方位按协议发回去。下位机无法区分它与真实视觉。

协议基线见 docs/vision/vision_protocol.md，用法见同目录 README.md。
"""

import argparse
import math
import struct
import sys
import time

import numpy as np
import serial

# --------------------------------------------------------------------------
# 协议常量（必须与下位机 components/devices/vision/vision_protocol.h 一致）
# --------------------------------------------------------------------------
TELEMETRY_HEADER = 0x5A
TARGET_HEADER = 0xA5
TELEMETRY_PAYLOAD_LEN = 18
TARGET_PAYLOAD_LEN = 48
FRAME_OVERHEAD = 4

TELEM_FLAG_ENEMY_COLOR = 1 << 0
TELEM_FLAG_RESET_TRACKER = 1 << 1
TELEM_FLAG_AUTO_AIM_ENABLE = 1 << 2

TARGET_FLAG_TRACKING = 1 << 0
TARGET_ID_SHIFT = 1
TARGET_ARMORS_SHIFT = 4

DEG2RAD = math.pi / 180.0


def build_crc_table():
    """CRC-16/MCRF4XX：多项式 0x8408（反射）、初值 0xFFFF、无最终异或。"""
    table = np.zeros(256, dtype=np.uint16)
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if (crc & 1) else (crc >> 1)
        table[i] = crc
    return table


CRC_TABLE = build_crc_table()
assert CRC_TABLE[1] == 0x1189, "CRC 表首项应为 0x1189"


def crc16_vectorized(chunks):
    """对 (n, L) 的 uint8 数组逐行算 CRC，返回 (n,) uint16。

    numpy 只是把逐字节循环向量化——算法本身与下位机查表实现完全相同，
    不是另一套近似实现。
    """
    crc = np.full(chunks.shape[0], 0xFFFF, dtype=np.uint16)
    for i in range(chunks.shape[1]):
        idx = (crc ^ chunks[:, i]) & 0xFF
        crc = (crc >> 8) ^ CRC_TABLE[idx]
    return crc


# --------------------------------------------------------------------------
# 帧编解码
# --------------------------------------------------------------------------
def decode_telemetry(frame):
    """22B 遥测帧 -> dict；CRC/长度/帧头任一不符返回 None。"""
    if len(frame) != TELEMETRY_PAYLOAD_LEN + FRAME_OVERHEAD:
        return None
    if frame[0] != TELEMETRY_HEADER:
        return None
    if frame[1] != TELEMETRY_PAYLOAD_LEN:
        return None
    want = int(crc16_vectorized(frame[None, :-2])[0])
    got = int(frame[-2]) | (int(frame[-1]) << 8)
    if want != got:
        return None

    flags, seq = struct.unpack_from("<HH", frame, 2)
    bullet_speed = frame[6]
    yaw, pitch, roll = struct.unpack_from("<fff", frame, 8)
    return {
        "flags": flags,
        "seq": seq,
        "bullet_speed": bullet_speed,
        "yaw": yaw,
        "pitch": pitch,
        "roll": roll,
        "enemy_color": flags & TELEM_FLAG_ENEMY_COLOR,
        "reset_tracker": bool(flags & TELEM_FLAG_RESET_TRACKER),
        "auto_aim_enable": bool(flags & TELEM_FLAG_AUTO_AIM_ENABLE),
    }


def encode_target(seq, x, y, z, tracking=True, target_id=1, armors_num=4):
    """组 52B 目标帧。位置为云台系。"""
    flags = 0
    if tracking:
        flags |= TARGET_FLAG_TRACKING
    flags |= (target_id & 0x7) << TARGET_ID_SHIFT
    flags |= (armors_num & 0x7) << TARGET_ARMORS_SHIFT

    frame = bytearray(TARGET_PAYLOAD_LEN + FRAME_OVERHEAD)
    frame[0] = TARGET_HEADER
    frame[1] = TARGET_PAYLOAD_LEN
    struct.pack_into("<HH", frame, 2, flags, seq & 0xFFFF)
    struct.pack_into("<fffffffffff", frame, 6, x, y, z, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    crc = int(crc16_vectorized(np.frombuffer(bytes(frame[:50]), dtype=np.uint8)[None, :])[0])
    struct.pack_into("<H", frame, 50, crc)
    return bytes(frame)


class TelemetryReader:
    """从字节流里切出遥测帧，帧头/长度/CRC 三重校验，失败逐字节重同步。"""

    def __init__(self):
        self.buf = bytearray()
        self.rx_frames = 0
        self.crc_errors = 0
        self.format_errors = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            start = self.buf.find(bytes([TELEMETRY_HEADER]))
            if start < 0:
                # 没有帧头，整段丢弃（保留最后 1 字节以防帧头被切开）
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return out
            if start > 0:
                del self.buf[:start]  # 丢弃帧头之前的杂字节
            if len(self.buf) < 2:
                return out
            if self.buf[1] != TELEMETRY_PAYLOAD_LEN:
                self.format_errors += 1
                del self.buf[:1]  # 只丢这一个字节，让下一个 0x5A 重新成为候选
                continue
            total = TELEMETRY_PAYLOAD_LEN + FRAME_OVERHEAD
            if len(self.buf) < total:
                return out
            frame = np.frombuffer(bytes(self.buf[:total]), dtype=np.uint8)
            telem = decode_telemetry(frame)
            if telem is None:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            self.rx_frames += 1
            out.append(telem)


def build_parser():
    p = argparse.ArgumentParser(
        description="RM2026 云台自瞄基本跟随验证桩",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("--port", default="/dev/ttyUSB0", help="下位机串口设备")
    p.add_argument("--baud", type=int, default=921600, help="波特率（须与下位机一致）")
    p.add_argument("--rate-hz", type=float, default=100.0, help="目标帧发送频率")
    p.add_argument(
        "--target-yaw-deg",
        type=float,
        default=20.0,
        help="目标相对【启动时云台指向】的偏角，正=左",
    )
    p.add_argument("--target-distance-m", type=float, default=1.5, help="水平距离")
    p.add_argument("--target-height-m", type=float, default=0.2, help="相对云台的高度")
    p.add_argument(
        "--orbit-deg-s",
        type=float,
        default=0.0,
        help="目标绕云台的角速度，非 0 时验证持续跟随",
    )
    p.add_argument(
        "--flip-every-s",
        type=float,
        default=0.0,
        help="每隔 N 秒把目标方位取反，用于确认 yaw 正方向",
    )
    p.add_argument("--listen-only", action="store_true", help="只收不发，排查接线用")
    p.add_argument("--verbose", action="store_true", help="每帧打印")
    return p


def main():
    args = build_parser().parse_args()

    try:
        port = serial.Serial(args.port, args.baud, timeout=0)
    except serial.SerialException as exc:
        print(f"打开串口失败: {exc}", file=sys.stderr)
        return 1

    reader = TelemetryReader()
    yaw0 = None
    target_seq = 0
    t_start = time.monotonic()
    last_report = t_start
    last_rx = 0
    last_tx = 0
    last_crc = 0
    last_fmt = 0
    cur_yaw = None
    last_rel = None
    period = 1.0 / args.rate_hz

    def report(rel_deg=None):
        """按 1 s 周期打印增量计数（与下位机 LogVisionStatus 同样思路）。"""
        nonlocal last_report, last_rx, last_tx, last_crc, last_fmt
        line = (
            f"[1s] rx={reader.rx_frames - last_rx} "
            f"crc={reader.crc_errors - last_crc} "
            f"fmt={reader.format_errors - last_fmt} "
            f"| tx={target_seq - last_tx}"
        )
        if cur_yaw is not None:
            line += f" yaw={cur_yaw:8.2f}"
        if rel_deg is not None:
            line += f" rel={rel_deg:7.2f}"
        print(line)
        last_rx = reader.rx_frames
        last_crc = reader.crc_errors
        last_fmt = reader.format_errors
        last_tx = target_seq
        last_report = time.monotonic()

    print(f"已打开 {args.port} @ {args.baud}")
    print("等待下位机遥测……（下位机 USART1 引脚：PA9=TX -> 本机 RXD，PB7=RX <- 本机 TXD）")
    if args.listen_only:
        print("--listen-only：只收不发")

    try:
        while True:
            now = time.monotonic()

            chunk = port.read(4096)
            if chunk:
                for telem in reader.feed(chunk):
                    cur_yaw = telem["yaw"]
                    if yaw0 is None:
                        yaw0 = telem["yaw"]
                        print(
                            f"\n收到首个遥测帧：yaw0={yaw0:.3f} pitch={telem['pitch']:.3f} "
                            f"roll={telem['roll']:.3f} bullet_speed={telem['bullet_speed']}"
                        )
                        print(
                            f"目标将放在启动指向偏 {args.target_yaw_deg} deg 处、"
                            f"距离 {args.target_distance_m} m、高 {args.target_height_m} m\n"
                        )
                    if args.verbose:
                        print(
                            f"  rx seq={telem['seq']} yaw={telem['yaw']:.3f} "
                            f"pitch={telem['pitch']:.3f} aim={int(telem['auto_aim_enable'])}"
                        )

            if now - last_report >= 1.0:
                report(last_rel)

            if args.listen_only or yaw0 is None:
                time.sleep(0.001)
                continue

            elapsed = now - t_start
            bearing = args.target_yaw_deg
            if args.orbit_deg_s:
                bearing += args.orbit_deg_s * elapsed
            if args.flip_every_s:
                cycle = int(elapsed / args.flip_every_s)
                if cycle % 2 == 1:
                    bearing = -bearing

            # 世界系目标方位 -> 云台系相对方位。
            # 这一步是唯一容易写错的地方：gimbal_link 随云台转动，下位机把
            # (x, y, z) 当【相对当前指向】用。若此处不减去当前 yaw，发出去的
            # 就是世界系绝对方位，下位机会当成相对方位，形成正反馈持续打偏。
            world_yaw = yaw0 + bearing
            rel_deg = world_yaw - cur_yaw
            rel = rel_deg * DEG2RAD
            x = args.target_distance_m * math.cos(rel)
            y = args.target_distance_m * math.sin(rel)
            z = args.target_height_m

            port.write(encode_target(target_seq, x, y, z))
            target_seq += 1
            last_rel = rel_deg

            elapsed_send = time.monotonic() - now
            if elapsed_send < period:
                time.sleep(period - elapsed_send)

    except KeyboardInterrupt:
        print("\n退出。")
    finally:
        port.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
