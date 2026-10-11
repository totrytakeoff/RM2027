#!/usr/bin/env python3
"""MC-02 development commands; RM2026 script interfaces adapted to H7 presets.

Debug/log workflows derived from RM2026 scripts by YZ-Control/myself.
Build and upload use independent native scripts without Python.

Debug and log entries share this implementation. Importing has no effects.
"""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import re
import shlex
import shutil
import socket
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_CFG = ROOT / 'platform/boards/dm_mc02/openocd/cmsis-dap.cfg'


def executable(name):
    found = shutil.which(name)
    if found:
        return found
    if name == 'openocd':
        candidate = Path.home() / '.platformio/packages/tool-openocd/bin' / (
            'openocd.exe' if os.name == 'nt' else 'openocd')
        if candidate.is_file():
            return str(candidate)
    raise ValueError(f'找不到工具: {name}')


def run(command, dry=False, cwd=ROOT):
    print('Command:', subprocess.list2cmdline(command) if os.name == 'nt'
          else shlex.join(command), flush=True)
    if not dry:
        subprocess.run(command, cwd=cwd, check=True)


def preset_dir(name):
    entries = json.loads((ROOT / 'CMakePresets.json').read_text())['configurePresets']
    presets = {p['name']: p for p in entries}
    if name not in presets:
        raise ValueError(f'未知 preset: {name}; 可选: {", ".join(presets)}')

    def merged(key):
        entry = presets[key]
        parents = entry.get('inherits', [])
        if isinstance(parents, str):
            parents = [parents]
        result = {}
        for parent in reversed(parents):
            result.update(merged(parent))
        result.update(entry)
        return result

    path = merged(name)['binaryDir'].replace('${sourceDir}', str(ROOT))
    return Path(path).resolve()


def default_target(preset):
    if preset == 'host':
        return 'all'
    if preset == 'h7-dbus-debug':
        return 'board_bringup.elf'
    return 'chassis_demo.elf'


def image_path(name, preset, fmt='elf'):
    path = Path(name).expanduser()
    if path.suffix.lower() not in ('.elf', '.bin', '.hex'):
        path = Path(str(path) + '.' + fmt)
    if path.is_absolute():
        return path.resolve()
    for candidate in (Path.cwd() / path, ROOT / path):
        if candidate.is_file():
            return candidate.resolve()
    if len(path.parts) == 1:
        return preset_dir(preset) / path
    return (ROOT / path).resolve()


def tcl_quote(value):
    # OpenOCD -c is Tcl code; shell quoting alone does not protect filenames.
    value = value.as_posix() if isinstance(value, Path) else str(value)
    for char in ('\\', '"', '$', '[', ']'):
        value = value.replace(char, '\\' + char)
    return '"' + value.replace('\n', '\\n').replace('\r', '\\r') + '"'


def config_path(value):
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = ROOT / path
    if not path.is_file():
        raise ValueError(f'OpenOCD 配置不存在: {path}')
    return path.resolve()


def openocd_command(args):
    cfg = config_path(args.cfg)
    command = [executable(args.openocd), '-f', str(cfg)]
    backend = {'dapv2': 'usb_bulk', 'dapv1': 'hid'}.get(args.backend)
    if backend:
        command += ['-c', f'cmsis_dap_backend {backend}']
    if args.speed:
        command += ['-c', f'adapter speed {args.speed}']
    return command


def debug(args):
    command = openocd_command(args)
    command += ['-c', f'gdb_port {args.port}', '-c', f'tcl_port {args.tcl_port}',
                '-c', f'telnet_port {args.telnet_port}', '-c', 'init']
    if args.halt:
        command += ['-c', 'reset halt']
    run(command, args.dry_run)


def gdb(args):
    image = image_path(args.file or default_target(args.preset), args.preset)
    if image.suffix != '.elf':
        raise ValueError('GDB 需要 ELF 文件')
    if not image.is_file() and not args.dry_run:
        raise ValueError(f'未找到 ELF: {image}')
    command = [executable(args.gdb), str(image), '-ex', f'target remote {args.host}:{args.port}']
    if args.load or args.reset or args.break_main:
        command += ['-ex', 'monitor reset halt']
    if args.load:
        command += ['-ex', 'load']
    if args.break_main:
        command += ['-ex', 'hbreak main', '-ex', 'continue']
    run(command, args.dry_run)


def serial_log(args):
    if args.dry_run:
        print(f'Serial: device={args.device or "auto"} baud={args.baud} 8N1 '
              f'duration={args.duration or "until Ctrl+C"} output={args.output or "stdout"}')
        return
    try:
        import serial
        from serial.tools import list_ports
    except ImportError as exc:
        raise ValueError('需要 pyserial: python3 -m pip install pyserial') from exc
    device = args.device
    if not device:
        ports = list(list_ports.comports())
        matches = [p for p in ports if p.vid == 0xfaed and p.pid == 0x4870]
        if len(matches) != 1:
            raise ValueError('请用 --device 指定串口；可用端口: ' +
                             ', '.join(f'{p.device} ({p.description})' for p in ports))
        device = matches[0].device
    output = None
    if args.output:
        path = Path(args.output).expanduser()
        path.parent.mkdir(parents=True, exist_ok=True)
        output = path.open('wb')
    print(f'Serial: {device}, {args.baud} 8N1 (Ctrl+C 退出)', flush=True)
    start = time.monotonic()
    try:
        with serial.Serial(device, args.baud, timeout=0.1, exclusive=os.name != 'nt') as port:
            while not args.duration or time.monotonic() - start < args.duration:
                chunk = port.read(max(1, port.in_waiting))
                if chunk:
                    if output:
                        output.write(chunk)
                        output.flush()
                    sys.stdout.buffer.write(chunk)
                    sys.stdout.buffer.flush()
    finally:
        if output:
            output.close()


def rpc(host, port, command):
    with socket.create_connection((host, port), timeout=3) as connection:
        connection.settimeout(5)
        connection.sendall(command.encode() + b'\x1a')
        data = bytearray()
        while b'\x1a' not in data:
            chunk = connection.recv(4096)
            if not chunk:
                raise ValueError('OpenOCD Tcl 连接断开')
            data.extend(chunk)
        reply = data.split(b'\x1a', 1)[0].decode(errors='replace')
    if re.search(r'(?i)error|invalid command|unknown command|not found|failed', reply):
        raise ValueError(f'OpenOCD: {reply}')
    return reply


def listening(host, port):
    try:
        with socket.create_connection((host, port), timeout=0.2):
            return True
    except OSError:
        return False


def rtt(args):
    image = image_path(args.file or default_target(args.preset), args.preset)
    # Current framework has a UART log ring, not SEGGER RTT. Do not connect
    # or reset the board unless the chosen image actually supplies RTT.
    if args.rtt_address is None:
        if not image.is_file():
            raise ValueError(f'未找到 ELF: {image}')
        symbols = subprocess.check_output([executable('arm-none-eabi-nm'), str(image)], text=True)
        match = re.search(r'^([0-9a-fA-F]+)\s+\w\s+_SEGGER_RTT$', symbols, re.MULTILINE)
        if not match:
            raise ValueError('当前固件没有 _SEGGER_RTT；请使用 log.sh 查看 USART 日志。'
                             '外部 RTT 固件可指定 --file，或 --rtt-address/--rtt-size。')
        address = int(match[1], 16)
        size = args.rtt_size or 4096
    else:
        address = args.rtt_address
        size = args.rtt_size or 0x50000
    command = openocd_command(args)
    command += ['-c', f'tcl_port {args.tcl_port}', '-c', f'telnet_port {args.telnet_port}', '-c', 'init']
    setup = (f'rtt setup {address:#x} {size:#x} {tcl_quote(args.rtt_name)}; '
             f'rtt start; rtt server start {args.rtt_port} {args.channel}')
    if args.dry_run:
        run(command, True)
        print('RTT:', setup)
        return
    owned = None
    log = None
    try:
        if not listening(args.host, args.tcl_port):
            if args.host not in ('localhost', '127.0.0.1', '::1'):
                raise ValueError('远程 OpenOCD 未启动；请在远端先启动服务器')
            log = tempfile.TemporaryFile()
            owned = subprocess.Popen(command, cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic() + 10
            while not listening(args.host, args.tcl_port):
                if owned.poll() is not None or time.monotonic() > deadline:
                    log.seek(0)
                    raise ValueError('OpenOCD 启动失败:\n' + log.read().decode(errors='replace'))
                time.sleep(0.05)
        if args.reset_run == 1:
            rpc(args.host, args.tcl_port, 'reset run')
            time.sleep(0.2)
        print(rpc(args.host, args.tcl_port, setup), flush=True)
        start = time.monotonic()
        with socket.create_connection((args.host, args.rtt_port), timeout=3) as stream:
            stream.settimeout(0.2)
            while not args.duration or time.monotonic() - start < args.duration:
                try:
                    data = stream.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                sys.stdout.buffer.write(data)
                sys.stdout.buffer.flush()
    finally:
        if owned and not args.keep_openocd:
            owned.terminate()
            try:
                owned.wait(timeout=3)
            except subprocess.TimeoutExpired:
                owned.kill()
                owned.wait()
        if log:
            log.close()


def integer(value):
    return int(value, 0)


def positive(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError('必须为正整数')
    return number


def port_number(value):
    port = positive(value)
    if port > 65535:
        raise argparse.ArgumentTypeError('端口必须在 1..65535')
    return port


def duration(value):
    number = float(value)
    if not 0 <= number < float('inf'):
        raise argparse.ArgumentTypeError('时长必须为有限非负秒数；0 为持续运行')
    return number


def firmware_options(parser):
    parser.add_argument('--preset', default='h7-debug', help='CMake preset，默认 h7-debug')
    parser.add_argument('--dry-run', action='store_true', help='仅显示操作')


def probe_options(parser):
    parser.add_argument('--cfg', '--config', default=str(DEFAULT_CFG))
    parser.add_argument('--openocd', default='openocd')
    parser.add_argument('--backend', choices=('dapv2', 'dapv1', 'auto'), default='dapv2',
                        help='默认 dapv2 (USB bulk)；ST-LINK 配置需使用 auto')
    parser.add_argument('--speed', type=positive, help='SWD kHz，默认沿用板级 1800')


def main():
    parser = argparse.ArgumentParser(description='RM2027 MC-02 常用开发脚本')
    commands = parser.add_subparsers(dest='command', required=True)
    p = commands.add_parser('debug', help='启动 OpenOCD；默认不复位、不主动 halt')
    firmware_options(p)
    probe_options(p)
    p.add_argument('--port', type=port_number, default=3333)
    p.add_argument('--tcl-port', type=port_number, default=6666)
    p.add_argument('--telnet-port', type=port_number, default=4444)
    p.add_argument('--halt', action='store_true', help='启动时 reset halt')
    p.set_defaults(handler=debug)
    p = commands.add_parser('gdb', help='连接已启动的 OpenOCD')
    firmware_options(p)
    p.add_argument('--file')
    p.add_argument('--format', choices=('elf',), default='elf')
    p.add_argument('--host', default='localhost')
    p.add_argument('--port', type=port_number, default=3333)
    p.add_argument('--gdb', default='arm-none-eabi-gdb')
    p.add_argument('--load', action='store_true', help='复位并用 GDB 写入 ELF')
    p.add_argument('--reset', action='store_true', help='reset halt')
    p.add_argument('--break-main', action='store_true', help='复位后在 main 设硬件断点并 continue')
    p.set_defaults(handler=gdb)
    p = commands.add_parser('log', help='USART 文本日志，只接收不发送')
    p.add_argument('--device', '--port', help='默认自动选择唯一 Horco DAP 串口')
    p.add_argument('--baud', type=positive, default=115200)
    p.add_argument('--duration', type=duration, default=0)
    p.add_argument('--output', help='可选：保存原始串口字节，同时打印')
    p.add_argument('--dry-run', action='store_true')
    p.set_defaults(handler=serial_log)
    p = commands.add_parser('rtt', help='外部 SEGGER RTT 固件的 OpenOCD 日志')
    firmware_options(p)
    probe_options(p)
    p.set_defaults(cfg=os.environ.get('OPENOCD_CFG', str(DEFAULT_CFG)))
    p.add_argument('--file', help='含 _SEGGER_RTT 的 ELF')
    p.add_argument('--host', default=os.environ.get('OPENOCD_HOST', '127.0.0.1'))
    p.add_argument('--tcl-port', type=port_number, default=int(os.environ.get('OPENOCD_TCL_PORT', '6666')))
    p.add_argument('--telnet-port', type=port_number, default=int(os.environ.get('OPENOCD_TELNET_PORT', '4444')))
    p.add_argument('--rtt-port', type=port_number, default=int(os.environ.get('RTT_SERVER_PORT', '19021')))
    p.add_argument('--channel', type=int, default=int(os.environ.get('RTT_CHANNEL', '0')))
    p.add_argument('--rtt-address', type=integer, default=os.environ.get('RTT_RAM_ADDR'))
    p.add_argument('--rtt-size', type=integer, default=os.environ.get('RTT_RAM_SIZE'))
    p.add_argument('--rtt-name', default=os.environ.get('RTT_NAME', 'SEGGER RTT'))
    p.add_argument('--reset-run', type=int, choices=(0, 1), default=int(os.environ.get('OPENOCD_RESET_RUN', '0')))
    p.add_argument('--keep-openocd', action='store_true', default=os.environ.get('OPENOCD_KEEP_RUNNING') == '1')
    p.add_argument('--duration', type=duration, default=0)
    p.set_defaults(handler=rtt)
    args = parser.parse_args()
    if args.command == 'rtt':
        if args.channel < 0 or (args.rtt_size is not None and args.rtt_size <= 0):
            parser.error('RTT channel 必须非负，size 必须为正数')
        if args.rtt_address is not None and not 0 <= args.rtt_address <= 0xffffffff:
            parser.error('RTT 地址必须为 uint32')
    try:
        args.handler(args)
    except KeyboardInterrupt:
        return 130
    except subprocess.CalledProcessError as exc:
        return exc.returncode if exc.returncode > 0 else 1
    except (ValueError, OSError) as exc:
        print(f'错误: {exc}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
