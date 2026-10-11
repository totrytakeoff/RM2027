#!/bin/bash
# =============================================================================
# @file scripts/gdb.sh
# @brief STM32 GDB 连接与固件加载脚本
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 自动定位 build 目录下的 ELF、连接本地 OpenOCD 服务器并在 main 处断点运行，简化调试流程。
# =============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
DEFAULT_FILE="output/app"
DEFAULT_FORMAT="elf"
DEFAULT_PORT="3333"
DEFAULT_GDB="arm-none-eabi-gdb"

usage() {
    cat <<EOF
用法: $(basename "$0") [--file <path-without-ext>] [--format elf] [--port <port>] [--gdb <path>]
示例:
  $(basename "$0")                      # 使用默认文件连接GDB
  $(basename "$0") --file tests/imu/test_imu --port 3333
EOF
}

FILE="$DEFAULT_FILE"
FORMAT="$DEFAULT_FORMAT"
PORT="$DEFAULT_PORT"
GDB_BIN="$DEFAULT_GDB"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --file) FILE="$2"; shift 2;;
        --format) FORMAT="$2"; shift 2;;
        --port) PORT="$2"; shift 2;;
        --gdb) GDB_BIN="$2"; shift 2;;
        -h|--help) usage; exit 0;;
        *) echo "未知参数: $1"; usage; exit 1;;
    esac
done

if [[ "$FORMAT" != "elf" ]]; then
    echo "GDB调试必须使用elf格式"; exit 1
fi

resolve_file() {
    local path="$1"
    # 相对路径自动指向 build 下（不含扩展名）
    [[ "$path" == /* || "$path" == "~/"* ]] && echo "$path" && return
    echo "${BUILD_DIR}/${path}.${FORMAT}"
}

TARGET_FILE="$(resolve_file "$FILE")"
if [ ! -f "$TARGET_FILE" ]; then
    echo "未找到文件: $TARGET_FILE"
    exit 1
fi

if ! command -v "$GDB_BIN" &> /dev/null; then
    echo "GDB不可用: $GDB_BIN"
    exit 1
fi

echo "GDB    : $GDB_BIN"
echo "Target : $TARGET_FILE"
echo "Port   : $PORT"
echo "启动GDB并连接到OpenOCD服务器..."

# 创建临时GDB脚本文件
GDB_SCRIPT=$(mktemp)
trap "rm -f $GDB_SCRIPT" EXIT

# 写入GDB命令
cat > "$GDB_SCRIPT" << EOF
target remote localhost:$PORT
monitor reset init
load $TARGET_FILE
monitor reset init
# 设置断点在main函数
break main
continue
EOF

# 启动GDB并执行脚本
"$GDB_BIN" -x "$GDB_SCRIPT" "$TARGET_FILE"
