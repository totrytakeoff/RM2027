#!/bin/bash
# =============================================================================
# @file scripts/upload.sh
# @brief STM32 OpenOCD 烧录脚本
# @project RM2027 / DM MC-02 (adapted from RM2026)
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 解析 build 目录产物路径，调用 OpenOCD 进行 flash/verify，可定制地址、格式和配置文件。
# =============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
PRESET="h7-debug"
OUTPUT_DIR="${BUILD_DIR}/${PRESET}"
DEFAULT_FILE="chassis_demo"
DEFAULT_FORMAT="bin"
DEFAULT_ADDR="0x08000000"
DEFAULT_CFG="${ROOT_DIR}/platform/boards/dm_mc02/openocd/cmsis-dap.cfg"

usage() {
    cat <<EOF
用法: $(basename "$0") [<name>] [--file <path>] [--format bin|hex|elf] [--addr <0x...>] [--cfg <openocd.cfg>] [--openocd <path>] [--verify-only] [--dry-run]
可选: --preset h7-debug|h7-release|h7-dbus-debug|h7-chassis-observe
      --backend dapv2|dapv1|auto  --speed <kHz>  --verbose
默认: chassis_demo.bin，h7-debug，DAP v2；只需 Bash 和 OpenOCD，不调用 Python。
说明: BIN 默认写入 0x08000000；HEX/ELF 默认使用文件内地址，--addr 对其表示地址偏移。
示例:
  $(basename "$0") chassis_demo           # 烧录 build/h7-debug/chassis_demo.bin
  $(basename "$0") board_bringup --format elf
  $(basename "$0") chassis_demo --preset h7-release
  $(basename "$0") --format hex --verify-only
  $(basename "$0") build/h7-debug/chassis_demo.elf
  $(basename "$0") /abs/path/to/custom.elf     # 直接指定绝对路径并保持扩展名
EOF
}

FILE="$DEFAULT_FILE"
FORMAT="$DEFAULT_FORMAT"
ADDR=""
CFG="$DEFAULT_CFG"
OPENOCD_BIN=""
VERIFY_ONLY=false
DRY_RUN=false
POSITIONAL_FILE=""
BACKEND="dapv2"
SPEED=""
VERBOSE=false

need_value() {
    if [[ $# -lt 2 || -z "$2" ]]; then
        echo "参数缺少值: $1" >&2; exit 1
    fi
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preset) need_value "$@"; PRESET="$2"; shift 2;;
        --backend) need_value "$@"; BACKEND="$2"; shift 2;;
        --speed) need_value "$@"; SPEED="$2"; shift 2;;
        --verbose) VERBOSE=true; shift;;
        --file) need_value "$@"; FILE="$2"; shift 2;;
        --format) need_value "$@"; FORMAT="$2"; shift 2;;
        --addr|--address) need_value "$@"; ADDR="$2"; shift 2;;
        --cfg|--config) need_value "$@"; CFG="$2"; shift 2;;
        --openocd) need_value "$@"; OPENOCD_BIN="$2"; shift 2;;
        --verify-only) VERIFY_ONLY=true; shift;;
        --dry-run) DRY_RUN=true; shift;;
        -h|--help) usage; exit 0;;
        --*) echo "未知参数: $1" >&2; exit 1;;
        *)
            if [[ -z "$POSITIONAL_FILE" ]]; then
                POSITIONAL_FILE="$1"
                shift
            else
                echo "未知参数: $1"; usage; exit 1
            fi
            ;;
    esac
done

case "$PRESET" in
    h7-debug|h7-release|h7-chassis-observe) ;;
    h7-dbus-debug) [[ "$FILE" != "$DEFAULT_FILE" || -n "$POSITIONAL_FILE" ]] || FILE="board_bringup";;
    *) echo "不支持的固件 preset: $PRESET" >&2; exit 1;;
esac
OUTPUT_DIR="${BUILD_DIR}/${PRESET}"
case "$BACKEND" in dapv2|dapv1|auto) ;; *) echo "backend 必须为 dapv2|dapv1|auto" >&2; exit 1;; esac
if [[ -n "$SPEED" && ! "$SPEED" =~ ^[1-9][0-9]*$ ]]; then
    echo "speed 必须为正整数 kHz" >&2; exit 1
fi
if [[ -n "$ADDR" && ! "$ADDR" =~ ^(0[xX][0-9a-fA-F]+|[0-9]+)$ ]]; then
    echo "地址必须为十进制或十六进制整数" >&2; exit 1
fi

if [[ -n "$POSITIONAL_FILE" ]]; then
    FILE="$POSITIONAL_FILE"
fi

if [[ "$FORMAT" != "bin" && "$FORMAT" != "hex" && "$FORMAT" != "elf" ]]; then
    echo "格式必须为 bin|hex|elf"; exit 1
fi

if [[ "$CFG" == "~/"* ]]; then
    CFG="${HOME}/${CFG:2}"
elif [[ "$CFG" != /* ]]; then
    CFG="${ROOT_DIR}/${CFG}"
fi

resolve_file() {
    local path="$1"
    local expanded="$path"
    local relative_candidate

    if [[ "$expanded" == "~/"* ]]; then
        expanded="${HOME}/${expanded:2}"
    fi

    if [[ "$expanded" == /* ]]; then
        [[ "${expanded##*/}" == *.* ]] && echo "$expanded" || echo "${expanded}.${FORMAT}"
        return
    fi

    # 如果参数已经指向当前工作目录中的文件，直接使用它。这样既支持
    # build/output/app.bin，也保留仅传 app 或 test_xxx 的快捷形式。
    relative_candidate="$expanded"
    [[ "${relative_candidate##*/}" == *.* ]] || relative_candidate="${relative_candidate}.${FORMAT}"
    if [[ -f "$relative_candidate" ]]; then
        echo "$(cd "$(dirname "$relative_candidate")" && pwd)/$(basename "$relative_candidate")"
        return
    fi

    if [[ -f "${ROOT_DIR}/${relative_candidate}" ]]; then
        echo "${ROOT_DIR}/${relative_candidate}"; return
    fi
    if [[ "$expanded" == */* ]]; then
        echo "${ROOT_DIR}/${relative_candidate}"; return
    fi

    if [[ "$expanded" == *.* ]]; then
        echo "${OUTPUT_DIR}/${expanded}"
    else
        echo "${OUTPUT_DIR}/${expanded}.${FORMAT}"
    fi
}

TARGET_FILE="$(resolve_file "$FILE")"
if [ ! -f "$TARGET_FILE" ]; then
    echo "未找到文件: $TARGET_FILE"
    exit 1
fi

IMAGE_TYPE="${TARGET_FILE##*.}"
IMAGE_TYPE="${IMAGE_TYPE,,}"
if [[ "$IMAGE_TYPE" != "bin" && "$IMAGE_TYPE" != "hex" && "$IMAGE_TYPE" != "elf" ]]; then
    echo "无法识别镜像类型: $TARGET_FILE"; exit 1
fi
if [[ "$IMAGE_TYPE" == "bin" && -z "$ADDR" ]]; then
    ADDR="$DEFAULT_ADDR"
fi

if [ -z "$OPENOCD_BIN" ]; then
    OPENOCD_BIN="$(command -v openocd || true)"
    [ -z "$OPENOCD_BIN" ] && OPENOCD_BIN="${HOME}/.platformio/packages/tool-openocd/bin/openocd"
fi
if [[ "$OPENOCD_BIN" != */* ]]; then
    OPENOCD_BIN="$(command -v "$OPENOCD_BIN" || true)"
fi

if [ ! -x "$OPENOCD_BIN" ]; then
    echo "OpenOCD 不可用: $OPENOCD_BIN"
    exit 1
fi

if [ ! -f "$CFG" ]; then
    echo "OpenOCD 配置不存在: $CFG"
    exit 1
fi

# OpenOCD -c interprets Tcl: quote paths independently from shell arguments.
tcl_quote() {
    local value="$1" escaped="" character i
    for ((i=0; i<${#value}; i++)); do
        character="${value:i:1}"
        case "$character" in
            '\'|'"'|'$'|'['|']'|'{'|'}') escaped+="\\$character";;
            $'\n') escaped+='\n';;
            $'\r') escaped+='\r';;
            *) escaped+="$character";;
        esac
    done
    printf '"%s"' "$escaped"
}
IMAGE_SPEC="$(tcl_quote "$TARGET_FILE")"
if [[ -n "$ADDR" ]]; then
    IMAGE_SPEC+=" $ADDR $IMAGE_TYPE"
fi

CMD=("$OPENOCD_BIN" -f "$CFG")
case "$BACKEND" in
    dapv2) CMD+=(-c "cmsis_dap_backend usb_bulk");;
    dapv1) CMD+=(-c "cmsis_dap_backend hid");;
esac
[[ -z "$SPEED" ]] || CMD+=(-c "adapter speed $SPEED")
if $VERBOSE; then CMD+=(-d 3); fi
if $VERIFY_ONLY; then
    OPERATION="verify_image $IMAGE_SPEC"
else
    OPERATION="flash write_image erase $IMAGE_SPEC; verify_image $IMAGE_SPEC"
fi
CMD+=(-c "init; reset halt; if {[catch {$OPERATION} flash_error]} {puts stderr \$flash_error; shutdown error}; reset run; shutdown")

echo "OpenOCD: $OPENOCD_BIN"
echo "Config : $CFG"
echo "File   : $TARGET_FILE"
echo "Addr   : ${ADDR:-<from-image>}"
echo "Backend: $BACKEND"
if $DRY_RUN; then
    printf 'Command:'
    printf ' %q' "${CMD[@]}"
    echo
    exit 0
fi

"${CMD[@]}"
