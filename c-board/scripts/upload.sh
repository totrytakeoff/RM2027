#!/bin/bash
# =============================================================================
# @file scripts/upload.sh
# @brief STM32 OpenOCD 烧录脚本
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 解析 build 目录产物路径，调用 OpenOCD 进行 flash/verify，可定制地址、格式和配置文件。
# =============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
OUTPUT_DIR="${BUILD_DIR}/output"
DEFAULT_FILE="app"
DEFAULT_FORMAT="bin"
DEFAULT_ADDR="0x08000000"
DEFAULT_CFG="${ROOT_DIR}/platform/boards/rm_f407/openocd/openocd_dap.cfg"

usage() {
    cat <<EOF
用法: $(basename "$0") [<name>] [--format bin|hex|elf] [--addr <0x...>] [--cfg <openocd.cfg>] [--openocd <path>] [--verify-only] [--dry-run]
说明: BIN 默认写入 0x08000000；HEX/ELF 默认使用文件内地址，--addr 对其表示地址偏移。
示例:
  $(basename "$0")                      # 烧录 build/output/app.bin 到 0x08000000
  $(basename "$0") test_motor --format elf
  $(basename "$0") --format hex --verify-only  # 默认使用 app.hex
  $(basename "$0") test_remote_control_demo    # 上传 build/output/test_remote_control_demo.bin
  $(basename "$0") build/output/app.bin        # 直接使用当前目录下的相对路径
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

while [[ $# -gt 0 ]]; do
    case "$1" in
        --file) FILE="$2"; shift 2;;
        --format) FORMAT="$2"; shift 2;;
        --addr|--address) ADDR="$2"; shift 2;;
        --cfg|--config) CFG="$2"; shift 2;;
        --openocd) OPENOCD_BIN="$2"; shift 2;;
        --verify-only) VERIFY_ONLY=true; shift;;
        --dry-run) DRY_RUN=true; shift;;
        -h|--help) usage; exit 0;;
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

if [[ -n "$POSITIONAL_FILE" ]]; then
    FILE="$POSITIONAL_FILE"
fi

if [[ "$FORMAT" != "bin" && "$FORMAT" != "hex" && "$FORMAT" != "elf" ]]; then
    echo "格式必须为 bin|hex|elf"; exit 1
fi

if [[ "$CFG" != /* && "$CFG" != "~/"* ]]; then
    CFG="${ROOT_DIR}/${CFG}"
fi

resolve_file() {
    local path="$1"
    local expanded="$path"
    local relative_candidate

    if [[ "$expanded" == "~/"* ]]; then
        expanded="${HOME}/${expanded#~/}"
    fi

    if [[ "$expanded" == /* ]]; then
        [[ "$expanded" == *.* ]] && echo "$expanded" || echo "${expanded}.${FORMAT}"
        return
    fi

    # 如果参数已经指向当前工作目录中的文件，直接使用它。这样既支持
    # build/output/app.bin，也保留仅传 app 或 test_xxx 的快捷形式。
    relative_candidate="$expanded"
    [[ "$relative_candidate" == *.* ]] || relative_candidate="${relative_candidate}.${FORMAT}"
    if [[ -f "$relative_candidate" ]]; then
        echo "$(cd "$(dirname "$relative_candidate")" && pwd)/$(basename "$relative_candidate")"
        return
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

if [ ! -x "$OPENOCD_BIN" ]; then
    echo "OpenOCD 不可用: $OPENOCD_BIN"
    exit 1
fi

if [ ! -f "$CFG" ]; then
    echo "OpenOCD 配置不存在: $CFG"
    exit 1
fi

IMAGE_SPEC="$TARGET_FILE"
if [[ -n "$ADDR" ]]; then
    IMAGE_SPEC+=" $ADDR $IMAGE_TYPE"
fi

CMD=(
    "$OPENOCD_BIN" -f "$CFG"
    -c "init; reset halt;"
)
if $VERIFY_ONLY; then
    CMD+=(-c "verify_image $IMAGE_SPEC; reset; shutdown")
else
    CMD+=(-c "flash write_image erase $IMAGE_SPEC; verify_image $IMAGE_SPEC; reset; shutdown")
fi

echo "OpenOCD: $OPENOCD_BIN"
echo "Config : $CFG"
echo "File   : $TARGET_FILE"
echo "Addr   : ${ADDR:-<from-image>}"
if $DRY_RUN; then
    printf 'Command:'
    printf ' %q' "${CMD[@]}"
    echo
    exit 0
fi

"${CMD[@]}"
