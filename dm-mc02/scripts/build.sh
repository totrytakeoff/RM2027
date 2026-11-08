#!/usr/bin/env bash
# RM2026 build.sh workflow by YZ-Control/myself, adapted to MC-02 presets.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ACTION="${1:-build}"
[[ $# -eq 0 ]] || shift
PRESET="h7-debug"
TARGET=""
JOBS=""
DRY_RUN=false
UPLOAD_OPTIONS=()
usage() {
    echo "用法: build.sh [clean|configure|build|upload|upload-verbose|verify] [--target name] [--preset name] [-j N] [--dry-run]"
    echo "默认 chassis_demo.elf / h7-debug；upload 使用原生 upload.sh，不需要 Python。"
}
while [[ $# -gt 0 ]]; do
    case "$1" in
        --target|--preset|-j|--jobs|--backend|--cfg|--config|--openocd|--speed)
            [[ $# -ge 2 && -n "$2" ]] || { echo "参数缺少值: $1" >&2; exit 1; }
            case "$1" in
                --target) TARGET="$2";;
                --preset) PRESET="$2";;
                -j|--jobs) JOBS="$2";;
                *) UPLOAD_OPTIONS+=("$1" "$2");;
            esac
            shift 2;;
        --dry-run) DRY_RUN=true; shift;;
        -h|--help) usage; exit 0;;
        *) echo "未知参数: $1" >&2; exit 1;;
    esac
done
case "$ACTION" in help|-h|--help) usage; exit 0;; clean|configure|build|upload|upload-verbose|verify) ;; *) usage; exit 1;; esac
case "$PRESET" in h7-debug|h7-release|h7-dbus-debug|h7-chassis-observe|h7-mecanum-observe|host) ;; *) echo "未知 preset: $PRESET" >&2; exit 1;; esac
if [[ -z "$TARGET" ]]; then
    case "$PRESET" in host) TARGET=all;; h7-mecanum-observe) TARGET=mecanum_demo.elf;; h7-dbus-debug) TARGET=board_bringup.elf;; *) TARGET=chassis_demo.elf;; esac
elif [[ "$PRESET" != host && "$TARGET" != all && "$TARGET" != *.elf ]]; then
    TARGET+=".elf"
fi
[[ -z "$JOBS" || "$JOBS" =~ ^[1-9][0-9]*$ ]] || { echo "jobs 必须为正整数" >&2; exit 1; }
if [[ "$ACTION" == upload* || "$ACTION" == verify ]]; then
    [[ "$PRESET" != host && "$TARGET" != all ]] || { echo "上传需要指定固件目标" >&2; exit 1; }
fi
run() {
    printf 'Command:'; printf ' %q' "$@"; printf '\n'
    if ! $DRY_RUN; then "$@"; fi
}
cd "$ROOT_DIR"
if [[ "$ACTION" == clean ]]; then
    run rm -rf -- "$ROOT_DIR/build/$PRESET"
    exit 0
fi
run cmake --preset "$PRESET"
[[ "$ACTION" != configure ]] || exit 0
BUILD_COMMAND=(cmake --build --preset "$PRESET" --target "$TARGET")
[[ -z "$JOBS" ]] || BUILD_COMMAND+=(--parallel "$JOBS")
run "${BUILD_COMMAND[@]}"
if [[ "$ACTION" == upload* || "$ACTION" == verify ]]; then
    UPLOAD_COMMAND=("$ROOT_DIR/scripts/upload.sh" "${TARGET%.elf}" --preset "$PRESET")
    [[ "$ACTION" != upload-verbose ]] || UPLOAD_COMMAND+=(--verbose)
    [[ "$ACTION" != verify ]] || UPLOAD_COMMAND+=(--verify-only)
    UPLOAD_COMMAND+=("${UPLOAD_OPTIONS[@]}")
    run "${UPLOAD_COMMAND[@]}"
fi
