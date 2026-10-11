#!/bin/bash
# =============================================================================
# @file scripts/build.sh
# @brief RM2026 构建与上传入口脚本
# @project RM2026
# @author YZ-Control/myself
# @version 1.0.0
# @date 2026-07-19
# @details 提供 clean/configure/build/upload 等命令封装，统一调用 CMake 与 Toolchain。
# =============================================================================

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="${ROOT_DIR}/build"
TOOLCHAIN_FILE="${ROOT_DIR}/cmake/Toolchain.cmake"

# 彩色输出
RED='\033[1;31m'; GREEN='\033[1;32m'; YELLOW='\033[1;33m'; BLUE='\033[1;34m'; CYAN='\033[1;36m'; NC='\033[0m'
msg() { echo -e "${1}${2}${NC}"; }

usage() {
    msg "$CYAN" "RM2026 构建脚本"
    echo "用法: $(basename "$0") [clean|configure|build|upload|upload-verbose|verify] [--target <name>]"
    echo "默认 target: app.elf"
    echo "示例: $(basename "$0") build --target test_imu"
}

ensure_configure() {
    # 首次或目录缺失时执行 CMake 配置，使用统一 Toolchain
    if [ ! -d "${BUILD_DIR}/CMakeFiles" ]; then
        msg "$CYAN" "首次配置 CMake..."
        cmake -S "${ROOT_DIR}" -B "${BUILD_DIR}" -DCMAKE_TOOLCHAIN_FILE="${TOOLCHAIN_FILE}"
    fi
}

clean() {
    msg "$CYAN" "清理构建目录 ${BUILD_DIR}"
    rm -rf "${BUILD_DIR}"
}

build() {
    # 先确保已配置，然后构建指定 target（默认 app.elf）
    local target=${1:-app.elf}
    ensure_configure
    msg "$CYAN" "构建目标: ${target}"
    cmake --build "${BUILD_DIR}" --target "${target}"
    if [ "${target}" = "app.elf" ]; then
        msg "$BLUE" "编译大小:"
        arm-none-eabi-size "${BUILD_DIR}/output/app.elf" || true
    fi
}

upload() {
    ensure_configure
    msg "$CYAN" "执行上传..."
    cmake --build "${BUILD_DIR}" --target upload
}

upload_verbose() {
    ensure_configure
    msg "$CYAN" "执行详细上传..."
    cmake --build "${BUILD_DIR}" --target upload-verbose
}

verify() {
    ensure_configure
    msg "$CYAN" "验证固件..."
    cmake --build "${BUILD_DIR}" --target verify
}

cmd=${1:-build}
shift $(( $# > 0 ? 1 : 0 ))
target=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --target)
            target=$2; shift 2;;
        *)
            msg "$RED" "未知参数: $1"; usage; exit 1;;
    esac
done

case "$cmd" in
    clean) clean ;;
    configure) ensure_configure ;;
    build) build "${target:-app.elf}" ;;
    upload) build "${target:-app.elf}"; upload ;;
    upload-verbose) build "${target:-app.elf}"; upload_verbose ;;
    verify) build "${target:-app.elf}"; verify ;;
    help|-h|--help) usage ;;
    *) msg "$RED" "未知命令: ${cmd}"; usage; exit 1 ;;
esac
