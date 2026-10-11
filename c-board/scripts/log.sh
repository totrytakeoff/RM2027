#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RTT_SCRIPT="${ROOT_DIR}/scripts/openocd_rtt_log.sh"

OPENOCD_CFG="${ROOT_DIR}/platform/boards/rm_f407/openocd/openocd_dap.cfg"
OPENOCD_RESET_RUN=1
RTT_SERVER_PORT=19021
RTT_CHANNEL=0
OPENOCD_KEEP_RUNNING=0

usage() {
  cat <<EOF
用法: $(basename "$0") [options]

默认行为:
  - 自动复用或启动 OpenOCD
  - 自动配置 RTT 并连接日志输出
  - Ctrl+C 后仅清理由本次脚本拉起的 OpenOCD

选项:
  --cfg <path>         OpenOCD 配置文件 (默认: platform/boards/rm_f407/openocd/openocd_dap.cfg)
  --reset-run <0|1>    连接前是否 reset run (默认: 1)
  --rtt-port <port>    RTT server 端口 (默认: 19021)
  --channel <id>       RTT 通道号 (默认: 0)
  --keep-openocd       即便脚本拉起 OpenOCD，退出时也不关闭
  -h, --help           显示帮助
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --cfg)
      OPENOCD_CFG="$2"
      shift 2
      ;;
    --reset-run)
      OPENOCD_RESET_RUN="$2"
      shift 2
      ;;
    --rtt-port)
      RTT_SERVER_PORT="$2"
      shift 2
      ;;
    --channel)
      RTT_CHANNEL="$2"
      shift 2
      ;;
    --keep-openocd)
      OPENOCD_KEEP_RUNNING=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ "${OPENOCD_RESET_RUN}" != "0" && "${OPENOCD_RESET_RUN}" != "1" ]]; then
  echo "--reset-run 仅支持 0 或 1" >&2
  exit 1
fi

if [[ ! -f "${RTT_SCRIPT}" ]]; then
  echo "未找到脚本: ${RTT_SCRIPT}" >&2
  exit 1
fi

if [[ "${OPENOCD_CFG}" != /* && "${OPENOCD_CFG}" != "~/"* ]]; then
  OPENOCD_CFG="${ROOT_DIR}/${OPENOCD_CFG}"
fi
if [[ "${OPENOCD_CFG}" == "~/"* ]]; then
  OPENOCD_CFG="${HOME}/${OPENOCD_CFG#~/}"
fi

export OPENOCD_CFG
export OPENOCD_RESET_RUN
export RTT_SERVER_PORT
export RTT_CHANNEL
export OPENOCD_KEEP_RUNNING

exec "${RTT_SCRIPT}"
