#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

OPENOCD_CFG="${OPENOCD_CFG:-$ROOT_DIR/platform/boards/rm_f407/openocd/openocd_dap.cfg}"

OPENOCD_HOST="${OPENOCD_HOST:-127.0.0.1}"
OPENOCD_TELNET_PORT="${OPENOCD_TELNET_PORT:-4444}"
OPENOCD_TCL_PORT="${OPENOCD_TCL_PORT:-6666}"
OPENOCD_RESET_RUN="${OPENOCD_RESET_RUN:-1}"
OPENOCD_KEEP_RUNNING="${OPENOCD_KEEP_RUNNING:-0}"

# STM32F407 RAM: 0x2000_0000, 128KB
RTT_RAM_ADDR="${RTT_RAM_ADDR:-0x20000000}"
RTT_RAM_SIZE="${RTT_RAM_SIZE:-0x20000}"
RTT_NAME="${RTT_NAME:-SEGGER RTT}"

RTT_SERVER_PORT="${RTT_SERVER_PORT:-19021}"
RTT_CHANNEL="${RTT_CHANNEL:-0}"

OPENOCD_PID=""

need_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "missing required command: $1" >&2
    exit 1
  fi
}

port_open() {
  # shellcheck disable=SC2312
  (exec 3<>"/dev/tcp/${OPENOCD_HOST}/${1}") >/dev/null 2>&1
}

wait_port() {
  local port="$1"
  local label="$2"
  local tries=0
  until port_open "${port}"; do
    tries=$((tries + 1))
    if (( tries > 100 )); then
      echo "[rtt] timeout waiting for ${label} on ${OPENOCD_HOST}:${port}" >&2
      return 1
    fi
    sleep 0.05
  done
  return 0
}

cleanup() {
  if [[ "${OPENOCD_KEEP_RUNNING}" == "1" ]]; then
    return 0
  fi
  if [[ -n "${OPENOCD_PID}" ]]; then
    kill "${OPENOCD_PID}" >/dev/null 2>&1 || true
    wait "${OPENOCD_PID}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

start_openocd_if_needed() {
  if port_open "${OPENOCD_TELNET_PORT}"; then
    echo "[rtt] OpenOCD already running on ${OPENOCD_HOST}:${OPENOCD_TELNET_PORT}"
    return 0
  fi

  need_cmd openocd
  if [[ ! -f "${OPENOCD_CFG}" ]]; then
    echo "[rtt] OpenOCD cfg not found: ${OPENOCD_CFG}" >&2
    exit 1
  fi

  echo "[rtt] starting OpenOCD: openocd -f ${OPENOCD_CFG}"
  openocd -f "${OPENOCD_CFG}" >/tmp/openocd-rtt.log 2>&1 &
  OPENOCD_PID="$!"

  if ! wait_port "${OPENOCD_TELNET_PORT}" "OpenOCD telnet"; then
    echo "[rtt] OpenOCD did not open telnet port ${OPENOCD_TELNET_PORT}" >&2
    echo "[rtt] last log lines:" >&2
    tail -n 80 /tmp/openocd-rtt.log >&2 || true
    exit 1
  fi
  if [[ "${OPENOCD_KEEP_RUNNING}" == "1" ]]; then
    echo "[rtt] keep-openocd enabled: process will stay after exit"
  fi
}

send_openocd_telnet_cmds() {
  local cmds="$1"

  if command -v python3 >/dev/null 2>&1; then
    printf "%s" "${cmds}" | python3 - "${OPENOCD_HOST}" "${OPENOCD_TELNET_PORT}" <<'PY'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
cmds = sys.stdin.read()

s = socket.create_connection((host, port), timeout=2.0)
s.settimeout(1.0)
time.sleep(0.05)
try:
    s.recv(4096)
except Exception:
    pass

s.sendall(cmds.encode("utf-8"))
time.sleep(0.05)

out = b""
try:
    while True:
        chunk = s.recv(4096)
        if not chunk:
            break
        out += chunk
        if b"\n>" in out or b"unknown command" in out:
            break
except Exception:
    pass

s.close()
sys.stdout.write(out.decode("utf-8", errors="ignore"))
PY
    return 0
  fi

  need_cmd nc
  printf "%s" "${cmds}" | nc "${OPENOCD_HOST}" "${OPENOCD_TELNET_PORT}" || true
}

send_openocd_tcl_cmd() {
  local cmd="$1"

  # OpenOCD TCL RPC uses Ctrl-Z (0x1a) as message terminator.
  if command -v python3 >/dev/null 2>&1; then
    python3 - "${OPENOCD_HOST}" "${OPENOCD_TCL_PORT}" "${cmd}" <<'PY'
import socket
import sys

host = sys.argv[1]
port = int(sys.argv[2])
cmd = sys.argv[3]
if not cmd.endswith("\n"):
    cmd += "\n"

term = b"\x1a"
payload = cmd.encode("utf-8") + term

s = socket.create_connection((host, port), timeout=2.0)
s.settimeout(2.0)
s.sendall(payload)

out = b""
while True:
    chunk = s.recv(4096)
    if not chunk:
        break
    out += chunk
    if term in out:
        break
s.close()

out = out.split(term)[0]
sys.stdout.write(out.decode("utf-8", errors="ignore"))
PY
    return 0
  fi

  need_cmd nc
  # Best-effort fallback: may not work reliably without Ctrl-Z framing.
  printf "%s\n\x1a" "${cmd}" | nc "${OPENOCD_HOST}" "${OPENOCD_TCL_PORT}" || true
}

reset_run_if_enabled() {
  if [[ "${OPENOCD_RESET_RUN}" != "1" ]]; then
    return 0
  fi
  echo "[rtt] reset run (OPENOCD_RESET_RUN=1)"
  send_openocd_tcl_cmd $'reset run' >/dev/null || true
  sleep 0.2
}

setup_rtt() {
  echo "[rtt] configuring RTT (addr=${RTT_RAM_ADDR}, size=${RTT_RAM_SIZE}, name=${RTT_NAME})"
  local out
  out="$(send_openocd_tcl_cmd $'rtt setup '"${RTT_RAM_ADDR}"$' '"${RTT_RAM_SIZE}"$' "'"${RTT_NAME}"$'"\nrtt start\nrtt server start '"${RTT_SERVER_PORT}"$' '"${RTT_CHANNEL}"$'\n')"
  if [[ -n "${out}" ]]; then
    printf "%s\n" "${out}"
  fi

  if echo "${out}" | grep -Eqi "unknown command|invalid command name \"rtt\"|no such command"; then
    echo "[rtt] ERROR: your OpenOCD does not support RTT commands (missing rtt)." >&2
    echo "[rtt] Try a newer OpenOCD build with RTT enabled, or switch logs to UART/SWO." >&2
    exit 1
  fi

  # Some OpenOCD builds return empty output even on success; verify by checking the RTT port.
}

connect_rtt() {
  need_cmd nc
  if ! wait_port "${RTT_SERVER_PORT}" "RTT server"; then
    echo "[rtt] RTT server port ${RTT_SERVER_PORT} is not listening." >&2
    echo "[rtt] last OpenOCD log lines:" >&2
    tail -n 120 /tmp/openocd-rtt.log >&2 || true
    echo "[rtt] tip: try 'telnet ${OPENOCD_HOST} ${OPENOCD_TELNET_PORT}' then run 'help rtt' to see if RTT is supported." >&2
    exit 1
  fi
  echo "[rtt] connecting: nc ${OPENOCD_HOST} ${RTT_SERVER_PORT}  (Ctrl-C to exit)"
  nc "${OPENOCD_HOST}" "${RTT_SERVER_PORT}" || true
}

main() {
  start_openocd_if_needed
  reset_run_if_enabled
  setup_rtt
  connect_rtt
}

main "$@"
