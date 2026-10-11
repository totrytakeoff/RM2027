# OpenOCD RTT 一键日志（推荐）

文档更新日期：2026-07-23

## 推荐用法（一条命令）

```bash
cd /path/to/RM2026
./scripts/log.sh
```

默认行为：
1. 自动复用或启动 OpenOCD（`4444`）
2. 自动执行 `rtt setup/start/server start`
3. 自动连接 RTT 日志输出（`19021`）
4. `Ctrl+C` 退出时，仅关闭本次脚本拉起的 OpenOCD；外部已有实例不受影响

## 常用参数

```bash
./scripts/log.sh --cfg platform/boards/rm_f407/openocd/openocd_dap.cfg
./scripts/log.sh --reset-run 0
./scripts/log.sh --rtt-port 19021 --channel 0
./scripts/log.sh --keep-openocd
```

## 串口资源规范

1. 全工程基础日志默认走 RTT：`BSP_LOG_USE_UART=0`。
2. 当前 ET08 正式机器人调试固件会把应用诊断同时镜像到 RTT 与 `USART6`，串口参数为 `115200 8N1`。
3. 切换为 VT 后端前必须先迁走或关闭 `USART6` 应用诊断；配置中的编译期冲突检查会阻止两者同时占用。
4. 其他固件仅在必要时可临时回退 UART 基础日志（命令行覆盖）：

```bash
cmake -B build -S . -DBSP_LOG_USE_UART=1 -DBSP_LOG_UART_PORT=1
```

## 故障说明

若出现类似：
- `unknown command rtt`
- `invalid command name "rtt"`
- `no such command`

说明当前 OpenOCD 构建不支持 RTT，脚本会直接退出。请更换支持 RTT 的 OpenOCD 版本。

---

## 底层原理（手动流程，仅排障时使用）

1. 启动 OpenOCD：

```bash
openocd -f platform/boards/rm_f407/openocd/openocd_dap.cfg
```

2. 连接 telnet 并执行：

```text
rtt setup 0x20000000 0x20000 "SEGGER RTT"
rtt start
rtt server start 19021 0
```

3. 连接 RTT 端口查看日志：

```bash
nc localhost 19021
```
