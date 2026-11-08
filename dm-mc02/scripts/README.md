# MC-02 常用开发脚本

从 RM2026 的 9 个常用脚本迁移入口和工作流。烧录直接沿用原生 Bash / PowerShell 实现，构建使用原生 Bash；调试/日志工具使用本目录 `devtools.py`。工程根目录根据脚本位置定位，可以从其他目录调用，无需访问 RM2026 源目录。

**烧录已有固件只需要系统自带的 Bash（Windows 使用 PowerShell）和 OpenOCD，不依赖 Python、CMake 或 ARM GCC。** `build.sh` 构建另需 CMake ≥3.21、Ninja、ARM GCC，但也不依赖 Python。调试/日志工具需要 Python 3.10+；GDB 使用 `arm-none-eabi-gdb`，串口另需 pyserial。这些工具可用 `PYTHON_BIN` 指定 Python。

## 构建、烧录

```sh
./scripts/build.sh                         # 配置并构建 h7-debug/chassis_demo.elf
./scripts/build.sh build --preset h7-release
./scripts/build.sh build --target board_bringup
./scripts/build.sh build --preset h7-chassis-observe
./scripts/build.sh build --preset h7-dbus-debug  # 默认 board_bringup.elf
./scripts/build.sh build --preset host          # 默认 all，不运行测试
./scripts/build.sh configure --preset h7-debug
./scripts/build.sh clean --preset h7-debug      # 仅删除此 preset 的构建目录

./scripts/build.sh upload                  # 先构建，再烧录、校验并 reset run
./scripts/upload.sh                       # 只烧录已有 chassis_demo.bin
./scripts/upload.sh board_bringup
./scripts/upload.sh --preset h7-chassis-observe
./scripts/upload.sh --preset h7-release --format bin
./scripts/upload.sh build/h7-debug/chassis_demo.hex
./scripts/upload.sh --file /absolute/path/firmware.elf
./scripts/upload.sh --verify-only          # 不写 Flash；reset halt 校验后 reset run
./scripts/upload.sh --dry-run              # 显示完整命令，不访问板子
```

默认与原脚本一样使用 BIN 格式，固件名默认 `chassis_demo`：`upload.sh chassis_demo` 对应 `build/h7-debug/chassis_demo.bin`；`upload.sh board_bringup --format elf` 对应 `board_bringup.elf`。传入完整文件名时按文件扩展名识别格式，不需要再指定 --format。

默认烧录使用 **DAP v2 / USB bulk**、MC-02 H723 配置、1800 kHz；显式选择 `--backend dapv2`。`--backend dapv1` 可用 HID；`--backend auto` 交给 OpenOCD 默认选择，也用于 ST-LINK：

```sh
./scripts/upload.sh --cfg platform/boards/dm_mc02/openocd/stlink.cfg --backend auto
./scripts/upload.sh --backend dapv2 --speed 1000 --openocd /path/to/openocd
```

ELF/HEX 使用内置地址，BIN 默认写入 `0x08000000`。`--addr` 对 BIN 为基地址，对 ELF/HEX 为偏移。文件简称按 `build/<preset>/` 解析，已有相对文件优先按调用目录/工程目录解析；带扩展名的文件按其自身格式处理。Bash 和 PowerShell 脚本分别处理路径转义，保留原脚本的 --file/--format/--addr/--cfg/--openocd/--verify-only/--dry-run 参数。

`build.sh upload-verbose` 开启 OpenOCD 调试输出；`build.sh verify` 先构建再只校验。`--target` 对这些操作始终选中对应固件，不会像旧入口一样退回固定的 app 镜像。

PowerShell 入口：

```powershell
./scripts/upload.ps1 --preset h7-debug --backend dapv2
./scripts/upload.ps1 --format bin --dry-run
```

## 麦轮目标

```sh
./scripts/build.sh build --target mecanum_demo
./scripts/upload.sh mecanum_demo
./scripts/build.sh upload --target mecanum_demo
./scripts/build.sh build --preset h7-mecanum-observe
./scripts/upload.sh --preset h7-mecanum-observe
```

默认目标仍为全向轮，麦轮需要显式指定，功率模型与接线见 [麦轮指南](../docs/guides/mecanum_demo.md)。

## OpenOCD / GDB

两个终端分别执行：

```sh
./scripts/debug.sh                  # 启动服务器，不主动复位/halt
./scripts/gdb.sh                    # 使用已有 ELF 连接服务器，不自动 load
```

`debug.sh --port 3334 --tcl-port 6667 --telnet-port 4445` 的端口实际传给 OpenOCD；GDB 对应 `--port 3334`。`debug.sh --halt` 显式 reset halt；`gdb.sh --reset` 复位暂停，`--load` 复位并写入 ELF，`--break-main` 复位后停在 main，三者都需明确指定。

GDB 连接会暂停目标。H7 的 IWDG 与 TIM5/TIM6 在暂停调试时需要考虑现有时间监督机制；继续后进行 reset run 可恢复完整启动时序。读取正常运行状态可以启动 `debug.sh`，再使用现有 `tools/board_probe.py`，它只通过 Tcl 读取而不暂停 CPU。脚本没有自动改写看门狗冻结寄存器。

## 日志

麦轮 demo 的 USART1 已改为 VOFA+ JustFloat 二进制输出，使用VOFA+接收，通道表见 [麦轮指南](../docs/guides/mecanum_demo.md)。以下 `log.sh` 用于文本日志。

本框架的日志传输是 USART，`log.sh` 默认只接收串口，自动选择唯一 Horco DAP 串口，115200 8N1：

```sh
./scripts/log.sh
./scripts/log.sh --device /dev/ttyACM0 --baud 115200
./scripts/log.sh --duration 30 --output /tmp/chassis_uart.txt
# Windows 可执行 python scripts/devtools.py log --device COM6
```

当前底盘 app 使用 USART1 PA9/PA10；`board_bringup` 日志使用 USART10 PE2/PE3，需按目标接线。退出使用 Ctrl+C，不发送控制字符到板子。保存文件为原始串口字节。需要捕获统计时仍可使用 `tools/serial_capture.py`。

## RTT 与 VT 工具

`openocd_rtt_log.sh` / `.ps1` 保留给带 SEGGER RTT 的固件；**当前框架没有 SEGGER RTT 控制块**。默认先检查所选 ELF 的 `_SEGGER_RTT` 符号，缺失时立即提示使用串口，不连接或复位板子。

```sh
./scripts/openocd_rtt_log.sh --file /path/to/rtt_enabled.elf
./scripts/openocd_rtt_log.sh --rtt-address 0x24000000 --rtt-size 0x50000
./scripts/openocd_rtt_log.ps1 --file /path/to/rtt_enabled.elf
```

有符号时从符号地址搜索 4096 bytes；手动地址模式默认搜索 320 KiB，覆盖本框架 AXI 应用内存，而非旧 F407 RAM。`--rtt-name`、`--channel`、`--rtt-port`、`--host`、`--tcl-port`、`--telnet-port` 可配置。默认不 reset；需要时指定 `--reset-run 1`。只清理自己启动的 OpenOCD，已有服务器不关闭，`--keep-openocd` 可保留自建服务器；启动等待和连接均有超时。保留旧环境变量 `OPENOCD_CFG/HOST/TCL_PORT/TELNET_PORT/RESET_RUN/KEEP_RUNNING`、`RTT_RAM_ADDR/RAM_SIZE/SERVER_PORT/CHANNEL/NAME`，命令行选项可覆盖（keep-openocd 为启用开关）。

`vt_serial_monitor.py` 原样迁入，是直接连接 VT03/VT13 原始协议串口的独立 PC 工具：

```sh
python3 scripts/vt_serial_monitor.py --port /dev/ttyUSB0 --baud 921600
```

它不解析 ET08 或底盘文本日志，也不代表当前固件已接入 VT。来源与实现哈希见 [docs/reference/import_manifest.json](../docs/reference/import_manifest.json)。

## 迁移边界

早期 Python 烧录入口曾完成 DAP v2 烧录；现已根据用户要求替换为 RM2026 原生脚本并适配 MC-02。本次替换未运行测试或操作板子；原生脚本的新版本、RTT 外部固件、VT 设备及 Windows PowerShell 仍待实际验证。
