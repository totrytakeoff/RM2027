# 构建与板级诊断

当前提供 `board_bringup` 基础固件和 [chassis_demo 底盘测试 app](chassis_demo.md)。2026-10-11 已执行基础固件烧录和实板验证，底盘基本联调已由用户确认，当前范围见 [验收状态](../validation/README.md)。

## 1. 构建

日常操作和原生烧录脚本见 [scripts/README.md](../../scripts/README.md)。本文通信表主要描述 `board_bringup`；底盘 app 的四电机绑定及 USART1 日志见 [底盘指南](chassis_demo.md)。

在工程根目录执行，需要 CMake ≥ 3.21、Ninja、带 newlib-nano 的 ARM GCC。

```sh
cmake --preset h7-debug
cmake --build --preset h7-debug

cmake --preset h7-release
cmake --build --preset h7-release

cmake --preset h7-dbus-debug
cmake --build --preset h7-dbus-debug

cmake --preset host
cmake --build --preset host
```

| preset | 输出 | 内容 |
| --- | --- | --- |
| h7-debug | `build/h7-debug/{board_bringup,chassis_demo}.{elf,bin,hex,map}` | M7 hard-float、`-Og -g3`、RM_DEBUG；暂停调试时冻结 IWDG |
| h7-release | `build/h7-release/{board_bringup,chassis_demo}.{elf,bin,hex,map}` | M7 hard-float、`-O2 -g`；保留离线符号，IWDG 正常运行 |
| h7-dbus-debug | `build/h7-dbus-debug/board_bringup.{elf,bin,hex,map}` | DBUS UART5 8E1；其余同 Debug；接线/反相仍需确认 |
| h7-chassis-observe | `build/h7-chassis-observe/chassis_demo.{elf,bin,hex,map}` | 完整输入/IMU/控制/诊断，电机仅发送零指令 |
| host | `build/host/librm_logic.a`、`librm_matrix.a`、`librm_chassis_logic.a` | 平台无关组件；`RM_BUILD_TESTS=ON` 时可执行底盘控制测试 |

`host` 是编译可移植组件的入口，不包含模拟 MCU/RTOS 或可运行机器人应用。CMake 的 `rm_logic` target 已声明对 `rm_matrix` 的依赖；手动使用 archive 时按 `librm_logic.a librm_matrix.a -lm` 顺序链接。

归档构建快照的产物大小和 SHA-256 见 [历史构建快照](../archive/records/build_manifest_2026-10-11.json)，仅对应其中记录的构建时点，当前产物以重新构建后的文件为准。

正式默认启用 I-Cache/D-Cache/MPU。排查启动时可以构建独立关闭 D-Cache 的目录：

```sh
cmake -S . -B build/h7-no-dcache -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Debug -DRM_ENABLE_DCACHE=OFF
cmake --build build/h7-no-dcache
```

该选项保留 I-Cache 和 MPU；关闭 D-Cache 的构建不能作为缓存一致性验收记录。

默认 FreeRTOS GCC port 为与内核同版本的 `ARM_CM7/r0p1`，使用 M7 屏障方案；目录名不表示当前芯片已判定为 r0p1。启动日志记录实际 CPUID，具体修订版仍以实板读取为准。`ARM_CM4F` 源码作为原工程参考保留，可通过 CMake 变量选择，当前默认构建未选择它。

## 2. 下载与调试入口

已提供 CMSIS-DAP 和 ST-LINK 配置，均使用 SWD，H723 单核/单 Flash bank、32 KiB DTCM 下载 work area。CMSIS-DAP 已连接 Horco 无线 DAP 验证，ST-LINK 尚未实测。按实际探针选择其一：

```sh
openocd -f platform/boards/dm_mc02/openocd/cmsis-dap.cfg \
  -c "cmsis_dap_backend usb_bulk" \
  -c "program build/h7-debug/board_bringup.elf verify reset exit"
```

```sh
openocd -f platform/boards/dm_mc02/openocd/stlink.cfg \
  -c "program build/h7-debug/board_bringup.elf verify reset exit"
```

命令将写入内部 Flash；CMake 构建不会自动调用下载。工程固件从 `0x08000000` 开始，不包含 bootloader 偏移。

需要交互调试时，启动 OpenOCD 后用 `arm-none-eabi-gdb build/h7-debug/board_bringup.elf` 连接 `localhost:3333`。CPU halt 时 TIM5/6 当前继续计数，DWT/RTOS 停止；恢复后任务时序可能记为超期，不能将调试暂停期间的数据作为实时预算。

## 3. 基础诊断固件通信配置

| 逻辑接口编号 | 外设及引脚 | 格式 | 当前功能 |
| --- | --- | --- | --- |
| serial 0 remote | UART5 RX PD2 | 默认 100000，8E2；DBUS 构建为 8E1，HAL 9-bit + EVEN | 基础固件接收解析；底盘 app 接 ET08 操作模式 |
| serial 1 vision | USART1 TX PA9 / RX PA10 | 921600，8N1；底盘 app 为115200 | RX 原视觉协议 parser；暂无正式回包 |
| serial 2 referee | UART7 TX PE8 / RX PE7 | 115200，8N1 | 字节计数；未接裁判协议 |
| serial 3 console | USART10 TX PE3 / RX PE2 | 115200，8N1 | 文本诊断 TX；RX 只计数，无命令控制 |
| CAN 0/1/2 | FDCAN1/2/3 | 经典 CAN、1 Mbps，75% sample point | 接收标准/扩展数据帧，拒绝远程帧；基础固件不提交运动 TX；底盘 app 使用 CAN1 |

UART5 的格式是当前 bringup 配置，不能直接宣称 DT7/DBUS 后端可用；板载反相与实际接收器仍需确认。控制台使用 3.3 V UART 电平并共地。

FDCAN kernel 为 PLL1Q 80 MHz，nominal prescaler 1、seg1 59、seg2 20、SJW 20。三个 Message RAM 起始偏移为 0/512/1024 **words**，每路实际 210 words；经典 TX dedicated buffer 三个，TX FIFO 为零。当前 NORMAL 模式会参与 ACK。

SPI2 使用 mode 3，PLL3P 80 MHz / 8 = 10 MHz，Accel CS PC0 / Gyro CS PC3。初始化逐步等待和回读寄存器，轮询采样时间戳是读取时间，尚未代表 DRDY 采样时刻。

## 4. 日志格式

控制台行格式：`RM <time_ms> event=<event> a=<uint32> b=<uint32>`，全部十进制。无浮点 printf。每秒发布一次统计，发送繁忙则保留队列记录；队列满时增加 drop。

| event | 含义 | a | b |
| --- | --- | --- | --- |
| 1 | BOOT | CPUID | HAL version 编码；1.11.5 为 `0x010B0500` |
| 2 | CLOCKS | CPU Hz，预期 480000000 | CAN kernel Hz，预期 80000000 |
| 3 | IMU_READY | `(accel_id << 8) \| gyro_id`；预期 `0x1E0F` / 7695 | 初始化 phase |
| 4 | IMU_FAILED | 同上，可能只有部分 ID | 失败 phase；结合 `imu.errors/register_index` 查看 |
| 5 | TASK_FAULT | reason bit mask | 当前为零 |
| 6 | HEARTBEAT | 当前 ms | 日志 dropped 数 |
| 7 | SERIAL_STATS | 高 4 位 serial ID，低 28 位 RX bytes | 软件队列丢失 bytes |
| 8 | CAN_STATS | 高 4 位 CAN ID，低 28 位 RX frames | FIFO/软件队列丢失计数 |
| 9 | IMU_SAMPLE | 原始样本 sequence | valid：0 / 1；与融合状态分开 |
| 10 | RESET_FLAGS | 启动时 RCC RSR | D-Cache：0 / 1 |
| 11 | PREVIOUS_FAULT | 上次 panic/exception reason | 上次堆栈 PC；不可读时为零 |
| 12 | TASK_TIMING | 高 4 位 task ID，低 28 位 deadline misses | 最大回调耗时 μs，含抢占时间 |
| 13 | TASK_STACK | 高 4 位 task ID，低 28 位 run count | 最小剩余栈 words；每 100 次执行更新 |
| 14 | INS_STATE | CALIBRATING=0 / RUNNING=1 / FAILED=2 | NONE=0 / CONFIG=1 / TIMEOUT=2 / SENSOR=3 / TIME=4 / NUMERIC=5 |
| 15 | INS_SAMPLE | 融合 sequence | valid：0 / 1；静止校准中为零 |
| 16 | REMOTE_SAMPLE | 有效输入 sequence | bit0=data_valid、bit1=failsafe；离线时 sequence 为零 |
| 17 | DJI_CANDIDATES | 高 4 位 bus、低 11 位 seen mask | 低 11 位 online mask；bit0..10 对应 0x201..0x20b |
| 18 | BINDING | SBUS=0 / DBUS=1 | 正式绑定电机数量，默认 0 |
| 19 | MOTOR_IO | 正式绑定电机数量 | 电机组帧提交失败累计数 |
| 20 | SERIAL_RECOVERY | 高 4 位 serial ID，bit0=online、bit1=quarantined | 高 16 位 abort timeout、低 16 位 recovery exhausted 计数，各取低 16 位 |

打包字段解码：`id = a >> 28`，`count = a & 0x0fffffff`。task ID 按 ins/io/motor/health/control/diagnostics 为 0～5。

调试器可观察 `imu.sample`（accel m/s²、gyro rad/s、temperature °C）、`ins_snapshot`、`remote.state`、`remote.stats`、`dji_candidates`、`stats`、`endpoints`、`buses`。跨任务快照入口见 [历史组件迁移](../archive/milestones/2026-10-07_components.md)；GDB 静态变量同名时用源文件限定名。

常用 panic 原因：`0x2001..0x2005` 供电/时钟/HAL 初始化；`0x2006` 运行时钟不符；`0x2007` 非预期 M7 CPUID 类型；`0x2008` 早期 IWDG 更新超限；`0x2009` 早期供电就绪等待超限；`0x20ff` generated 初始化错误；`0x3001..0x300c` runtime 配置/任务/看门狗/调度异常；`0x4001` 通信启动失败；`0x4002` 组件 binding 校验失败；`0xf000 \| IPSR` HardFault。详细区分以源码记录点为准。

reset assembly 在供电初始化之前启动名义 12 s 的 IWDG；runtime 切换到名义 500 ms 的进度监督。Debug 冻结看门狗、Release 取消冻结。UART abort 超过 50 ms 或一次故障的恢复尝试耗尽 3 次后隔离该端口，直到 MCU reset，不自动复用 TX 缓冲。接口和限制见 [审计修复](../archive/milestones/2026-10-08_audit_fixes.md)。

## 5. 板上记录入口

首次记录应保留构建 preset、固件 SHA-256、板版本、供电、接线和日志，并逐项对照 [历史组件进度](../archive/milestones/2026-10-07_components.md) 中尚未通过的出口。尤其需要观察：

- 启动及复位原因、时钟、心跳、IMU chip ID、静态重力、姿态转动方向和样本递增。
- 连续 UART、HT/TC/IDLE 边界、错误恢复；三路 CAN 的独立计数、队列丢失与 bus-off。
- D-Cache 开启时 DMA 地址与收发正确性；关键任务 run count、超期、最大耗时和栈余量。

`board_bringup` 电机 binding 数量为零，不发送运动指令。`chassis_demo` 绑定 CAN1 四电机并允许运动；当前验收结论见 [验收状态](../validation/README.md)。
