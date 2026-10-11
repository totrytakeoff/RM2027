# 2026-10-11 MC-02 实板记录

> 历史归档：本文件记录当时阶段，不代表当前进度。当前结论见 [验收状态](../../validation/README.md)。

原始记录目录：[hardware_runs/2026-10-11](../hardware/2026-10-11)。所有测试在用户授权烧录、测试和调试后执行；初期仅接 24 V 和无线 DAP，随后接入 ET08，最后用户说明 USART1 已接 DAP 串口。

## 探针与备份

Horco CMSIS-DAPv2，USB `faed:4870`，序列号 `305410062834`，探针固件 Horco v0.2。MCU CPUID `0x411fc272`（Cortex-M7 r1p2），DBGMCU_IDCODE `0x10016483`，Flash size 寄存器为 1024 KiB。初始 SWD 1800 kHz。

首次烧录前备份完整 1 MiB Flash：[original_flash.bin](../hardware/2026-10-11/backup/original_flash.bin)，SHA-256：`83ef6ddd53af11fa72dc77d4b67e55bf8b21ea2fb3ba7ff229ffc0521dbed1c3`。备份命令/结果在 `backup_record.json`；烧录通过 OpenOCD verify。

## 基础固件实测

`final` 目录是修复 RX 上拉和启动失败恢复后的重新测试结果。该批 Debug ELF SHA-256：`353f31c635ee031e99e11e82a0cd960665fb51ecc33a6f496d44b69c485db541`。后续加入底盘接口后的当前构建哈希以 build manifest 为准。

| 项目 | 结果与证据 |
| --- | --- |
| 空载 Debug 连续 60 s | 无复位，六任务 deadline misses 均为零；`debug_baseline.json` |
| BMI088 / INS | 芯片 ID 0x1e/0x0f，128 静态校准样本，INS RUNNING/valid/converged，dt 约 1 ms；温度约 31–37 °C |
| 时钟 / 缓存 | CPU 480 MHz、HCLK 240 MHz、FDCAN 80 MHz；CCR 0x70200、MPU CTRL 5 |
| 带 ET08 的 Debug 60 s | `bringup_verified/debug_baseline.json`，任务/INS/串口/CAN/门状态有逐秒记录 |
| 初始 RX 启动返回失败 | 用 GDB 将首次 HAL RX 启动返回值设为 HAL_ERROR；框架继续启动，I/O 完成一次重启，INS 正常；`bringup_verified/serial_start.json` |
| UART 正常恢复 | SWD 设置故障标志，真实 HAL/DMA abort 回调完成并重启；`bringup_verified/serial_recover.json` |
| UART 缺少 abort 回调 | 屏蔽 UART5 RX 的 DMA1 Stream4 IRQ 15，50 ms 后隔离；UART UE/DMA EN 清零、IRQ 关闭，其他任务继续；`bringup_verified/serial_timeout.json` |
| CONTROL 任务停滞 | 将其 callback 指向长延时，HEALTH 继续运行，bit11 失联保护出现，约 0.6 s 内观察到 IWDG 复位，任务恢复；`bringup_verified/task_stall.json` |
| C 初始化前停滞 | 在 EarlyStart 后、ExitRun0Mode 前转入死循环；约 12 s 后 IWDG 复位并正常启动；`bringup_verified/early_stall.json` |
| 真实 FPU HardFault | RAM 中执行 vmov/vadd 后 UDF；EXC_RETURN=0xffffffed，扩展/惰性 FP 帧，记录 PC=0x2404f008 与实际 UDF 一致；`bringup_verified/fpu_fault.json` |
| Fault 输出抑制 | FPU fault 时 PB1/PC13/PC14 为低，三个 FDCAN CCCR.INIT=1；随后 IWDG 复位与 INS 恢复 |
| ET08 静态接收 60 s | 4166 帧、4150 good；UART 丢字节/新增错误/重启均为零，parser bad_frames=0；接收器标记 frame_lost=16；`bringup_verified/et08_movement.json` |

最后一份遥控文件名称含 movement，但本次窗口未观察到摇杆或开关变化，因此它只证明静态通信。四轴为 `[17,0,0,0]`，旋钮为 `[-418,365]`，四开关均 DOWN；操作行程、开关全档位和发射机断电/恢复尚未完成实测。

## 本轮发现和修复

1. 未接设备时 RX 悬空，出现一次性 UART 接收错误；为四个 MCU 侧 RX 配置 idle-high 上拉，TX 保持原配置。
2. 早期看门狗确实复位，但某次重启时 UART 初始 RX 失败引发 0x4001，使框架无法进入调度。`RmSerial_Start` 现在返回端点/存储注册成功，单端口 RX 失败进入现有有截止时间、限次恢复/隔离流程。
3. Release ELF 保留 DWARF，便于不暂停 CPU 的 SWD 诊断；不启用 Debug watchdog 冻结。

`tools/board_probe.py` 是实时只读观测，`--reset` 才发复位命令。OpenOCD H7 配置使用 cacheable AP access。分次 SWD 读取可能跨任务更新或复位，不代表原子软件 API 调用。任务时间来自固件 DWT 统计；主机读取间隔不作为微秒级精度证据。

故障脚本会改写 live RAM/NVIC/PC，运行结束 reset run；FPU 测试临时关闭 MPU/D-Cache以执行 AXI 尾部的测试指令，正式启动恢复原配置。带故障的运行不能作为正常实时预算。实验记录已移至 `bringup_experiments`；验收应使用 `bringup_verified` 中带 firmware hash 的记录。

FDCAN INIT 证明控制器进入停止请求状态，本轮没有受阻仲裁/自动重传或带负载的总线波形，因此不宣称故障时已上总线帧能撤回。UART 缺回调测试使用 RX DMA，不证明活动 TX 缓冲的所有权边界。

## 底盘 demo 板上状态

已烧录底盘运行版 `build/h7-debug/chassis_demo.elf`，OpenOCD 返回 Programming Finished / Verified OK / Resetting Target，ELF SHA-256 为 `f6bdfd1d59f2ff7c1783d6ca763f5b6d7cf8dce9f0f1fdb07c7b74cfe7b0a425`。先前未上电/探针无响应的失败记录保留，成功证据见 `chassis/chassis_flash/flash.json`。

烧录后读取 20 s：无复位，六任务 deadline_misses 均为 0，CONTROL 栈最小余量 404 words；INS 校准 128 样本、valid/converged=true，ET08 在线且有效，SA/SB 均在下档。USART1 115200 捕获 46302 bytes、100 组 CH 行，decode_errors=0，运行版显示 tx=1、en=0。

四路电机均无反馈，CAN1 记录 bus-off 与恢复尝试，输出门关闭；这些观测不能确定具体接线/供电原因。当前已验证应用启动、静态遥控、IMU 和串口诊断，真实四轮闭环、运动模式和 yaw 极性仍待联调。证据见 `chassis/chassis_flash/baseline.json`、`usart1.txt` 与 `usart1.json`。

实际 CAN 口、轮序/电调 ID、机械尺寸与 IMU 安装位置仍需对应 app 配置。DT7/DBUS、视觉/裁判外部输入、CAN 压力/掉线恢复、DRDY/DMA 采样和整机 RobotApp 仍按移植计划推进。

## 实车轮序与 yaw 初调（后续修订）

用户反馈平移左右反向、旋转方向正确、yaw 超调和震荡，提供布局 `4 1 / 3 2`。按上排为车头暂配置 FR1/FL4/BR2/BL3；车头方向及前后平移尚待确认。yaw 改为 Kp4/Ki0/Kd0.8，目标90 deg/s、上限2.5 rad/s，实测角速度阻尼加独立目标角速度前馈。各构建与已有主机测试通过。

本修订连接两次均在 CMD_INFO 阶段失败，8 s UART 捕获为零；尚未烧录，也未验证负载下的超调和震荡改善。当前已成功烧录的仍是上一节记录的初版 hash。本修订证据见 `chassis/chassis_tuning/revision.json`。新旧实测记录按各自 ELF hash 区分。

### 修订版烧录成功

默认 DAP v2 USB bulk 仍返回 CMD_INFO failed；切换 `cmsis_dap_backend hid` 后连接、烧录、Verified OK、复位均成功。当前修订 ELF SHA-256：`755ad80898512f9473c3028c70fee7f24f2662118c9b89b5da978552abdce6a4`。证据为 `chassis/chassis_tuning/flash.json`。

10 s 启动观测：复位 0 次，任务 deadline_misses [0, 0, 0, 0, 0, 0]，INS valid/converged=True/True；USART1 23863 bytes、50 组 CH，电机在线 mask=15。见同目录 baseline.json、usart1.txt/json。本次未操作摇杆，轮序和 yaw 超调改善仍需实车动作验证。

### DAP v2 再次烧录成功

按用户要求显式指定 `cmsis_dap_backend usb_bulk`，OpenOCD 确认使用 CMSIS-DAPv2，Programming Finished / Verified OK / Resetting Target 均成功，退出码0。烧录同一轮序/yaw 修订版，源码及固件未改；证据见 `chassis/chassis_tuning/flash_dapv2.json`。早前 CMD_INFO 失败记录仅表示当次连接状态，不能断言 DAP v2 不可用。

## 前后方向修订版

按用户反馈保留 FR1/FL4/BR2/BL3、左右映射与旋转映射，前后输入由 -left_y 改为 +left_y，配置为 REMOTE_X_SIGN=-1 / REMOTE_Y_SIGN=+1，极性只允许±1。保留当前源码 yaw Kp10，已编入本次固件。通过 scripts/upload.sh 显式 DAP v2 完成烧录、Verified OK 和复位，退出码0；ELF SHA-256：`93eec62f11ec05f2e098a59c7741cfd8758f25373d75ab369f90ccba7a0bf368`。记录见 `chassis/chassis_forward_fix/flash.json`。各构建通过，现有前后映射测试断言同步更新，但本次未运行测试或实车动作验证。


归档后的各原始文件目录见 [归档索引](../README.md)。上文未带完整链接的文件名属于历史记录简称。当前参数和用户基本可用反馈见验收状态。
