# 参考资料分析

本文保留方案阶段的资料阅读与取舍分析；其中工具链、实施进度和未验收描述属于当时记录。当前使用见 [指南](../guides/build_and_bringup.md)，完成情况见 [验收状态](../validation/README.md)。

分析日期：2026-10-07。本文记录源码和资料阅读结论，不代表硬件验证结果。

## 1. 阅读范围与来源

相对路径均以 `/home/myself/workspace/RM` 为起点。

| 来源 | 重点阅读范围 | 用途 |
| --- | --- | --- |
| `RM2026` | 顶层及分层 CMake；`applications/robot`；正式 FreeRTOS 任务；平台 CAN/UART/SPI/time/critical/watchdog；PID、DJI、BMI088/INS、遥控、视觉、安全及健康服务 | 确定实际复用对象、运行周期和平台耦合 |
| `dm-mc02/达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf` | MCU、供电、CAN、SBUS、串口、IMU、扩展接口及电源控制 | 板卡功能与连接 |
| `dm-mc02/原理图/CtrBoard-H7_V1.0-240124.pdf` | MCU 引脚；IMU、加热、电源控制、SBUS 和通信电路 | 解决引脚和电气关系的歧义 |
| `dm-mc02/原理图/CtrBoard-H7_V1.0-管脚标注图240127.pdf`、`image/H7_管脚标注图.png` | 板上接口位置、引脚归属 | 建立资源表；与手册交叉核对 |
| `dm-mc02/数据手册/stm32h723vg.pdf` | 内核、Flash、各域 SRAM、DMA、FDCAN | 避免把其他 H7 型号的参数带入 H723 |
| ST 官方补充资料 | DS13313 在线 Rev5、AN4839；FreeRTOS 10.3.1 的 M7 port 说明 | 交叉核对缓存与 RTOS port 选择；实施前再核对时钟及器件勘误 |
| 官方 `例程/CtrBoard-H7_*` | DBUS/UART、FDCAN/CAN、IMU/IMU_TempCtrl、MPU、ADC、TIM、WS2812、ALL 的相关配置、驱动及 RTOS 配置 | 单外设初始化参考 |
| `damiao_mc02_bsp/readme.md`、`开发日志.md` | 适配经过、IMU 时序、SPI6/BDMA、缓存和工具链说明 | 理解参考工程的设计动机，识别未经验证的说明 |
| `damiao_mc02_bsp/dm02_test` | CMake、链接脚本、Core、CAN/UART/SPI、时间戳、BMI088、WS2812、任务及 HAL 回调 | 主要实现参考 |
| `dm02_test_pure`、`dm02_mouse_test`、`leg_stand_demo` | 共用基础文件、任务入口、USB 鼠标用途、达妙电机与站姿 demo 的组合方式 | 区分框架基础与特定演示应用 |
| `uart_demo`、`ws2812_demo` | 任务入口、通信/灯效配置及共用驱动 | 补充单功能使用方式 |

本次聚焦工程和平台架构，没有展开轮腿运动学、站姿 PID 和具体灯效等业务算法的正确性审查。

源码基线：

- RM2026：`4cb0020d4a46746eedbe8b7cb3ba629d42c7adf3`。
- 官方资料仓库 `dm-mc02`：`100f51dd4237a4951038c8ce9ac2cf078e4c1c25`。
- `damiao_mc02_bsp/`、`uart_demo/`、`ws2812_demo/` 在参考仓库中属于本地未跟踪目录，不能用官方仓库提交号代表这些目录的版本。实施时应对实际采用文件记录 SHA-256。

## 2. 对参考框架的判断

参考框架提供了比较完整的 MC-02 外设组合：三路 FDCAN、多串口、异步 SPI、BMI088 数据就绪和时间戳、灯和蜂鸣器、USB、外置 Flash。它能帮助快速建立硬件适配。

六个 demo 的 `Core/Src/main.c`、`usart.c`、`spi.c`、UART/SPI 驱动和 Flash 链接脚本分别具有相同的 SHA-256。本质上它们共享一套基础代码，应用入口不同。因此可以选一份基础配置逐项核对，多个 demo 的存在不能单独证明基础驱动在各种条件下都可靠。

它的主要运行方式是定时器中断驱动：10 μs 启动/调度 IMU SPI，125 μs 执行姿态处理，1 ms 回调组合其他业务。该方式服务于这些演示；当前移植延续 RM2026 的任务模型，采用事件采集与任务计算的组合。

## 3. 参考内容的具体取舍

| 内容 | 采用方式 | 原因 |
| --- | --- | --- |
| HSE、PLL、GPIO 复用、SPI2 引脚、UART5 RX、三路 FDCAN 引脚 | 作为初始化基线，经统一资源表核对 | 硬件事实与现有适配直接相关 |
| BMI088 双片选、DRDY、异步 SPI、时间戳 | 借鉴流程，接入原 INS/EKF 的采集接口 | 改善采样时刻与计算时刻的区分 |
| UART 双缓冲、接收至空闲 | 借鉴缓冲交接思路；正式流式链路优先循环 DMA + HT/TC/IDLE 增量收取 | 要处理满缓冲和连续无空闲数据，解析交给任务 |
| 三路 CAN 管理对象和回调 | 用逻辑 bus ID、固定端点表和任务分发重新封装 | 上层不依赖 `hfdcan1`，明确三路资源与错误处理 |
| 达妙电机协议、USB、VOFA、W25Q64 | 按功能阶段参考；单独可选目标 | 保持首版范围，按需引入 |
| 10 μs / 125 μs / 1 ms 中断业务调度 | 不作为新工程的运行主线 | EKF、PID、解析、日志需要可测量的任务预算与状态所有权 |
| CAN 错误回调内直接 Stop/Start | 改为错误记录、输出抑制和任务恢复状态机 | 恢复时间受总线状态影响，避免 ISR 承担复杂恢复 |
| 定时器内无条件喂狗 | 改为健康监督条件喂狗 | 定时器活着不能说明应用和电机任务正常 |
| SPI6 阻塞发送 | 首次灯功能可低优先级有界使用；正式实现可用 SRAM4 + BDMA | 不让灯效占用高优先级中断 |
| 只打开 I-Cache、普通 `.bss` 保存所有缓冲 | 正式实现重新设计 MPU 和 DMA 区域 | D-Cache 开启后需要明确一致性与 DMA 可达性 |
| `-Ofast`、全局递归 GLOB、巨大默认 heap/stack | 使用目标级 flags、显式源文件和静态内存预算 | 数值有效性检查、构建可追踪性和内存可控性 |
| 整体 C++ 类框架 | 保留其有价值的接口思路，保持现有 C 为主的组件体系 | 无需为换板同时改写整套应用语言和运行模型 |

## 4. 必须纠正的细节

1. **SBUS 引脚冲突。** 手册 V1.1 的表格写 PC02；原理图、引脚图和例程 RX 均为 PD02/UART5。PC02 是 IMU SPI2 MISO。按原理图与实物验证采用 PD2。
2. **独立 DBUS 例程的 TX 冲突。** 其 UART5 配为 TX_RX，使用 PB13 TX；PB13 同时是 BMI088 SPI2 SCK。参考综合 demo 已采用 RX 模式。新工程使用 RX-only，并禁止为 UART5 配置 PB13。
3. **SPI6 内存说明有误。** 参考 SPI 驱动把 SRAM4 称为外置 SRAM；它实际是片内 D3 SRAM。SPI6/BDMA 的缓冲按该域放置。
4. **内存型号区别。** H723 的 D2 SRAM 只有 32 KiB；不能照搬 H743 的 288 KiB D2 布局。
5. **CAN 长度需结合 HAL 版本。** 综合 demo 使用的 HAL 版本为 1.11.5，`FDCAN_DLC_BYTES_8` 是数值 8，发送时由 HAL 左移到消息头。不能仅凭 `DataLength = Length` 就判该 demo 错误；新接口仍应在平台内进行显式转换并锁定 HAL 版本。
6. **FreeRTOS port 不能仅按目录名判断。** 原工程与官方 ALL 均为 10.3.1。上游明确说明较新 M7 内核可以使用 ARM_CM4F port，r0p1 必须使用对应 M7 port；实施时读取 CPUID 并按同版本上游说明选择。参见 [FreeRTOS 10.3.1 Cortex-M7 port 说明](https://raw.githubusercontent.com/FreeRTOS/FreeRTOS-Kernel/V10.3.1-kernel-only/portable/GCC/ARM_CM7/ReadMe.txt)。
7. **加热电源不同。** 原理图第 2 页显示电阻接 VCC_IN、PB1 控制 MOSFET；参考 demo 默认关闭加热。不能直接套用 C 板的 PWM compare 和加热 PID 输出。
8. **FDCAN 时钟不同。** 综合 demo 的 PLL1Q 为 120 MHz，独立官方 FDCAN 例程为 80 MHz；它们的 nominal timing 分别与自己的时钟配套。新基线选择后者的 80 MHz / 1 Mbps 配置，保持 CPU 480 MHz，避免跨例程混用分频参数。

本地 H723 数据手册为 DS13313 Rev4；本次查到 [ST 在线版本为 Rev5](https://www.st.com/resource/en/datasheet/stm32h723vg.pdf)。实施前将对应 RM0468、器件 errata 与锁定的数据手册版本记录到来源清单，本方案没有将无法完整在线读取的 RM0468 当作已通读资料。

## 5. 工具链和来源管理

本机已发现 ARM GCC 16.2.0、CMake 4.4.3、Ninja 和 OpenOCD。参考 README 描述的 GCC 13.3.1 并非当前 PATH 工具链。没有发现 PATH 下的 STM32CubeProgrammer CLI。方案分析阶段未编译工程；后续实施的 Debug / Release / host 构建结果见 [实施进度](../archive/milestones/2026-10-07_platform.md)。参考项目未编译。

实施时用同一份 HAL/CMSIS Device/CMSIS Core 配套来源建立基础，优先核对综合 demo 的 HAL 1.11.5 及 `.ioc` 所标包版本；不混合多个官方独立例程的驱动文件。CMSIS-DSP 与所用内核/FPU ABI 对齐，记录版本与构建方式。

参考用户文件可见 USTC-RoboWalker 等版权署名，未定位到覆盖全部用户文件的统一根许可证说明。借鉴设计可以继续；实际复制文件时保留原头部并逐文件记录来源和许可信息，不将其统一改署为本项目原创。

## 6. 待实物确认项

| 项目 | 影响 | 确认阶段 |
| --- | --- | --- |
| 手中 MC-02 板版本与资料版本 | 引脚、电源控制和布局 | P1 上电前 |
| BMI088 的安装轴向、整板在机器人上的姿态 | 重力补偿、yaw/pitch/roll 和云台反馈符号 | P3，整机运动前 |
| 遥控器型号、SBUS/DBUS 类型、硬件反相连接 | 串口格式、解析后端和 failsafe | P2 |
| 实际电机型号、CAN 线、ID、GM6020 协议版本 | 协议、分组和输出量纲 | P4，任何非零输出前 |
| 机械零位、减速比、软限位及 PID 是否沿用旧机器人 | 旧参数是否适用 | P5 |
| 视觉、裁判系统、调试串口最终接线 | 默认资源分配是否需变更 | P2/P5 |
| 板卡可控输出实际供电对象 | 软件停止能否辅以电源切断 | P5，默认不假定它能切断所有电机供电 |

以上不会阻塞工程骨架和基础外设阶段；涉及机械动作的阶段以实际接线配置为依据。
