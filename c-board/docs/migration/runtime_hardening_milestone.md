# 运行时加固与组件边界里程碑

更新日期：2026-07-23

本里程碑关闭控制所有权迁移后的主要软件加固项：BMI088 有界启动、与调度器独立的复位包络、
仓库自有 newlib 边界、非 RWX ELF 段，以及不再链入所有兼容驱动的正式组件集。

## 1. BMI088/INS 有界初始化

正式云台使用 `INS_InitWithTimeout()` 和统一 15,000 ms 总预算，覆盖 BMI088 寄存器设置与在线标定。
历史 `INS_Init()`/`BMI088Init()` 保持源兼容，但也使用有限默认预算，不再无限自旋。

BMI088 路径具有以下边界：

- 每个 SPI 字节 HAL timeout 为 2 ms；
- 每个配置寄存器最多写入/校验三次；
- 向上返回 chip-ID、寄存器、参数、总超时和标定错误；
- 标定在采样循环内检查总 deadline，超时时丢弃不完整平均；
- 标定失败会为诊断连续性加载机器人离线值，但仍向正式应用返回失败；
- 修复 gyro 寄存器重试索引下溢和错误表索引问题。

`INS_InitWithTimeout()` 也检查加热 PWM 启动。任一传感器失败会将加热输出置零、停 PWM、记录初始化状态与
BMI088 错误并返回 `NULL`。失败一直上传到 `RobotApp_Init()`，全局执行器门保持关闭且不创建运行任务。

初始四元数构造已处理零长和平行向量，并限幅 `acosf` 输入，防止启动边界样本产生非法浮点状态。

## 2. 独立硬件看门狗

正式固件只在应用初始化成功且六个控制/运行任务创建成功后启动 STM32F407 IWDG。
然后在调度器启动前创建有界的 USB 一次性任务。这避免在两秒电机稳定和 IMU 标定阶段误复位。

看门狗使用预分频 256，按保守 48 kHz LSI 上限计算 reload。配置最小 1000 ms 时 reload=187。
STM32F407 数据手册给出 LSI 17–47 kHz，因此预期复位间隔约 1.02–2.83 s。保守算法保证快 LSI 不会早于配置下限复位。
数据来源：[STM32F407 官方数据手册](https://www.st.com/resource/en/datasheet/stm32f407ie.pdf)。

只有 5 ms 健康任务喂 IWDG，且只在所有关键任务通过心跳、连续 deadline miss 和栈余量检查时喂狗。
可包络单个关键任务停止、整个调度器停止、持续关键超期或栈不健康。

软件调度仍可运行时，安全管理器先立即关闭执行器；调度不再可靠时，IWDG 是独立最后保障。
Debug 中 Cortex-M 核停止时冻结 IWDG，避免断点被误认为运行时故障。

## 3. HardFault 保留

2026-07-19 结构收口中移除了 HardFault 里的错误 `bx lr`。新处理路径：

1. 通过 EXC_RETURN 判断 MSP/PSP 异常栈帧；
2. 关闭可屏蔽中断；
3. 在校验栈地址后保留 R0–R3、R12、LR、PC、xPSR；
4. 保留 CFSR、HFSR、DFSR、AFSR、MMFAR、BFAR、SHCSR 和 ICSR；
5. 最后写入 magic，并在 `.noinit` 中保留到复位后；
6. IWDG 已运行时等待其复位，否则保持现场供调试器检查。

调试器可直接检查 `g_rm_fault_record`；只有 `magic == RM_FAULT_RECORD_MAGIC` 时记录才完整。

## 4. newlib 与 ELF 约定

每个嵌入式可执行目标直接包含 `platform/stm32f4/runtime` 中的仓库自有 bare-metal syscall 边界：

- stdout/stderr 是非阻塞 RTT 诊断流；
- 不支持的文件/进程操作以 `errno` 明确失败；
- `_exit` 关中断后进入终止等待；
- 故意不提供 `_sbrk`，正式符号审计继续拒绝堆分配。

链接脚本将 RAM/CCMRAM 标记为不可执行，构造/析构数组为只读，BSS、`.noinit` 和预留堆/栈为 `NOLOAD`。
`readelf -lW` 审计不允许任何 RWX LOAD 段。链接脚本也是所有嵌入式目标的显式 link dependency，修改后会可靠重链。

正式应用现已包含 C++ 遥控适配层。固件显式使用 C++ 链接驱动，C API 头统一提供
`extern "C"` 边界；编译关闭异常、RTTI、线程安全静态初始化、析构注册和展开表，链接后再审计
`new/delete`、`__cxa_*`、personality 与 unwind 符号。

## 5. 正式组件与兼容组件

- `RM::components_runtime` 只含正式步兵固件已批准的算法、BMI088/INS、DJI 电机、输入、裁判、视觉和设备健康。
- `rm_components_compat` 保留历史 demo 使用的驱动和注册 API。
- `RM::components` 只是现有嵌入式回归固件的兼容伞。
- `app.elf` 和步兵裸机对照 target 直接链正式集，不会意外拉入兼容归档。

这是过渡边界；后续仍需将 runtime 拆为更窄算法、设备和服务 target。

## 6. 验证与历史数据

本里程碑门禁包含正式 `app.elf` 及符号审计、`test_infantry_minimal`、30 个嵌入式回归 target、
9 个严格警告主机测试和 ELF 程序头权限审计。

2026-07-16 该里程碑建立时，Debug 正式镜像使用 RAM 51,216 字节、Flash 97,452 字节。
2026-07-19 目录收口与 HardFault 保留后的当前数据见[步兵 FreeRTOS 迁移基线](infantry_freertos_baseline.md)。

## 7. 仍需实板验收

- 冷启动标定稳定完成并发布首个 INS 样本。
- BMI088 断开或移动时 15 s 内退出，所有执行器保持禁用并暴露正确错误码。
- 分别暂停关键任务会停止输出并触发 IWDG 复位。
- 整个调度器停止会在实测板级时间内复位。
- 最大正常负载不误复位，调试器 halt/resume 正确冻结/恢复 IWDG。
- 确认 HardFault 记录可跨看门狗复位读取，PC/LR/CFSR 可用于定位。
- yaw/pitch 方向、命令租约、CAN 负载和平滑性门禁仍全部满足。

剩余软件债包括 INS 运行时 data-ready/合理性健康、可变 `g_robot`、
`DJIMotorInstance` 公开兼容字段、更细组件 target 与非活动兼容路径清理。
