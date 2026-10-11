# 延后通信入站里程碑

更新日期：2026-07-23

本里程碑将正式步兵固件的 CAN/UART 协议解析与设备状态发布移出中断。
该历史切片建立时不改 PID、执行器限幅、输入选择和控制模式转换；后续遥控重构已经删除
运行时输入仲裁，改为 ET08、DT7/DR16、VT 编译期三选一，并统一发布遥控状态。

## 1. 数据流

```text
CAN/UART 中断
    -> 复制有界接收数据
    -> endpoint inbox
    -> 5 ms 电机任务 dispatch
    -> 协议校验和解析
    -> 发布一致设备快照
    -> 控制/诊断消费者
```

电机任务在 `DJIMotorControl()` 前 dispatch，使电机反馈解析和 PID 消费位于同一任务。
入站开销也因此进入已有 5 ms 执行时间/deadline 监测，无需增加未监测的通信任务。

## 2. Inbox 约定

`platform/common/transport/rm_rx_queue` 是使用调用方存储的平台无关队列，从不动态分配。
单核 ISR 生产者和任务消费者只在复制一个保留项时进入短 STM32 临界区。

| 传输 | 保留策略 | 溢出策略 | 正式 dispatch 上限 |
| --- | --- | --- | ---: |
| CAN | 每 endpoint 1 个槽 | 新反馈替换未消费旧反馈 | 16 callbacks / 5 ms |
| UART | 每 endpoint 4 个事件 | 丢弃最旧、保留最新 | 12 callbacks / 5 ms |

CAN 反馈表示当前设备状态，回放每个 1 kHz 电机帧只会增加延迟。合并后每个 dispatch 每 endpoint
最多处理一个最新帧。UART receive-to-idle DMA 可能提供部分流数据，因此保留事件边界。

延后 UART 模式中，DMA 存储与回调可见 `recv_buff` 分离。ISR 可立即重启 DMA，不会覆盖任务正在解析的数据。
ET08 仍要求精确 25 字节 SBUS 帧，VT 仍执行流重组和 CRC 校验。

`CANGetDispatchStats()` 和 `USARTGetDispatchStats()` 提供合并、拒绝和覆写计数，不在中断中格式化日志。

## 3. 兼容与启动

历史板级 demo 默认仍在中断中 dispatch。需要任务上下文解析的固件必须在首个 endpoint 注册前调用
`CANConfigureDispatch()` 和 `USARTConfigureDispatch()`，注册后不允许改模式。

正式 FreeRTOS 固件和 `test_infantry_minimal` 都在 `RobotApp_Init()` 前选择延后模式。
FreeRTOS 的 5 ms 电机任务、裸机对照循环都在消费电机反馈前调用
`CANDispatchPending(0)` / `USARTDispatchPending(0)`。剩余启动期间，CAN 保留最新反馈，
UART 保留最新四个事件，避免调度器启动前累积无界旧数据。

## 4. 一致设备读取

延后解析不等于跨任务数据自动安全。正式应用不再直接读取可变 ET08、VT 或 DJI 实例：

- `ET08_Read()` 在保护下复制控制快照，并在复制前后确认链路健康。
- `VT_Read()` 对 CRC 校验后的 VT 状态使用同样约定。
- `DJIMotorGetMeasure()` 一次复制编码器、速度、电流和温度。
- DJI 解码先在局部构建新状态，再用一个短临界区发布。
- 首个有效 DJI 反馈帧建立编码器基线，不会相对全零结构误判回绕。

历史 live-view getter 只为单循环对照 demo 保留，新并发固件必须使用快照 API。

## 5. 验证与历史数据

主机接收队列测试覆盖 FIFO 顺序、索引回绕、保留最新溢出、单槽合并、无效输入与目标容量不足。

2026-07-16 该里程碑建立时，Debug 正式镜像使用 RAM 50,696 字节、Flash 95,028 字节；
增量 RAM 是固定 UART DMA/事件 inbox，没有恢复堆预留。当前数据以
[步兵 FreeRTOS 迁移基线](infantry_freertos_baseline.md)为准。

编译和主机测试不能代替中断时序、总线负载、UART burst/部分帧注入、断链恢复和长时间 soak 验收。

本里程碑当时留下的电机命令/PID/INS 所有权债务已由
[控制所有权与设备故障包络里程碑](control_ownership_milestone.md)关闭。
