# 平台运行时里程碑

更新日期：2026-10-02

本里程碑建立正式步兵固件使用的仓库自有运行时路径。它只调整存储、计时、恢复和诊断基础设施，
不改 PID、执行器限幅和应用模式转换。

## 1. 单调时间

`platform/stm32f4/time/rm_time.h` 是应用可见的平台时钟：

- `RmTime_NowMs()` 返回无符号 HAL 毫秒 tick。
- `RmTime_NowUs()` 返回基于 DWT 扩展的 64 位微秒时间线。
- `RmTime_ElapsedMs()` 使用无符号减法，可跨越 32 位毫秒回绕。
- `RmTime_DeadlineReached()` 使用序列号算法，配置截止不得超过未来 `2^31` ms。

DWT 32 到 64 位扩展在任务/中断间串行化，并按 `2^32` 周期处理回绕。168 MHz 下必须至少
每约 25.6 s 采样一次；正式 INS、电机、PID 和 CAN 活动远高于此频率。

## 2. 设备健康

`components/services/device_health` 管理最多 64 个固定实例。注册时使用毫秒超时和启动宽限，
有效喂入写入新的绝对截止。因此健康任务周期不再改变设备超时。

新实例在首次有效喂入前不视为在线；启动宽限只延后首次离线回调。每段离线仅回调一次，
下一次有效喂入后重新布防。一旦判定离线，状态保持到新喂入，避免长时间运行的回绕歧义让旧设备“复活”。

| 消费者 | 截止 |
| --- | ---: |
| DJI/HT 电机反馈 | 20 ms |
| LK 电机反馈 | 50 ms |
| DT7/DR16 D-BUS | 100 ms |
| 正式视觉 UART (`VISION_ONLINE_TIMEOUT_MS`) | 250 ms |
| VT 输入 | 200 ms |
| 裁判链路 | 300 ms |
| ET08 兼容默认 | 4000 ms |
| 正式步兵 ET08 | 100 ms |

全零配置选择服务默认 1000 ms，大于等于 `2^31` ms 的值会被拒绝。

## 3. CAN/UART 注册

CAN 和 UART endpoint 不再从 C 堆分配。

| 资源 | 固定容量 | 注册规则 |
| --- | ---: | --- |
| CAN endpoint | 总计 16 | 每个 CAN handle 下标准 RX ID 唯一 |
| CAN filter | 每总线 14 | 每 endpoint 一个 16 位 ID-list bank |
| UART endpoint | 总计 3 | 每 UART handle 一个 endpoint |
| UART 接收缓冲 | 每 endpoint 256 字节 | 请求长度必须容纳 |

CAN 发送超时使用整数微秒。接收回调排空选定 FIFO，经典 CAN 帧最多复制 8 字节。
UART 使用 receive-to-idle DMA；HAL busy 恢复会中止旧接收后仅重启一次。

后续的[延后通信入站里程碑](deferred_ingress_milestone.md)已将正式固件中的协议回调移出中断。

UART 注册会立即启动 receive-to-idle DMA。若后续设备注册失败，启动代码必须对刚注册的
最后一个端点调用 `USARTUnregister()` 执行回滚；该接口只允许后进先出注销，以保持既有
端点地址稳定。它先尝试 `HAL_UART_AbortReceive()`，DMA 无法安全停止时返回失败并保留
端点，不把未停止的 DMA 当作已释放资源。该接口只用于单线程启动失败回滚，不是运行时
动态卸载机制。

## 4. 正式路径无堆化

正式固件的以下资源使用静态存储：

- 12 个 DJI 电机实例；
- CAN、UART 和设备健康 registry；
- 四元数 EKF 的 Kalman workspace（约 1.6 KiB）；
- 所有 FreeRTOS 任务和栈；
- 有界诊断格式化。

正式链接器不预留 C 堆，为启动和中断上下文预留 8 KiB 主栈。每次正式链接会拒绝
堆分配和 libc 格式化符号；引入 C++ 后还拒绝 `new/delete`、异常展开、RTTI、
`__cxa_guard_*` 和 `__cxa_atexit` 等不允许的运行时依赖。

`RmFormat_Snprintf()`/`RmFormat_Vsnprintf()` 提供固件日志所需的整数、字符串、指针和定点小数格式，
它是有界固件 formatter，不保证完整 libc `printf` 兼容。

未进入 `app.elf` 的兼容模块仍含堆注册 API，包括部分 GPIO/IIC/PWM/SPI、消息总线、
通用数学、CAN 通信和非活动设备驱动。它们在迁移或删除前不得成为比赛固件依赖。

## 5. 验证与历史数据

主机测试覆盖 deadline 边界、轮询频率无关性、时钟回绕、长期离线锁定、一次性回调、
非法截止、registry 容量、格式宽度/标志、浮点舍入和缓冲截断。

2026-07-16 该里程碑建立时，Debug 正式镜像使用 RAM 45,872 字节、Flash 92,476 字节；
这是历史对比数据，当前数据以[步兵 FreeRTOS 迁移基线](infantry_freertos_baseline.md)为准。

后续[运行时加固里程碑](runtime_hardening_milestone.md)已补齐 newlib 边界、消除 RWX LOAD 段并加入程序头审计。

本里程碑是软件构建/主机测试门禁，不能替代实板时序、断链恢复、CAN 饱和和长时间 soak 验证。
