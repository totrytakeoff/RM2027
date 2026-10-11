# 2026-10-08 审计修复

> 历史归档：本文件记录当时阶段，不代表当前进度。当前结论见 [验收状态](../../validation/README.md)。

本轮处理当前 bringup 源码审计发现的七项边界问题。运动与加热保持关闭，正式电机绑定仍为零。完整 RobotApp、电机命令发布/位置重建接口、DRDY/DMA 和生效 CubeMX 配置仍按移植阶段推进。

## 1. 修复内容

| 审计项 | 当前源码处理 | 仍需的运行证据 |
| --- | --- | --- |
| CAN 使用过时的 release 时间 | 每个槽处理时读取当前时间；填入 Message RAM 后、开启 TX 请求前再次检查 gate/epoch/deadline；过期 pending 直接丢弃 | 抢占与 deadline 边界下实际总线输出 |
| Fault 未撤销硬件 CAN 请求 | IRQ 屏蔽后检查 FDCAN 时钟；逐控制器写 TXBCR、请求 INIT 并关闭通知；不等待 HAL 或发送新帧 | 仲裁受阻、自动重传和故障进入时的硬件停止 |
| 浮点异常栈偏移错误 | 基本寄存器始终从异常 SP 读取，移除扩展帧的 18-word 偏移 | 非浮点/浮点/惰性压栈情况下 PC、LR 与实际异常点一致 |
| 遥控共享状态不一致 | I/O 独占 parser；短临界区发布快照并合并未消费边沿；control 通过独立 Take 接口消费邮箱 | 同优先级切换、连续边沿、断流/恢复时无撕裂或重复消费 |
| PID 限幅隐藏 Inf | 校验历史状态、误差、各计算阶段及限幅前中间结果；异常复位并输出零，dt 清零供电机检测 | 有限输入溢出、滤波/积分边界及旧行为对照 |
| UART abort 无截止时间 | 50 ms callback deadline；每次故障最多 3 次恢复尝试，失败间隔 100 ms；超时或耗尽则隔离端口 | DMA 错误、回调缺失/迟到与 TX buffer 所有权 |
| runtime 之前缺少启动监督 | reset assembly 在供电初始化前启动寄存器级 IWDG；供电等待增加迭代上限；runtime 切换至健康进度喂狗 | 早期启动失败、LSI 实际时间、复位记录和 Debug/Release 行为 |

ARM Cortex-M7 异常栈依据：[DUI 0646C，Figure 2-2 / 第 2-25 页](https://documentation-service.arm.com/static/61efd8352dd99944d05142d1)。浮点存储位于基本八个寄存器字之后，SP 指向基本帧的 R0。

附带完善了 DJI 候选快照的原子发布，并在读取时检查 CAN online/generation；总线错误与原始反馈丢失不会继续借用旧候选的在线状态。

## 2. 遥控交接契约

- `RmRemote_Push/Discontinuity/Take` 仅由 I/O 调用，其他任务不访问 parser 可变状态。
- I/O 每轮提取 parser 边沿，合并到已发布且同 generation、未过期的邮箱；解析和滤波在临界区外执行。
- `RmBringup_ReadRemote` 只读一致快照，并检查时间、transport online/generation；不消费边沿。
- `RmBringup_TakeRemote` 供唯一 control 消费者使用，在同一临界区拷贝并清除累计边沿。离线读取不会发布可用控制输入。
- parser 失效、generation 改变或旧邮箱超时后不继承旧边沿；恢复的首份有效输入继续遵循 parser 的首帧规则。

## 3. UART 隔离与诊断

隔离时撤销 UART DMA 请求、关闭 UART、请求停止所属 DMA stream 并屏蔽对应 IRQ；没有等待 stream 停止的循环，也不复用可能仍被 DMA 持有的 TX 存储。隔离是直到 MCU reset 的终态，迟到 callback 不会释放或重新启动端口。其他 UART 和共享 DMA 控制器继续工作。

`RmSerialStats` 新增 `online`、`quarantined`、`abort_timeouts`、`recovery_exhausted`。event 20 每秒报告端口状态和两项升级计数。成功恢复后清除本次故障的尝试次数；隔离端口不能被下一轮 service 自动重新启用。

## 4. 启动与异常限制

早期 IWDG 配置为 prescaler 256 / reload 1499，按 32 kHz LSI 名义约 12 s；实际时间受器件 LSI 偏差影响。早期配置和供电等待分别有 10000000 次迭代上限，依赖 reset 默认时钟，不宣称为精确微秒超时。board 初始化不续期此预算。runtime 使用原 prescaler 32 / reload 499，名义约 500 ms，健康任务负责 6 s bootstrap 内及其后的进度喂狗。

Debug 显式冻结 IWDG，Release 显式取消冻结。早期 IWDG 更新失败记录 `0x2008`，供电就绪等待失败记录 `0x2009`；路径不依赖已初始化的 C 全局状态。

异常抑制只能请求撤销尚未发送的硬件请求，已经开始的总线帧无法撤回；实际停止时序仍需板上确认。早期启动看门狗增加了硬件监督覆盖，不替代硬件时钟/供电失效场景的验收。

## 5. 构建记录

SBUS Debug、SBUS Release、DBUS Debug 与 host 四个 preset 已编译。产物与完整源码/构建配置 SHA-256 快照见 [build_manifest.json](../records/build_manifest_2026-10-11.json)；导入文件的适配哈希同步更新，原始来源哈希保留。

本轮未添加或运行测试，未烧录或进行硬件验证。以上修复状态表示源码已落地和编译成立，运行出口仍待按 [移植方案](../../architecture/porting_plan.md) 验收。
