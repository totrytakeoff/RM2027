# STM32F4 UART 适配层

该模块管理独占 UART 端点、receive-to-idle DMA、错误恢复和回调分发。端点、
DMA 缓冲和收件箱全部来自固定容量静态存储。

## 资源约束

- 最多注册 `USART_MAX_INSTANCES`（当前 3）个端点；
- 每个 UART 句柄只能注册一次；
- 单次 DMA 接收长度为 1–256 字节；
- 延迟模式下每个端点保留最近 4 个 DMA/IDLE 事件；
- 注册仅允许发生在单线程启动阶段。

配置示例：

```c
const USART_Init_Config_s config = {
    .usart_handle = &huart3,
    .recv_buff_size = 25U,
    .module_callback = RemoteFrameCallback,
};
USARTInstance *endpoint = USARTRegister(&config);
```

注册或 DMA 启动失败时返回 `NULL`。`USARTServiceInit()` 用于恢复接收；HAL
报告 busy 时会先中止陈旧接收，再尝试启动一次。

启动阶段如果后续设备注册失败，可对刚注册的最后一个端点调用
`USARTUnregister(endpoint)`。该操作先尝试停止接收 DMA，再释放端点和收件箱槽位；
为了保持已经返回的实例地址稳定，只允许按后进先出顺序注销；DMA 无法安全停止时
返回 `false` 并保留原端点。

## 回调上下文

系统提供两种全局分发模式：

- `USART_DISPATCH_INTERRUPT`：默认兼容模式，在 UART 中断中直接调用协议回调；
- `USART_DISPATCH_DEFERRED`：中断只复制 DMA 事件，任务调用
  `USARTDispatchPending()` 后执行协议回调。

必须在第一个端点注册前调用 `USARTConfigureDispatch()`。正式机器人固件使用延迟
模式，独立 demo 默认保持中断模式。

延迟模式把 DMA 缓冲与回调可见的 `recv_buff` 分离：DMA 可以立即重新启动，
任务解析期间缓冲内容不会被下一次接收覆盖。收件箱满时淘汰最旧事件并保留最新
输入，防止启动阶段或短时调度抖动回放过时遥控指令。协议仍应通过 `recv_len`
验证长度，并自行完成帧头、CRC 和流式重组检查。

## 发送与诊断

`USARTSend()` 支持阻塞、IT 和 DMA 模式并返回 HAL 是否成功入队。连续异步发送
仍需要上层发送队列，调用方可用 `USARTIsReady()` 判断发送状态。

`USARTGetDispatchStats()` 返回接收、分发、覆盖、拒绝、UART 错误和恢复失败
计数；各端点中也保留同类 debugger 可见计数器。中断路径不解析协议、不格式化
日志，也不执行设备级恢复策略。
