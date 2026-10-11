# MC-02 硬件与平台基线

板级基线与应用角色分开：底盘 demo 的 USART1 用于115200日志，实际参数见 [底盘指南](../guides/chassis_demo.md)；当前验收范围见 [验收状态](../validation/README.md)。

状态：资料与源码交叉核对后的实施基线；接线、时序、电压和负载仍需实物验收。引脚表以官方 V1.0 原理图、引脚图和综合 demo 为依据，手册采用 V1.1。

## 1. MCU、时钟和构建

| 项目 | 基线 |
| --- | --- |
| MCU | STM32H723VGT6，Cortex-M7，带双精度 FPU |
| 晶振 | HSE 24 MHz |
| 首版 CPU / HCLK | 480 / 240 MHz，参考综合 demo；不以芯片最高 550 MHz 作为首版时钟 |
| APB1/APB2 | 参考基线 120 MHz；定时器实际 kernel clock 结合 TIMPRE/分频计算 |
| Flash | 内部 1 MiB，基址 `0x08000000` |
| 外置 Flash | W25Q64JV，64 Mbit = 8 MiB；不作为首版启动或控制的依赖 |
| CPU flags | `-mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard`，在平台目标内统一 |
| 优化 | Debug `-Og`，发布初期 `-O2`；不全局使用 `-Ofast` / `-ffast-math` |
| HAL / CMSIS | 选一套配套来源，记录包版本；主要 demo HAL 版本 1.11.5 |

综合 demo 的 PLL1 为 `M=2, N=40, P=1, Q=4, R=2`，VCO 480 MHz、PLL1Q 120 MHz。**新工程初始方案采用官方独立 FDCAN 例程的 Q=6，即 FDCAN 80 MHz；CPU 仍为 480 MHz。** 在完整时钟表中核对所有 PLL1Q 使用者，并按 H723 对应参考手册的 kernel clock 限制确认。电源配置、VOS、Flash latency 与时钟作为一组板级配置保存。

SPI2 参考 PLL3P 为 80 MHz，分频 8 后 SCK 10 MHz，模式 3。先按 BMI088 数据手册和总线实测核对，不把“SPI 分频相同”当成 F4/H7 速率相同。

CMSIS-DSP 优先使用匹配版本源码构建所需模块。若采用预编译库，核对 hard-float ABI、内核和浮点指令配置；参考 demo 的 `fpv4-sp-d16` 编译选项不能直接成为 H723 的统一配置。

## 2. 首版引脚和功能分配

“建议角色”是工程默认配置，可在机器人接线 profile 修改；“硬件连接”属于板级事实。

| 外设 | 硬件连接 | 建议角色 / 注意事项 |
| --- | --- | --- |
| FDCAN1 | RX PD0，TX PD1 | 延续旧 CAN1 的电机接线角色 |
| FDCAN2 | RX PB5，TX PB6 | 延续旧 CAN2 的电机接线角色 |
| FDCAN3 | RX PD12，TX PD13 | 扩展总线；达妙电机等按实际需要加入 |
| UART5 | RX PD2 | 遥控；仅接收，禁止配置 PB13 TX |
| USART1 | TX PA9，RX PA10 | 视觉；初始串口格式沿用旧视觉链路 921600 8N1 |
| UART7 | TX PE8，RX PE7 | 裁判系统；独立于视觉，波特率按协议配置 |
| USART10 | TX PE3，RX PE2 | 调试/调参；基础固件默认日志；当前未接 SEGGER RTT |
| USART2 / RS485 | TX PD5，RX PD6，DE PD4 | 可选扩展，初期关闭 |
| USART3 / RS485 | TX PD8，RX PD9，DE PB14 | 可选扩展，初期关闭 |
| BMI088 SPI2 | SCK PB13，MOSI PC1，MISO PC2 | 双片选共享一条 SPI，总线内串行访问 |
| BMI088 片选 | Accel PC0，Gyro PC3 | 上电先置为不选中；访问流程区分 accel dummy byte |
| BMI088 DRDY | Accel PE10，Gyro PE12 | EXTI：采样时刻记录和采集事件 |
| IMU 加热 | TIM3_CH4 / PB1 | 默认关闭；控制接口使用归一化 duty |
| WS2812 | SPI6 MOSI / PA7 | SPI6/BDMA 缓冲使用 D3 SRAM4 |
| 蜂鸣器 | TIM12_CH2 / PB15 | 低优先级状态指示 |
| 用户按键 | PA15 | 去抖和事件在任务中处理 |
| 输入电压 ADC | ADC1_CH4 / PC4 | 按原理图分压、参考电压和 ADC 分辨率换算 |
| 可控输出 1 / 2 | PC14 / PC13 | 原理图推导为高电平使能，上板确认；默认关闭 |
| 对外 5V 使能 | PC15 | 原理图推导为高电平使能；按所选接口供电需求开启 |
| USB FS | DM PA11，DP PA12 | 可选 CDC；首个可运行目标不依赖 USB 枚举 |
| SWD | PA13 / PA14 | 调试与下载 |

扩展预留：UART8 RX PE0 / TX PE1；UART9 RX PD14 / TX PD15；SPI3 PC10/PC11/PC12；扩展 I2C PB8/PB9。LCD 使用 SPI1 PB3/PB4/PD7 和 PE15 CS，并占用 I2C2 PB10/PB11、PD10、PA5 等资源。摄像头、LCD、RS485、OSPI 的组合必须经统一资源表审核，首版不统一初始化这些外设。

SBUS 采用 100000、8 数据位、偶校验、2 停止位；在 HAL 中对应 9-bit word length + EVEN。板载反相电路见原理图，不能再次盲目开启 UART RX inversion。DT7 的 DBUS 后端按其实际线格式配置。接收器型号及接线通过 P2 的原始帧观察确认。

## 3. 定时器、IRQ 与 DMA

### 定时器归属

| 资源 | 唯一所有者 |
| --- | --- |
| SysTick / PendSV / SVC | FreeRTOS |
| TIM6 | HAL 毫秒 timebase，独立于 RTOS tick |
| TIM5 | 32 位自由运行微秒计数器，扩展为 64 位单调时钟 |
| DWT | 短时性能测量，不直接暴露给 PID 和应用 |
| TIM3_CH4 | IMU 加热 |
| TIM12_CH2 | 蜂鸣器 |
| TIM1 / TIM2 等 | 按机器人可选 PWM 资源登记 |

不继承参考 demo 的 TIM4 10 μs、TIM8 125 μs 业务轮询。TIM5 回绕约 71.6 分钟，通过低频溢出维护和一致读取扩展；P1/P6 要覆盖回绕场景。DWT 在 480 MHz 下约 8.95 秒回绕，不能把单次读取当成长时绝对时间。

### DMA 初始资源分配

以下尽量保留综合 demo 的已知映射，未启用的资源只预留、不启动。

| 用途 | 资源 | 首版 |
| --- | --- | --- |
| UART5 RX | DMA1 Stream4 | 启用 |
| USART1 RX / TX | DMA1 Stream5 / Stream6 | 视觉启用时使用 |
| UART7 RX / TX | DMA1 Stream7 / DMA2 Stream0 | 裁判链路启用时使用 |
| USART10 RX / TX | DMA2 Stream1 / Stream2 | 调试链路启用时使用 |
| SPI2 RX / TX | DMA2 Stream3 / Stream4 | 正式异步 IMU 使用 |
| ADC1 | DMA2 Stream5 | 需要周期采集时使用 |
| USART2 RX / TX | DMA1 Stream1 / Stream0 | 预留 |
| USART3 RX / TX | DMA1 Stream2 / Stream3 | 预留 |
| SPI6 TX | BDMA，具体 channel 由统一 `.ioc` 分配 | 可选灯效阶段 |

使用 `NVIC_PRIORITYGROUP_4`，4 位抢占优先级。FreeRTOS syscall 阈值初定 5；调用 `...FromISR` 的 DMA、UART、FDCAN 和 EXTI 使用数值 5～15 的优先级。初版采集 IRQ 建议 5、一般通信 6、HAL timebase 7、RTOS 内核 15，按实测调整。优先级 0～4 的 ISR 不调用 RTOS API，也不写由 BASEPRI 临界区保护的共享对象。

所有中断向量和 HAL 回调集中在板级入口分发，避免模块分别定义同名 HAL callback。复用共享 IRQ 时显式判断实际 pending 来源。

## 4. H723 内存与缓存

采用当前参考工程的默认 RAM 分配，初期不调整 AXI/ITCM remap。片内 RAM 的“564 KiB”包含 4 KiB backup RAM，不等于一块连续通用 SRAM。

| 区域 | 基址 | 容量 | 计划用途 |
| --- | --- | --- | --- |
| ITCM | `0x00000000` | 64 KiB | 初期保留；性能证据充分后放关键代码 |
| DTCM | `0x20000000` | 128 KiB | 任务栈、CPU 专用控制/EKF 工作区；禁止普通 DMA1/2 buffer 放入 |
| AXI SRAM / D1 | `0x24000000` | 320 KiB | 普通 `.data/.bss`、设备状态、软件队列 |
| SRAM1+SRAM2 / D2 | `0x30000000` | 32 KiB | `.dma_buffer`，UART/SPI2/ADC 等 DMA1/2 缓冲 |
| SRAM4 / D3 | `0x38000000` | 16 KiB | `.bdma_buffer`；另留独立的 `.noinit_fault` 区域 |
| Backup SRAM | `0x38800000` | 4 KiB | 初期不用；以后可作有保留条件的故障记录 |

依据本地 DS13313 Rev4 §3.3.2：DTCM/ITCM 可由 CPU 与 MDMA 访问；不要据此认为 DMA1/2 同样可以访问 TCM。DMA 访问权限按 H723 的总线结构核对，不能从其他型号或缓存应用笔记的示意图类推。后续实现参照 [ST RM0468](https://www.st.com/resource/en/reference_manual/rm0468-stm32h723733-stm32h725735-and-stm32h730-value-line-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)。

正式版本的内存策略：

1. MPU 先设置 D2 32 KiB 和 D3 16 KiB 为 Normal non-cacheable、execute-never；AXI 作为可缓存数据区。AXI 320 KiB 可用 256 KiB + 64 KiB 两个对齐 MPU region 表达。
2. 再使用 CMSIS 函数启用 I-Cache 和 D-Cache；DMA 原始收发存储必须落在指定 section，不能仅加 `volatile`。
3. DMA section 中的 buffer 按 32 字节对齐，地址与长度在平台层校验；CPU 软件队列、解析状态仍在普通 RAM。
4. 初版不让设备层管理 cache clean/invalidate。未来允许 DMA 使用可缓存区时，由传输层集中处理独占 cache line 和维护操作。

该策略依据 [ST AN4839 的 DMA 一致性与非缓存缓冲建议](https://www.st.com/resource/en/application_note/an4839-level-1-cache-on-stm32f7-series-and-stm32h7-series-stmicroelectronics.pdf)。

链接脚本、startup 初始化表和 MPU 配置必须一并实现：新增 `.data` 区域需复制，新增 `.bss` / DMA NOLOAD 区域需显式清零；`.noinit_fault` 保留并用 magic、版本及校验判断是否有效。warm reset 保留不等于掉电保留。

初始预算上限建议：DTCM 已分配小于 96 KiB；D2 DMA 小于 8 KiB；D3 使用小于 4 KiB；AXI 先控制在 192 KiB 内；内部 Flash 先控制在 768 KiB 内。均为规划约束，实际容量与余量以链接 map 和栈高水位为准。沿用静态分配，不照搬 demo 的 128 KiB heap + 64 KiB 主栈。

## 5. FDCAN 基线

三路初期统一经典 CAN、标准电机帧 8 字节、nominal 1 Mbps。FDCAN kernel clock 计划为 80 MHz，分频及位时间需要结合 RCC 和 FDCAN_CKDIV 实际配置核对。

计划参数：prescaler 1、seg1 59、seg2 20、SJW 20，延用官方 FDCAN 例程的 nominal timing。

```text
bitrate = 80 MHz / [1 × (1 + 59 + 20)] = 1 Mbps
sample point = (1 + 59) / 80 = 75%
```

这是可复现的起点；综合 demo 则使用 120 MHz、prescaler 3、seg1 29、seg2 10，同样推导为 1 Mbps。两套配置不能混用。是否调整 sample point 按总线长度和通信实测决定，驱动创建时打印实际 kernel clock 和推导 bitrate。

H723 三路控制器共享 10 KiB = 2560 个 32-bit word 的消息 RAM，本地数据手册 §3.42 已明确。以下是新工程初始规划，不是现有 demo 配置：

| 内容 / 每路 | 数量 | words |
| --- | --- | --- |
| 标准过滤器 | 16 | 16 |
| 扩展过滤器 | 8 | 16 |
| Rx FIFO0，8-byte element | 32 | 128 |
| Rx FIFO1，8-byte element | 8 | 32 |
| Dedicated TX buffer，8-byte element | 3 | 12 |
| Tx Event FIFO | 3 | 6 |
| 合计 | — | 210 |

每路预留 512 words，`MessageRAMOffset` 分别 0、512、1024；三区总预留 1536 words，剩余 1024 words 未使用。offset 单位是 word。静态断言每个实际占用不超过 slot、总量不超过 2560；使用 HAL 的实际布局公式再核对。首版 TX 使用 dedicated buffer，FIFO queue 为 0，便于跟踪和取消过期请求。

采用精确 ID 过滤，拒绝无关帧和远程帧。平台把 HAL 的 DLC 转换为实际字节长度，设备不接触 HAL 常量。FD/BRS、64 字节 payload 和不同波特率后续按设备需求配置并重新计算消息 RAM。

TX 由电机/传输服务管理固定请求和 deadline，按 bus + ID 合并尚未提交的电机命令。停止时禁止新非零请求，取消尚未开始的旧请求并安排停止帧；已经在总线上发送的帧无法撤回，总线故障时也无法保证零帧到达，需依赖执行器自身超时与实际供电策略。记录这种情况，不把 HAL 接受请求当成总线已完成发送。

## 6. 上电、电源和温控

1. 先建立安全 GPIO：片选释放，heater duty=0，可控电源输出关闭，RS485 DE 不发送。
2. 对外 5V 按外设需要启动；CAN transceiver 和外接设备的实际电源路径分别确认。不能将关闭可控 XT30 当成关闭 CAN 收发器。
3. 读取复位原因，初始化时钟、MPU、时间基准，再启动通信和运行器。
4. BMI088 加热电阻接输入电源；根据实测电压、加热响应和采样有效性确定 duty cap、温度目标、超温阈值及温控周期。初版默认关闭，温控是显式功能开关。
5. 温控使用归一化 duty，由板级 PWM 驱动映射 ARR/CCR。传感器无新数据、通信错误、温度非法或超温时立即清零 duty；健康监督在 INS/采集失去进度时也执行清零，避免只依赖已经停滞的温控任务。不能用 C 板 CCR 数字作为 H7 占空比。
6. 软件启动 IWDG 的时机必须覆盖 INS 初始化/标定流程。可在有界 bootstrap 后由健康任务管理；若 option bytes 已配置硬件自动启动，则 bootstrap 必须提供有进度约束的喂狗策略。读取配置后确定，避免长标定导致复位循环。
