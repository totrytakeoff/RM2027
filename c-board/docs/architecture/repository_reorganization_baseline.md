# RM2026 仓库重组基线

更新日期：2026-07-23

状态：顶层所有权收口已完成；API 细分、兼容模块清理与实板验收继续进行。

## 1. 目标

仓库不再把第三方代码、板级生成代码、平台适配、可复用组件、机器人行为和
固件组合当成一个“框架大库”。最终要达成：

1. 将裸机对照固件中已验证的步兵逻辑迁入稳定、可观测的 FreeRTOS 固件。
2. 建立仓库自有的路径、target、API、命名、测试和文档叙事。
3. 在开始控制参数优化前，建立可量化的安全、时序、内存和实板验收门禁。
4. 结构迁移期间不调 PID、执行器限幅、方向和模式转换逻辑。

## 2. 历史问题

重组前主要问题包括：

- 上游风格的大目录同时混合 CMSIS、STM32 HAL、FreeRTOS、USB、DSP、BSP、算法、
  设备驱动和服务。
- 递归源码/头文件搜索使内部实现全局可见，无法判断正式固件真正依赖哪些模块。
- 应用头文件暴露 `main.h` 和 HAL handle，算法头文件暴露 HAL/RTOS/DWT 实现细节。
- 正式 `app.elf` 需要链接组处理静态库循环依赖。
- FreeRTOS 虽禁用动态分配，但多个注册和算法路径仍使用 C 堆。
- 设备超时以健康任务调用次数表达，任务周期会改变真实超时时间。
- 中断中执行协议解析和高层状态发布，缺少有界入站队列。
- 顶层 `Inc`、`Src`、`hal`、`system`、`application`、`test`、`script` 和 `config` 所有权混乱。

## 3. 2026-07-19 结构收口结果

已完成：

- 删除废弃 `Src/application`，其历史仍可从 Git 追溯。
- 顶层 `Inc`、`Src`、`hal`、`system`、`application`、`test`、`script` 和 `config` 已消失。
- CubeMX 生成源、板级头、启动、链接、OpenOCD 和 HardFault 保留逻辑归属
  `platform/boards/rm_f407`。
- 正式入口、FreeRTOS 任务/hooks 和固件组合归属 `firmware/rm_f407`。
- 机器人行为、板级测试、主机测试和工具入口分别收口到
  `applications`、`tests/firmware`、`tests/host` 和 `scripts`。
- 删除未引用的 `minimal_config1.h` 与 `test_imu` 板级配置副本；备用底盘接线测试
  改用语义化名称 `infantry_chassis_gimbal_et08_alt_wiring_demo`。
- GitHub Actions 主机测试入口已同步为 `tests/host`，与本地验证命令一致。
- `HAL_Lib`、`HAL_IRQ` 和 `rm_system_freertos` 已替换为仓库自有 target。
- CMake 正式烧录/校验改用 `BIN + 0x08000000`；HEX/ELF 默认使用镜像内地址。
- HardFault 不再错误返回，而是保留异常栈帧和 SCB 故障寄存器到 `.noinit`。
- 正式路径使用静态任务和静态容量，链接后自动审计堆/格式化符号与 ELF 段权限。

当前 Debug 正式镜像：

- RAM：51,488 字节（39.28%）；
- Flash：97,792 字节（9.33%）；
- 主栈预留：8 KiB；
- C 堆预留：0 字节；
- 额外保留 HardFault 记录，看门狗复位后仍可用调试器读取。

## 4. 依赖方向

```text
applications ------> components/devices ------> platform ------> third_party
      |                       |
      +------> components/services + algorithms

firmware ------> applications + components + platform + FreeRTOS
```

规则：

- `third_party` 只保存外部依赖；STM32 HAL 编译期只注入当前板级的 HAL 配置头。
- `platform` 负责 MCU 和板级适配，不得依赖机器人应用。
- `components` 保存可复用算法、服务和设备，不得依赖具体机器人应用。
- `applications` 只描述机器人行为，目标是不直接暴露 HAL、FreeRTOS 和 CMSIS-OS。
- `firmware` 负责任务创建、板级绑定、中断/运行时组合和可执行入口。
- 正式控制路径只使用调用方存储或固定静态容量，禁止运行时动态分配。
- 超时使用单调时间戳和绝对时间单位，不使用调度调用次数。
- 中断只复制有界数据并发布事件，不阻塞、不格式化日志、不执行高层行为。

## 5. 当前目录基线

```text
RM2026/
|-- applications/robot/
|-- components/
|   |-- algorithms/
|   |-- devices/
|   `-- services/
|-- platform/
|   |-- common/
|   |-- stm32f4/
|   `-- boards/rm_f407/
|       |-- include/
|       |-- generated/
|       |-- startup/
|       |-- linker/
|       |-- openocd/
|       `-- fault/
|-- firmware/rm_f407/
|   `-- freertos/
|-- third_party/
|-- tests/
|   |-- host/
|   `-- firmware/{basic,hero,infantry,leg}/
|-- cmake/
|-- docs/
|-- scripts/
`-- tools/
```

## 6. 源码所有权映射

| 内容 | 归属 |
| --- | --- |
| CMSIS、STM32 HAL、FreeRTOS、USB Device、CMSIS-DSP、SEGGER RTT | `third_party` |
| CAN/UART/SPI/DWT/USB/时间/看门狗适配 | `platform/stm32f4` |
| CubeMX 生成源、板级头、启动、链接、OpenOCD、fault capture | `platform/boards/rm_f407` |
| 数学、PID、滤波、四元数 EKF、校验 | `components/algorithms` |
| 安全、设备健康、消息/传输服务 | `components/services` |
| 电机、IMU/INS、遥控、裁判、视觉 | `components/devices` |
| 机器人行为 | `applications/robot` |
| FreeRTOS 任务与正式固件组合 | `firmware/rm_f407` |
| 板级 bring-up、对照、回归固件 | `tests/firmware` |
| 纯算法、协议和状态机测试 | `tests/host` |

无正式消费者也无测试消费者的模块不进入正式框架。必须为其明确指定负责人、target
和验证路径，否则在兼容清理阶段移除。

遥控目录进一步按职责拆分：`components/devices/remote` 保存统一状态、C++ 适配器和
`dt7`、`et08`、`vt` 三个设备后端；`applications/robot/command` 只保存当前机器人语义映射。
Pitch 角度边界、底盘速度边界等机械约束属于对应执行模块，不进入遥控层。

## 7. 命名与 CMake 规则

- 新 C 文件使用 `snake_case.c/.h`。
- 公开类型使用 `RmXxx`，公开函数使用 `RmXxx_Action`，公开宏使用 `RM_XXX`。
- CMake 实现 target 使用 `rm_xxx`，alias 使用 `RM::xxx`。
- 每个维护中的 target 显式列出源码，禁止 `GLOB_RECURSE`。
- 依赖默认 `PRIVATE`；只在公开头实际暴露依赖时使用 `PUBLIC`。
- 主机构建与 ARM 交叉构建必须使用独立构建树。
- 来源战队命名不出现在普通路径、target、API 和项目叙事中；归属只在
  `THIRD_PARTY_NOTICES.md` 和必要架构历史中保留。

## 8. 验证命令

```bash
cmake -S tests/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure

cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target app.elf test_infantry_minimal --parallel
cmake --build build --parallel
```

每次结构修改还必须：

- 记录正式镜像 RAM/Flash；
- 通过堆/格式化符号和 ELF 非 RWX 审计；
- 保持控制参数和执行顺序，除非任务明确授权改变；
- 更新当前日期的中文基线记录。

## 9. 未完成验收

- [ ] 应用公开头不再暴露 HAL/FreeRTOS/CMSIS-OS。
- [ ] `app.elf` 不再需要链接组。
- [ ] 拆分 `rm_components_runtime`，移除不再支持的兼容驱动和重复 demo。
- [ ] 建立 INS 运行时 data-ready/合理性健康截止。
- [ ] 实板验证输入丢失、急停、任务故障和设备故障会关闭所有执行器。
- [ ] 实板记录任务最大执行时间、释放间隔、栈余量、CAN 邮箱压力与长时间运行结果。

上述项完成前，当前代码是“可编译、可进入实板验收”的迁移基线，不是已完成赛场验收的最终固件。
