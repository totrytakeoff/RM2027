# RM2026 机器人控制源码

文档更新日期：2026-07-23

本仓库是面向 RoboMaster 2026 赛季、基于 STM32F407IGT6 的机器人电控固件。
当前主目标为单板步兵固件：机器人行为位于 `applications/robot`，
STM32F4 通用适配位于 `platform/stm32f4`，具体控制板和 FreeRTOS 固件组合分别由
`platform/boards/rm_f407` 与 `firmware/rm_f407` 管理。保留的裸机对照固件
与正式 FreeRTOS 固件复用同一套应用逻辑。

## 构建与测试

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target app.elf --parallel
cmake --build build --parallel

# 主机纯逻辑测试，使用本机编译器
cmake -S tests/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

所有可烧录的 `ELF/HEX/BIN` 产物统一收集到 `build/output`。
构建、烧录、GDB 和 RTT 日志入口位于 `scripts`。

## 仓库分层

- `applications/`：机器人行为、状态和参数。
- `components/`：可复用算法、服务和设备驱动。
- `platform/stm32f4/`：MCU-family 级平台适配。
- `platform/boards/rm_f407/`：CubeMX 生成源、板级头、启动、链接、异常保留与 OpenOCD 配置。
- `firmware/rm_f407/`：正式入口、FreeRTOS 任务策略和固件组合。
- `third_party/`：保留原始许可说明的外部依赖。
- `tests/firmware/`：板级 bring-up、对照和回归固件。
- `tests/host/`：可在主机上独立执行的纯逻辑测试。
- `docs/`：架构基线、迁移记录和上机验收项。
- `scripts/`：构建、烧录、调试和日志脚本。

## 当前状态

`tests/firmware/infantry/infantry_minimal` 中验证过的步兵逻辑已迁入正式
FreeRTOS `app.elf`。目录所有权、静态任务、输入安全、设备健康、命令租约、
独立看门狗、无堆正式路径和链接审计已建立。在标记为比赛可用前，仍必须完成
实板方向、故障停机、时序、栈余量、CAN 负载和长时间 soak 验收。

详细状态见：

- [仓库重组基线](docs/architecture/repository_reorganization_baseline.md)
- [遥控适配器与机器人输入基线](docs/architecture/remote_control_adapter_baseline.md)
- [步兵 FreeRTOS 迁移基线](docs/migration/infantry_freertos_baseline.md)
- [安全与任务健康服务](docs/migration/safety_health_service.md)
- [运行时加固里程碑](docs/migration/runtime_hardening_milestone.md)
- [机器人核心控制参数索引](docs/motor/control_parameter_reference.md)
- [机器人 UART6/VOFA+ 调参遥测](docs/debug/tuning_telemetry.md)

## 来源说明

项目早期架构和部分迁移代码参考了跃鹿战队开源 `basic_framework`。
当前项目路径、target 和 API 使用仓库自有命名。完整归属与许可说明见
[第三方来源与许可说明](THIRD_PARTY_NOTICES.md)。
