# RM2027 · 达妙 MC-02

基于 STM32H723 的机器人控制框架，延续 RM2026 的分层与静态运行模型。

**当前阶段：基础框架已完成实板运行和故障恢复验证，四 M3508 全向轮底盘 demo 已由用户确认基本可用。** 完整 RobotApp 和后续设备功能仍按移植方案推进。验收范围见 [当前状态](docs/validation/README.md)。

## 快速使用

在工程根目录执行：

```sh
./scripts/build.sh                         # 默认构建 chassis_demo.elf
./scripts/upload.sh chassis_demo           # 烧录已有 BIN，默认 DAP v2
./scripts/build.sh upload                  # 构建并烧录
./scripts/log.sh                           # 底盘 USART1，115200
```

烧录已有固件只需要 Bash（Windows 使用 PowerShell）和 OpenOCD；构建另需 CMake ≥3.21、Ninja、ARM GCC。日志/调试工具的依赖见 [脚本说明](scripts/README.md)。

基础诊断固件：

```sh
./scripts/build.sh build --target board_bringup
./scripts/upload.sh board_bringup
```

固件位于 `build/<preset>/`。底盘运行版绑定 CAN1 四电机，基础 `board_bringup` 不绑定执行器；两者的串口用途和输出策略不同，详见使用指南。

## 文档入口

| 需要了解什么 | 文档 |
| --- | --- |
| 全部文档与阅读顺序 | [文档目录](docs/README.md) |
| 底盘接线、遥控操作、IMU yaw 与参数 | [底盘 demo](docs/guides/chassis_demo.md) |
| 构建、基础诊断、日志含义 | [构建与板级诊断](docs/guides/build_and_bringup.md) |
| 脚本命令与参数 | [常用脚本](scripts/README.md) |
| 分层、接口契约与后续移植目标 | [架构与移植方案](docs/architecture/porting_plan.md) |
| 引脚、时钟、缓存和 DMA 分区 | [硬件基线](docs/architecture/hardware_baseline.md) |
| 已验证范围、证据与剩余工作 | [当前验收状态](docs/validation/README.md) |
| 文件来源、版本与适配边界 | [来源记录](docs/reference/provenance.md) |
| 早期进度、审计与联调过程 | [历史归档](docs/archive/README.md) |

## 配置与源码

- 板级资源：`config/boards/dm_mc02/resources.h`。
- 底盘配置与绑定：`config/robots/chassis_demo/`，电机 ID、遥控极性、机械参数和 PID 以源码为准。
- 应用：`applications/board_bringup/`、`applications/chassis_demo/`。
- 框架：`components/`、`platform/`、`runtime/`。

本工程独立构建，不通过绝对路径依赖 RM2026 或参考 demo。`platform/boards/dm_mc02/reference/dm02_test.ioc` 是原始参考配置，不能用于重新生成当前源码。
