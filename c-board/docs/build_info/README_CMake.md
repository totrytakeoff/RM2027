# RM2026 CMake 构建指南

文档更新日期：2026-07-23

当前构建系统按源码所有权组织，不再通过上游风格的大库聚合第三方代码、
板级生成代码、平台适配和机器人逻辑。目录目标与迁移约束见
[仓库重组基线](../architecture/repository_reorganization_baseline.md)。

## 构建层次

```text
third_party -> platform/boards + platform/stm32f4 -> components
            -> applications -> firmware/rm_f407 -> app.elf
```

- `third_party/`：CMSIS、CMSIS-DSP、STM32 HAL、FreeRTOS、USB Device 和
  SEGGER RTT。
- `platform/boards/rm_f407/`：CubeMX 生成源、板级头文件、启动、
  链接、异常保留与 OpenOCD 配置。
- `platform/stm32f4/`：CAN、UART、SPI、DWT、日志、USB、看门狗等平台适配。
- `components/`：算法、设备驱动和通用服务。
- `applications/robot/`：机器人行为与参数。
- `firmware/rm_f407/`：正式入口、FreeRTOS 任务策略与固件组合。
- `tests/firmware/` 与 `tests/host/`：板级固件回归与本机纯逻辑测试。

## 环境与命令

需要 CMake 3.16 以上版本，以及可从 `PATH` 找到的
`arm-none-eabi-gcc/g++/objcopy/size`。OpenOCD 只在烧录、校验和调试时需要。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --target app.elf test_infantry_minimal --parallel
cmake --build build --parallel

cmake -S tests/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

正式 `ELF/HEX/BIN` 和所有可烧录测试产物统一收集到
`build/output/`。分类测试产物同时位于 `build/tests/<name>/`。

```bash
# 仅检查烧录命令，不访问板子
./scripts/upload.sh --dry-run

# 实际烧录/校验
cmake --build build --target upload
cmake --build build --target verify
```

BIN 镜像使用 `0x08000000` 基地址。HEX/ELF 默认使用文件内嵌地址，
不得再额外叠加 Flash 基地址。

## 主要 target

| 层 | 实现 target | CMake alias |
| --- | --- | --- |
| STM32 HAL driver | `rm_vendor_stm32_hal` | `RM::vendor_stm32_hal` |
| FreeRTOS | `rm_vendor_freertos` | `RM::vendor_freertos` |
| board headers | `rm_board_rm_f407_headers` | `RM::board_rm_f407_headers` |
| board generated/startup | `rm_board_rm_f407` | `RM::board_rm_f407` |
| STM32F4 platform | `rm_platform_stm32f4` | `RM::platform_stm32f4` |
| formal components | `rm_components_runtime` | `RM::components_runtime` |
| compatibility components | `rm_components_compat` | - |
| safety service | `rm_service_safety` | `RM::service_safety` |
| robot application | `rm_robot_app` | `RM::robot_app` |
| robot FreeRTOS policy | `rm_robot_freertos` | `RM::robot_freertos` |
| robot firmware | `rm_robot_firmware` | `RM::robot_firmware` |
| formal image | `app.elf` | - |

`rm_components` 只为历史 demo 保留兼容聚合，不会被正式 `app.elf` 链入。
正式镜像仍使用链接组处理存量循环依赖，这是后续 API/target 细分的明确技术债。

## 构建约束

- 维护中的 target 必须显式列出源码和公开 include 目录。
- 新代码不得使用递归源码或递归 include 搜索。
- 正式控制路径不得引入动态分配。
- 结构调整必须同时通过主机测试、`app.elf`、
  `test_infantry_minimal` 与全量板级构建。
- 正式镜像链接后会自动审计堆/格式化符号和 ELF LOAD 段权限。
