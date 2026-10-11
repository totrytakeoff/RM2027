# RM2026 CMake target 维护说明

文档更新日期：2026-07-23

本文说明新增或调整源码时应修改哪个 `CMakeLists.txt`。构建命令和 target
总表见 [CMake 构建指南](README_CMake.md)。

## 根目录加载顺序

```text
third_party
platform
components
applications
firmware
tests
```

这个顺序表达总体依赖方向。板级 HAL 配置头是 vendor HAL 的编译期注入配置；
除此板级配置点外，低层 target 不得反向依赖机器人应用。

## 各层职责

### `third_party/CMakeLists.txt`

每个外部依赖使用独立 target、独立公开头文件路径和显式源码列表。
FreeRTOS 的 `heap_4.c` 被明确排除，因为正式配置禁止动态分配。

### `platform/boards/rm_f407/CMakeLists.txt`

`rm_board_rm_f407_headers` 公开板级生成头；
`rm_board_rm_f407` 编译 CubeMX 生成源与启动文件；
`rm_board_rm_f407_irq` 作为 OBJECT library 保证中断和 fault capture 直接进入镜像。
链接脚本、OpenOCD 和板级说明也只属于该目录。

### `platform/stm32f4/CMakeLists.txt`

维护 MCU-family 适配源码和公开头文件。平台层可以依赖 vendor 与板级头，
不能依赖机器人应用。

### `components/CMakeLists.txt`

`rm_components_runtime` 是正式机器人固件支持的组件集；`rm_components_compat` 只服务
历史 demo；`rm_components` 是 demo 兼容伞。新正式功能应建立窄 target，不应扩大兼容伞。

### `applications/robot` 与 `firmware/rm_f407`

- `applications/robot` 只描述机器人行为、状态和控制参数。
- `firmware/rm_f407/freertos` 管理任务入口、周期、栈、优先级和 hooks。
- `firmware/rm_f407` 组合应用、运行时、板级绑定，并生成 `app.elf`。

## 新增代码

新增仓库自有组件：

1. 放入 `components/algorithms`、`components/devices` 或 `components/services`。
2. 建立窄 target，只公开 API 需要的 include 目录。
3. 将 HAL/传输细节留在 platform 或设备适配层。
4. 先为纯逻辑补充主机测试，再接入正式固件。

新增板级测试：

1. 在 `tests/firmware/<category>/<name>/` 添加源码与 `CMakeLists.txt`。
2. 使用 `create_embedded_test()`，显式填写 `LINK_LIBRARIES`。
3. 将相对目录登记到 `tests/CMakeLists.txt`的 `RM_EMBEDDED_TEST_DIRS`。
4. 构建单个 `test_<name>` target，并在提交前执行全量构建。

新增主机测试时修改 `tests/host/CMakeLists.txt`，不要将其加入 ARM 交叉编译树。

## 提交前检查

```bash
rg 'GLOB_RECURSE|target_include_directories_recursively' \
  third_party platform components applications firmware tests
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel
cmake -S tests/host -B build-host -DCMAKE_BUILD_TYPE=Debug
cmake --build build-host --parallel
ctest --test-dir build-host --output-on-failure
```

第一条命令应无输出。来源与许可说明只保留在
`THIRD_PARTY_NOTICES.md` 及必要的架构历史文档中。
