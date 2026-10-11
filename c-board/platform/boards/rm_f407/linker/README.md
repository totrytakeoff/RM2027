# STM32F407IGHx 链接脚本

文档更新日期：2026-07-23

此目录包含 STM32F407IGHx 板级链接脚本。启动文件位于同级 `startup/`。

## 文件说明

- `STM32F407IGHx_FLASH.ld` - STM32F407IGHx的链接脚本，定义内存布局
- `../startup/startup_stm32f407xx.s` - STM32F407xx 启动文件

## 使用方法

在项目构建时，通过CMake或Makefile引用这些文件：

```makefile
LDSCRIPT = platform/boards/rm_f407/linker/STM32F407IGHx_FLASH.ld
```

或

```cmake
set(LINKER_SCRIPT "${RM_BOARD_ROOT}/linker/STM32F407IGHx_FLASH.ld")
```
