# OpenOCD 调试配置

文档更新日期：2026-07-23

此目录包含用于OpenOCD的调试器配置文件。

## 文件说明

- `openocd_dap.cfg` - 使用CMSIS-DAP调试器的配置
- `openocd_jlink.cfg` - 使用J-Link调试器的配置
- `openocd_stlink.cfg` - 使用ST-Link调试器的配置

## 使用方法

在下载程序时，选择对应的配置文件：

```bash
# 使用CMSIS-DAP
openocd -f platform/boards/rm_f407/openocd/openocd_dap.cfg \
  -c "program build/output/app.elf verify reset exit"

# 使用J-Link
openocd -f platform/boards/rm_f407/openocd/openocd_jlink.cfg \
  -c "program build/output/app.elf verify reset exit"

# 使用ST-Link
openocd -f platform/boards/rm_f407/openocd/openocd_stlink.cfg \
  -c "program build/output/app.elf verify reset exit"
```

使用仓库脚本时默认烧录 `build/output/app.bin` 到 `0x08000000`。
ELF/HEX 镜像已带地址，不应额外叠加 Flash 基地址。
