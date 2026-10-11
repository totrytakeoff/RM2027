# VT 链路监视测试（USART6 RX + USART1 TX）

文档更新日期：2026-07-19

## 目的

用于隔离排查图传链路问题：

- `USART6`：接收 VT03/VT13 遥控帧（`921600 8N1`）
- `USART1`：输出调试文本

输出格式对齐 `scripts/vt_serial_monitor.py`，包含：

- `[LINK] ONLINE/OFFLINE`
- `[STAT] online valid invalid header crc_ok fps last_valid_age bytes drop`
- `[DATA]` 多行字段（gear/ch/mouse/keyboard）

## 构建

```bash
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
ninja -C build test_vt_link_monitor_demo
```

## 烧录

```bash
./scripts/upload.sh test_vt_link_monitor_demo --format bin
```

## 串口观察

- 调试输出串口：`USART1`
- 建议波特率：`921600`
