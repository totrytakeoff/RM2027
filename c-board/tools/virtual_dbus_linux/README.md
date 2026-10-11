# virtual_dbus_linux

文档更新日期：2026-07-23

Linux 上位机后台采集键盘鼠标（evdev `/dev/input/event*`），并以 DT7/DR16 的 **D-BUS 18Byte** 协议通过串口发送，供固件端 `remote_control` 模块直接复用解析。

## 编译

```bash
cd tools/virtual_dbus_linux
make
```

## 运行（Wayland 推荐方式）

Wayland 下想“后台全局采集键鼠”，最通用的方法是读取 evdev：通常需要 `root` 或配置 udev 权限。

```bash
sudo ./virtual_dbus --serial /dev/ttyACM0
```

如自动识别失败，显式指定键盘与鼠标设备：

```bash
ls -l /dev/input/by-id/
sudo ./virtual_dbus --serial /dev/ttyACM0 --kbd /dev/input/event3 --mouse /dev/input/event5
```

常用选项：

```bash
sudo ./virtual_dbus --serial /dev/ttyACM0 --hz 70 --mouse-scale 1.0 --invert-y
```

### Mock/调试串口输出

在无法创建串口或仅想观察帧内容时，可以开启干跑和 HEX 打印：

```bash
./virtual_dbus --serial mock --dry-run --print-hex --kbd /dev/null --mouse /dev/null --hz 20
```

## 协议与固件对齐

- 串口：`100000 bps, 8E1`（与 `platform/boards/rm_f407/generated/usart.c` 的 USART3 配置一致）
- 帧：固定 `18 Byte`，字段对齐 `docs/remote_control/dt7_dr16_protocol.md`
- 默认：
  - `switch_left=up`（使固件进入键鼠模式）
  - `switch_right=mid`（避免默认触发“解除急停”）
  - `ch0..ch4=1024`（摇杆/拨轮中位，拨轮不触发急停）
