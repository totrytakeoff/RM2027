# DM-IMU-L1 Qt 上位机

基于 Qt Widgets + Qt SerialPort 的 IMU 上位机，支持：
- USB 虚拟串口连接
- 实时数据显示（加速度/角速度/欧拉角/四元数）
- 配置输出接口/输出项/频率/ID/温控
- 校准与维护指令
- 自定义十六进制指令发送

## 构建

需要 Qt5/Qt6（Widgets + SerialPort 模块）。

```
cmake -S tools/dm_imu_gui -B build/dm_imu_gui
cmake --build build/dm_imu_gui -j
```

## 运行

```
./build/dm_imu_gui/dm_imu_gui
```

## 使用提示

- 配置类指令建议先进入“设置模式”。
- 修改配置后点击“Save Params”保存。
- 数据解析默认启用 CRC16 校验，可在 Data 页关闭。

