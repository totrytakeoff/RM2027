# infantry_gimbal_et08_demo

目标：把 infantry 的云台控制链路独立出来，单独验证 ET08 输入与 yaw/pitch 控制。

## 功能
- ET08(SBUS, USART3) 读取摇杆。
- 仅控制 2 个 GM6020：Yaw(CAN1 ID1)、Pitch(CAN2 ID1)。
- Pitch 支持“手动速度 + 松杆保持角度”控制。
- 遥控离线 / failsafe / frame_lost 时立即停机。
- USART6 输出关键调试信息（输入、模式、参考值、反馈值）。

## 调试记录
- 已确认坑点与规避规则见 `known_pitfalls.md`

## 摇杆映射
- `right.x` -> `yaw_speed_ref`
- `right.y` -> `pitch_speed_ref`
- `SD` 上/下：云台模式标记（仅调试输出，不依赖底盘）

## 构建
```bash
cmake -S . -B build
cmake --build build --target test_infantry_gimbal_et08_demo -j8
```

## 烧录
```bash
cmake --build build --target flash-test_infantry_gimbal_et08_demo
```
