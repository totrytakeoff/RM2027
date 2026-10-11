# infantry_shoot_et08_demo

目标：把 infantry 的发射逻辑独立出来，单独验证 ET08 输入与摩擦轮/拨弹控制。

## 功能
- ET08(SBUS, USART3) 读取开关量。
- 仅控制发射机构：摩擦轮(CAN2 ID1/2, M3508) + 拨弹(CAN2 ID6, M2006)。
- SA 控摩擦轮开关；SB 控拨弹模式。
- 遥控离线 / failsafe / frame_lost 时立即停机。
- USART6 输出关键调试信息（开关状态、模式、参考与反馈）。

## 映射
- `SA` 上：摩擦轮开；下：摩擦轮关
- `SB` 上：拨弹连发（速度环）
- `SB` 中：拨弹停止
- `SB` 下：拨弹单发（角度步进）

## 构建
```bash
cmake -S . -B build
cmake --build build --target test_infantry_shoot_et08_demo -j8
```

## 烧录
```bash
cmake --build build --target flash-test_infantry_shoot_et08_demo
```
