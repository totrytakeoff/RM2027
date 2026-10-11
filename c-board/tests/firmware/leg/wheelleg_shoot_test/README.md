# 轮腿云台 + 发射机构联测（ET08）

文档更新日期：2026-07-19

## 项目简介

本测试固件仿照 `tests/firmware/infantry/infantry_shoot_test` 的实现：ET08（SBUS）遥控控制云台 yaw/pitch 与发射机构（摩擦轮/拨弹），用于轮腿整机联调。

## 硬件配置（按你的需求）

- 摩擦轮：CAN2 ID 1 / 2（M3508）
- pitch 轴 GM6020：CAN2 ID 1
- yaw 轴 GM6020：CAN1 ID 5
- 拨弹 M2006：CAN1 ID 5

## 遥控映射（与步兵一致，可按现场改宏）

- `CH5(开关组 SA/SB)`：摩擦轮使能（state 0~2 开启，3~5 关闭）
- `CH6(开关组 SD/SC)`：发射模式（0 单发，1 双连发，2 连发）
- `KnobRight`：拨弹允许（向负方向拧过阈值允许，回到阈值以上停止）
- 右摇杆左右：yaw 轴速度
- 右摇杆上下：pitch 轴速度

## 构建和烧录

### 构建
```bash
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
ninja -C build test_wheelleg_shoot
```

### 烧录（OpenOCD）
```bash
ninja -C build flash-test_wheelleg_shoot
```
