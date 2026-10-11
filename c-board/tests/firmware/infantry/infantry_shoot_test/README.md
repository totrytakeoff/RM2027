# 步兵云台 + 发射机构联测（ET08）

文档更新日期：2026-07-19

## 项目简介

本测试固件仿照 `tests/firmware/hero/hero_shoot_test`，将步兵云台（yaw/pitch）与发射机构（摩擦轮/拨弹）集成在同一工程中，使用 ET08（SBUS）遥控联调。

## 硬件配置（按你的需求）

- yaw 轴 GM6020：CAN1 ID 1
- pitch 轴 GM6020：CAN2 ID 1
- 摩擦轮：CAN2 ID 1 / 2
- 拨弹：CAN2 ID 6（M2006，减速比按 1:13）

## 遥控映射（默认实现，可按现场改宏）

- `CH5(开关组 SA/SB)`：摩擦轮使能（state 0~2 视为开启，3~5 关闭）
- `CH6(开关组 SD/SC)`：发射模式（state 0 单发，1 双连发，2 连发，其它按单发处理）
- `KnobRight`：拨弹允许（向负方向拧过阈值允许，回到阈值以上停止）
- 右摇杆左右：yaw 轴速度
- 右摇杆上下：pitch 轴速度（上推抬头，默认做了减半缩放）

> ET08 的开关/旋钮映射可能因你的模型设置不同而变化：如果不符合预期，直接在 `tests/firmware/infantry/infantry_shoot_test/main.c` 顶部宏区修改映射/阈值即可。

## 构建和烧录

### 构建
```bash
cmake -S . -B build -G "Ninja" -DCMAKE_BUILD_TYPE=Debug
ninja -C build test_infantry_shoot
```

### 烧录（OpenOCD）
```bash
ninja -C build flash-test_infantry_shoot
```
