# 历史归档

本目录保留过程与证据。当前完成情况见 [验收状态](../validation/README.md)，使用方法见 [指南](../README.md)。

## 阶段记录

| 日期 | 记录 |
| --- | --- |
| 2026-10-07 | [第一轮平台实现](milestones/2026-10-07_platform.md) |
| 2026-10-07 | [第二轮组件迁移](milestones/2026-10-07_components.md) |
| 2026-10-08 | [七项审计修复](milestones/2026-10-08_audit_fixes.md) |
| 2026-10-11 | [板上验证和底盘联调过程](milestones/2026-10-11_hardware_log.md) |

## 构建与脚本快照

- [构建/源码快照](records/build_manifest_2026-10-11.json)：保留当时 artifact/source hash，整理后不重新认定为当前版本。
- [脚本迁移快照](records/script_migration_2026-10-11.json)：包含原脚本来源、适配记录与原生烧录入口修订。

## 2026-10-11 硬件证据

| 目录 | 内容 |
| --- | --- |
| [backup](hardware/2026-10-11/backup/) | 首次烧录前完整 Flash 备份与备份记录 |
| [bringup_verified](hardware/2026-10-11/bringup_verified/) | 具有固件 hash 的基础运行与故障确认记录，原 `final/` |
| [bringup_experiments](hardware/2026-10-11/bringup_experiments/) | 初始观测、故障脚本调试、中间结果和失败尝试 |
| [host](hardware/2026-10-11/host/) | 底盘早期主机测试结果 |
| [chassis](hardware/2026-10-11/chassis/) | 底盘观察、烧录、轮序/yaw 调整与前后修订记录 |

底盘记录保留原子目录名：`chassis_observe`、`chassis_flash`、`chassis_tuning`、`chassis_forward_fix`。同一目录也可能包含失败尝试，请按文件中的 verify 标志、退出码和固件 hash 判断。

所有 68 份原始硬件文件按原字节归档，完整 Flash 备份保留。JSON/GDB/日志中的路径是实验执行时的路径，迁移后不改写这些原始证据。
