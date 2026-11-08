# 文档目录

日常使用从指南开始；当前完成情况统一看验收状态。早期阶段记录已归档，其“尚未完成”描述只代表当时状态。

## 分类

| 目录 | 内容 | 入口 |
| --- | --- | --- |
| `guides/` | 当前构建、诊断、底盘接线与操作 | [全向轮指南](guides/chassis_demo.md)、[麦轮指南](guides/mecanum_demo.md)、[构建诊断](guides/build_and_bringup.md)、[电机测试](guides/motor_test.md)、[DM4310](guides/motor_test_dm4310.md) |
| `architecture/` | 分层、接口契约、移植目标、硬件与内存基线 | [移植方案](architecture/porting_plan.md)、[硬件基线](architecture/hardware_baseline.md) |
| `reference/` | 参考工程分析、来源与导入清单 | [来源记录](reference/provenance.md)、[资料分析](reference/reference_analysis.md)、[导入清单](reference/import_manifest.json) |
| `validation/` | 当前阶段结论、已验证范围和剩余工作 | [验收状态](validation/README.md) |
| `archive/` | 阶段报告、审计过程、构建快照、原始硬件证据 | [归档索引](archive/README.md) |

脚本说明与脚本放在一起：[scripts/README.md](../scripts/README.md)。

## 阅读顺序

1. 使用或调试底盘：底盘指南 → 脚本说明 → 构建诊断。
2. 继续开发框架：验收状态 → 架构方案 → 硬件基线。
3. 追溯问题：归档索引 → 对应日期和固件 hash 的记录。

## 维护规则

- 操作、接线和当前配置说明更新到 `guides/`；具体参数以配置源码为准。
- 当前阶段的完成情况只在 `validation/README.md` 汇总。
- 阶段过程结束后按日期归档；历史结论不覆盖当前状态。
- 硬件证据保留原始字节和固件 hash。记录中的旧输出路径表示当时执行环境，不要求与归档路径一致。
- 新硬件记录放入 `archive/hardware/<日期>/`，在验收状态中引用。
- 构建/来源快照按实际生成时点维护，不因整理文档重新计算历史源码 hash。
