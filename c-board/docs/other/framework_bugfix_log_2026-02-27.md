# 框架问题修复记录（2026-02-27）

文档更新日期：2026-07-23

> 标题日期为首批问题修复日期；本文在仓库结构收口时同步更新了现行路径。

## 1. `message_center` 队列索引更新错误

## 问题描述

`SubGetMessage()` 中使用了：

```c
sub->front_idx = (sub->front_idx++) % QUEUE_SIZE;
```

该写法对同一变量在一个表达式中重复读写，属于未定义行为，可能导致索引不按预期推进，进而出现消息读取异常。

## 修复

改为确定性写法：

```c
sub->front_idx = (sub->front_idx + 1) % QUEUE_SIZE;
```

## 影响范围

1. 订阅者队列出队行为稳定，索引推进可预期。
2. 避免编译器优化差异导致的偶发问题。

## 修改文件

1. `components/services/message_bus/message_center.c`

---

## 2. `daemon` 初始化计数被覆盖

## 问题描述

`DaemonRegister()` 中先根据 `init_count` 赋值 `temp_count`，随后又被：

```c
instance->temp_count = config->reload_count;
```

直接覆盖，导致“初始宽限计数”逻辑失效。

## 修复

删除覆盖语句，保留 `init_count` 语义：

1. 若 `init_count != 0`，按 `init_count` 初始化；
2. 否则使用默认初始值（当前实现为 100）。

## 影响范围

1. 模块上电初始宽限逻辑恢复有效；
2. 避免部分模块在启动窗口被误判离线。

## 修改文件

1. `components/services/device_health/daemon.c`

---

## 3. 建议后续补充

1. 为 `message_center` 增加最小回归测试（入队/出队/覆盖旧消息）。
2. 为 `daemon` 增加用例：`init_count` 与 `reload_count` 的边界行为验证。
3. 在文档中明确 `init_count` 和 `reload_count` 的设计意图，避免后续误改。

---

## 4. `shoot` 任务编译错误（迁移阶段）

## 问题描述

`ShootTask()` 中新增斜坡限速逻辑后，`loader_last_update_ms = now;` 出现在 `float now = DWT_GetTimeline_ms();` 定义之前，导致编译报错：

```c
error: 'now' undeclared (first use in this function)
```

## 修复

将 `float now = DWT_GetTimeline_ms();` 提前到 `ShootTask()` 前部，在首次使用前完成定义。

## 影响范围

1. 恢复 `app.elf` 正常构建。
2. 保持连发斜坡限速逻辑功能不变，仅修复变量作用域/时序错误。

## 修改文件

1. `applications/robot/shoot/shoot.c`
