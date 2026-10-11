# Referee 2026 TODOLists

本文档用于管理 `components/devices/referee` 对齐 2026 协议的剩余工作项。
状态字段建议：`TODO / DOING / DONE / BLOCKED`。

更新时间：2026-07-23

---

## P0（必须优先）

### [TODO] 1. 客户端 ID 映射改为 2026 固定表

目标：

1. 不再使用 `0x0100 + robot_id` 线性推导 `Cilent_ID`。
2. 按 2026 文档附录的客户端 ID 固定映射实现（红蓝分开）。

建议修改位置：

1. `components/devices/referee/referee_task.c` 的 `DeterminRobotID()`
2. 可新增 `static uint16_t RefereeMapClientId(uint8_t robot_id)` 辅助函数

验收标准：

1. 红方 1/2/3/4/5/6 对应 `0x0101~0x0106`，蓝方对应 `0x0165~0x016A`。
2. 蓝方 UI 可稳定显示，交互目标无错发。

---

### [TODO] 2. 控制环接入裁判限额闭环

目标：

1. 将 `PowerHeatData`、`GameRobotState`、`ProjectileAllowance` 等接入底盘/发射控制策略。
2. 最少实现“超限降额”保护，不仅仅解析数据。

建议修改位置：

1. `applications/robot/chassis/chassis.c`（功率/缓冲能量限幅）
2. `applications/robot/shoot/shoot.c`（发弹节流/热量保护）

验收标准：

1. 实测在限制工况下，输出会按策略自动收敛。
2. 不依赖人工急停避免超限。

---

## P1（建议尽快）

### [TODO] 3. 完整落地 `0x020A~0x020E`

目标：

1. 在 `referee_protocol.h` 增加对应 payload 结构体。
2. 在 `referee_info_t` 增加字段。
3. 在 `rm_referee.c` switch 中完成解析写入。

建议修改位置：

1. `components/devices/referee/referee_protocol.h`
2. `components/devices/referee/rm_referee.h`
3. `components/devices/referee/rm_referee.c`

验收标准：

1. `0x020A~0x020E` 命令字均可解析到结构体。
2. 提供最小日志或调试接口验证字段变化。

---

### [TODO] 4. 统一 Robot ID 枚举口径

目标：

1. `Robot_ID_e` 枚举与 2026 协议一致，避免历史 101~109 混用。

建议修改位置：

1. `components/devices/referee/referee_protocol.h`
2. 相关调用点全局检索核对

验收标准：

1. 无旧版蓝方 ID 常量残留（除历史注释）。
2. 编译通过且 UI/交互无回归。

---

### [TODO] 5. 发送节流策略参数化

目标：

1. 将 `RefereeSend()` 中固定 `osDelay(115)` 改为可配置节流机制。
2. 区分 UI 慢速刷新与交互数据快速发送。

建议修改位置：

1. `components/devices/referee/rm_referee.c`
2. 可配宏放到 `applications/robot/config/robot_config.h` 或 referee 配置头

验收标准：

1. 不同消息类型可配置不同节流。
2. 不触发裁判系统带宽/频率违规。

---

## P2（维护优化）

### [TODO] 6. 协议注释与文档全量同步

目标：

1. 修正结构体注释中的历史字节数和旧术语。
2. 保证注释与真实字段一一对应。

验收标准：

1. 无明显“注释字节数 != 实际结构体大小”的条目。
2. 新成员可仅依靠代码注释理解模块。

---

### [TODO] 7. 回归测试与抓包样例沉淀

目标：

1. 沉淀一份裁判串口抓包样例与字段对照。
2. 添加最小离线回放测试（CRC + cmd + length）。

建议位置：

1. `tests/host/` 下新增 referee 协议回放测试
2. `docs/referee/` 增加抓包说明

验收标准：

1. 关键 cmd 的回放解析结果可自动比对。
2. 升级协议时可快速回归。

---

## 建议执行顺序

1. `P0-1` 客户端 ID 映射
2. `P0-2` 控制闭环
3. `P1-3` 020A~020E 落地
4. `P1-4` ID 枚举统一
5. `P1-5` 发送节流参数化
6. `P2` 文档与测试完善
