# Infantry Shoot ET08 Demo 日志字段文档

## 概述
- 调试串口: USART6 (115200bps)
- Text日志周期: 100ms
- VOFA帧周期: 20ms (JustFloat协议)

---

## Text日志

### 日志1: 状态信息
```
[sht_et08] #seq online= fs= lost= sa_sb(raw= st= sb=) friction= loader_en= cont= mode= single= pending=
```

| 字段 | 含义 | 取值 |
|------|------|------|
| seq | 序号 | 递增计数 |
| online | 遥控器在线 | 0/1 |
| fs | failsafe状态 | 0/1 |
| lost | 丢帧状态 | 0/1 |
| sa_sb(raw) | 开关原始值 | 0-5 |
| sa_sb(st) | 开关状态值 | 0-5 |
| sb | SB逻辑位置 | 0=上,1=中,2=下 |
| friction | 摩擦轮使能 | 0/1 |
| loader_en | 拨弹使能 | 0/1 |
| cont | 连发使能 | 0/1 |
| mode | 拨盘模式 | 0=停止, 1=单发, 2=连发 |
| single | 单发进行中 | 0/1 |
| pending | 待发射弹数 | 0/1 |

### 日志2: 电机信息
```
[sht_et08] friction(l= r=) loader(ref= speed= angle=)
```

| 字段 | 含义 | 单位 |
|------|------|------|
| friction(l) | 左摩擦轮速度 | rpm |
| friction(r) | 右摩擦轮速度 | rpm |
| loader(ref) | 拨盘参考值 | rpm/deg |
| loader(speed) | 拨盘实际速度 | rpm |
| loader(angle) | 拨盘累计角度 | deg |

### 日志3: 拨弹过程信息
```
[sht_et08] loader(single_start= target= delta= err= speed_cmd= elapsed=)
```

| 字段 | 含义 | 单位 |
|------|------|------|
| single_start | 单发起始角 | deg |
| target | 单发目标角 | deg |
| delta | 本次单发已走过角度 | deg |
| err | 单发角度误差 | deg |
| speed_cmd | 连发速度斜坡命令 | deg/s |
| elapsed | 单发已运行时间 | ms |

---

## VOFA帧 (JustFloat协议)

### 帧格式
- 32通道 float (128字节) + 4字节尾帧 `00 00 80 7F`
- 周期: 20ms (50Hz)

### 通道定义

| 索引 | 字段名 | 含义 | 单位 | 备注 |
|------|--------|------|------|------|
| 0 | tick | 系统时间 | ms | HAL_GetTick() |
| 1 | online | 遥控在线 | - | ET08_IsOnline() |
| 2 | friction_en | 摩擦轮使能 | - | SA开关控制 |
| 3 | loader_en | 拨弹使能 | - | friction开启且SB非中位 |
| 4 | loader_continuous | 连发使能 | - | SB上 |
| 5 | loader_mode | 拨盘模式 | - | 0停止/1单发/2连发 |
| 6 | single_active | 单发进行中 | - | 状态机标志 |
| 7 | pending_shots | 待发射弹数 | 发 | 单发队列 |
| 8 | sa_sb_raw | 开关原始值 | - | ET08原始 |
| 9 | sa_sb_state | 开关状态 | - | 0~5 |
| 10 | sb_pos | SB逻辑位置 | - | 0上/1中/2下 |
| 11 | loader_ref | 当前拨弹参考 | deg/s或deg | 观测控制输出 |
| 12 | loader_speed_cmd | 连发速度命令 | deg/s | 斜坡后速度 |
| 13 | loader_target | 单发目标角度 | deg | 单发步进目标 |
| 14 | loader_angle | 拨盘实际角度 | deg | total_angle |
| 15 | loader_angle_err | 单发角度误差 | deg | target-angle |
| 16 | single_start_angle | 单发起始角 | deg | 单发开始采样 |
| 17 | single_delta | 单发已走过角度 | deg | abs(angle-start) |
| 18 | single_elapsed_ms | 单发已运行时间 | ms | timeout观察 |
| 19 | loader_speed_fdb | 拨弹速度反馈 | deg/s | speed_aps |
| 20 | loader_curr | 拨弹电流 | mA | M2006 |
| 21 | friction_l_speed | 左摩擦轮速度 | deg/s | M3508 |
| 22 | friction_r_speed | 右摩擦轮速度 | deg/s | M3508 |
| 23 | friction_target | 摩擦轮目标 | deg/s | 常量 |
| 24 | loader_conti_speed | 连发目标速度 | deg/s | 常量 |
| 25 | loader_single_speed | 单发拨弹速度 | deg/s | 常量 |
| 26 | loader_angle_step | 单发步长 | deg | 常量 |
| 27 | loader_speed_kp | 拨弹速度环Kp | - | 运行时PID参数 |
| 28 | loader_speed_kd | 拨弹速度环Kd | - | 运行时PID参数 |
| 29 | loader_angle_kp | 拨弹角度环Kp | - | 运行时PID参数 |
| 30 | loader_angle_kd | 拨弹角度环Kd | - | 运行时PID参数 |
| 31 | shoot_en | 发射使能 | - | 电机已使能 |

---

## 遥控开关映射

### ET08 SBUS (USART3)

| 开关 | 位置 | 值 | 功能 |
|------|------|-----|------|
| SA | UP | 0-2 | 摩擦轮开启 |
| SA | MID/DOWN | 3-5 | 摩擦轮关闭 |
| SB | UP | 0-2 | 连发模式 |
| SB | MID | 3 | 停止 |
| SB | DOWN | 4-5 | 单发模式 |

---

## 发射参数 (来自minimal_config.h)

| 参数 | 值 | 说明 |
|------|-----|------|
| FRICTION_TARGET_SPEED | -30000 deg/s | 摩擦轮目标转速 |
| LOADER_CONTINUOUS_SPEED | 20000 deg/s | 连发拨盘速度 |
| LOADER_SINGLE_SPEED | 7000 deg/s | 单发拨弹速度 |
| LOADER_ANGLE_STEP | 585 deg | 单发角度步进 |
| LOADER_SINGLE_TIMEOUT_MS | 350 ms | 单发超时 |
| SHOOT_INTERVAL_MS | 2000 ms | 单发间隔 |

---

## 建议重点观察

### 连发手感
- `loader_continuous`
- `loader_speed_cmd`
- `loader_speed_fdb`
- `loader_curr`

期望：
- 切到连发后 `loader_speed_cmd` 平滑爬升
- `loader_speed_fdb` 平稳跟随 `loader_speed_cmd`
- `loader_curr` 不长期顶满

### 单发手感
- `single_active`
- `loader_target`
- `loader_angle`
- `loader_angle_err`
- `single_delta`
- `single_elapsed_ms`

期望：
- 触发单发后 `single_active` 拉高
- `single_delta` 接近 `loader_angle_step`
- `loader_angle_err` 收敛，不明显来回振荡
- `single_elapsed_ms` 明显小于 `LOADER_SINGLE_TIMEOUT_MS`

---

## 电机配置

| 电机 | CAN | ID | 类型 |
|------|-----|-----|------|
| 左摩擦轮 | CAN2 | 1 | M3508 |
| 右摩擦轮 | CAN2 | 2 | M3508 |
| 拨盘 | CAN2 | 6 | M2006 |
