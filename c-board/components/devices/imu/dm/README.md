# DM-IMU 模块封装设计方案（CAN + 串口）

本设计针对 DM-IMU-L1，目标是把 **CAN 总线通信** 与 **串口通信（USB/485）** 封装为统一、可复用的模块，便于在 RM2026 工程中快速接入与维护。

## 1. 依据与结论

参考资料：
- 手册：`/home/myself/workspace/RM2026/temp/dm-imu/达妙科技DM-IMU-L1六轴IMU模块使用说明书V1.2.pdf`
- 官方 CAN 例程：`/home/myself/workspace/RM2026/temp/dm-imu/02.例程/MC02_CAN收发例程`
- 官方 485 例程：`/home/myself/workspace/RM2026/temp/dm-imu/02.例程/MC02_485接收例程`

结论摘要：
- USB/485 主动模式：同一协议，`0x55 0xAA` 帧头 + float 数据 + CRC16 + `0x0A` 结尾
- CAN：支持主动/请求两种模式；主动输出为 **映射值**，请求/应答为 **寄存器读写**
- 配置类指令：多需要“设置模式”执行（进入设置模式后再保存）

## 2. 协议摘要

### 2.1 串口（USB/485 主动模式）
帧格式：
```
55 AA ID TYPE DATA... CRC16(2) 0A
```
- TYPE：01 加速度 / 02 角速度 / 03 欧拉角 / 04 四元数
- DATA：小端序 float
- CRC16：手册附录四算法

### 2.2 串口（485 应答模式，可选）
帧格式：
```
A5 TYPE ID RID RW DATA[0..3] RESP RSVD 5A
```
若后续需要应答式读取，可按手册补齐。

### 2.3 CAN（请求/应答）
请求帧（ID=CAN_ID）：
```
DATA[0]=0xCC DATA[1]=RID DATA[2]=RW DATA[3]=0xDD DATA[4..7]=DATA
```
应答帧（ID=MST_ID）：
```
DATA[0]=0xCC DATA[1]=RID DATA[2]=0xDD DATA[3]=ACK DATA[4..7]=DATA
```

### 2.4 CAN 主动输出映射值
- 加速度/角速度/欧拉角：16-bit 映射值
- 四元数：14-bit 映射值打包
- 映射函数与范围见手册附录二

## 3. 设计目标

- 统一 **CAN/串口** 两种接口的使用体验
- 解析与配置逻辑分离，易于调试
- 可接入 RTOS 或裸机
- 支持主动模式与请求模式

## 4. 模块结构建议

```
components/devices/imu/dm/
  dm_imu.h            // 对外 API
  dm_imu.c            // 状态机与统一逻辑
  dm_imu_uart.c       // 串口接入
  dm_imu_can.c        // CAN 接入
  dm_imu_parser.c     // 协议解析
  dm_imu_crc.c        // CRC16
```

## 5. 核心数据结构

```c
typedef struct {
  float accel[3];
  float gyro[3];
  float euler[3];
  float quat[4];
  float temp;
  uint8_t valid_mask;   // bit: accel/gyro/euler/quat
  uint32_t ts_ms;
} dm_imu_data_t;

typedef enum { DM_IMU_IFACE_USB, DM_IMU_IFACE_RS485, DM_IMU_IFACE_CAN, DM_IMU_IFACE_VOFA } dm_imu_iface_t;
typedef enum { DM_IMU_MODE_NORMAL, DM_IMU_MODE_SETTINGS } dm_imu_mode_t;
```

## 6. 传输层抽象

### 6.1 UART 传输接口
```c
typedef struct {
  bool (*send)(void *ctx, const uint8_t *buf, size_t len);
  uint32_t (*time_ms)(void *ctx);
  void *ctx;
} dm_imu_uart_if_t;
```

### 6.2 CAN 传输接口
```c
typedef struct {
  bool (*send_frame)(void *ctx, uint32_t id, const uint8_t data[8], uint8_t dlc);
  uint32_t (*time_ms)(void *ctx);
  void *ctx;
} dm_imu_can_if_t;
```

## 7. 对外 API 设计

### 7.1 初始化/数据
```c
void dm_imu_init_uart(dm_imu_t *imu, dm_imu_uart_if_t *iface, uint8_t id);
void dm_imu_init_can(dm_imu_t *imu, dm_imu_can_if_t *iface, uint8_t can_id, uint8_t mst_id);
bool dm_imu_get_data(dm_imu_t *imu, dm_imu_data_t *out);
```

### 7.2 RX 接入
```c
void dm_imu_on_uart_rx(dm_imu_t *imu, const uint8_t *buf, size_t len);
void dm_imu_on_can_rx(dm_imu_t *imu, uint32_t id, const uint8_t data[8], uint8_t dlc);
```

### 7.3 配置/控制（UART/CAN 通用）
```c
void dm_imu_enter_settings(dm_imu_t *imu);
void dm_imu_exit_settings(dm_imu_t *imu);
void dm_imu_save_params(dm_imu_t *imu);
void dm_imu_reboot(dm_imu_t *imu);
void dm_imu_angle_zero(dm_imu_t *imu);
void dm_imu_gyro_cal(dm_imu_t *imu);
void dm_imu_accel_cal(dm_imu_t *imu);
void dm_imu_restore_factory(dm_imu_t *imu);
```

### 7.4 输出与接口配置
```c
void dm_imu_set_iface(dm_imu_t *imu, dm_imu_iface_t iface);
void dm_imu_set_output_mask(dm_imu_t *imu, uint8_t mask); // bit0 accel, bit1 gyro, bit2 euler, bit3 quat
void dm_imu_set_interval_ms(dm_imu_t *imu, uint16_t ms);
void dm_imu_set_can_id(dm_imu_t *imu, uint8_t can_id);
void dm_imu_set_mst_id(dm_imu_t *imu, uint8_t mst_id);
void dm_imu_set_temp_control(dm_imu_t *imu, bool enable, uint8_t target);
```

### 7.5 CAN 请求模式（可选）
```c
void dm_imu_request_accel(dm_imu_t *imu);
void dm_imu_request_gyro(dm_imu_t *imu);
void dm_imu_request_euler(dm_imu_t *imu);
void dm_imu_request_quat(dm_imu_t *imu);
```

## 8. 解析设计

### 8.1 UART 解析（USB/485 主动模式）
- 状态机识别帧头 `0x55 0xAA`
- 根据 TYPE 计算帧长（19/23）
- CRC16 校验
- float 小端序解码

### 8.2 CAN 解析
1) 主动模式（DATA[0]=1..4）：
   - Accel/Gyro/Euler：16-bit 映射值 → float
   - Quat：14-bit 打包 → float
2) 请求应答模式（DATA[0]=0xCC）：
   - 根据 RID 解析返回数据

## 9. 配置流程建议

推荐流程：
1. 进入设置模式  
2. 设置接口 + 输出项 + 频率 + ID  
3. 保存参数  
4. 退出设置模式  
5. 必要时重启  

注意：多数配置指令必须在设置模式内执行。

## 10. VOFA（JustFloat）说明（可选）

如果输出接口设为 VOFA：
- 数据以 float 流形式输出
- 帧结束符为 `+inf`（0x7F800000）
- 解析器应以 `+inf` 作为分隔符取一帧

## 11. 与工程集成建议

- 模块位于 `components/devices/imu/dm`
- 提供最小 HAL 适配层（UART/CAN 发送、系统时间）
- 上层只调用 dm_imu_* API，不直接操作协议

---

如果需要，我可以按该设计直接落地代码（C 实现 + 接口文件 + 示例接入）。
