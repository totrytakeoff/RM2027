# 麦轮底盘测试 demo

从当前全向轮 demo 独立复制，目标名 `mecanum_demo.elf`；应用在 `applications/mecanum_demo/`，配置和绑定在 `config/robots/mecanum_demo/`。原 `chassis_demo` 的应用和配置保留。ET08、IMU yaw、静态运行时、安全输出门、USART1 115200 8N1 沿用原框架，麦轮上位机协议使用VOFA+ JustFloat。

## 1. 布局和机械约定

俯视，上方为车头：

```text
      前
   FL4   FR3
   BL1   BR2
```

CAN1，C620/M3508，指令组0x200。采用标准 X 型麦轮安装；轮端速度向前为正，默认右侧电机镜像安装（FR/BR reference_sign=-1，FL/BL=+1）。实际轴向和安装不同只修改对应 binding/config，不交换轮序来补偿方向。

默认轮半径0.075m，前后轮中心距L=0.34m，左右中心距W=0.34m，减速比19.20320856；机械尺寸尚未收到另行确认。轮速顺序为 FR/FL/BR/BL，x向右、y向前、wz逆时针为正：

```text
k = (L + W)/2
FR = (vy - vx + k*wz)/r
FL = (vy + vx - k*wz)/r
BR = (vy + vx + k*wz)/r
BL = (vy - vx - k*wz)/r
```

再乘减速比，最后通过电机 reference_sign 转成原始转子速度。heading 坐标变换按这个坐标约定独立实现，没有直接沿用全向轮的反向平移补丁。ET08 默认左X=-1、左Y=-1（前推负原始值映射为正向前进）；需要按实际发射机通道方向确认。右摇杆、SA使能、SB跟随/小陀螺规则见 [原 demo 操作](chassis_demo.md)。当前复制的 yaw 参数为 Kp20/Ki0/Kd1，右摇杆目标360 deg/s，跟随上限10 rad/s；不是早期文档里的旧调参值。

## 2. 只输出功率估算，不限制驱动

按用户要求，已删除原50W预算、总电流限幅、反馈回退、PID积分干预和限流hook。电机指令由原速度/电流PID直接生成，保留框架原有输出范围、安全门与失联保护。通过遥控手动控制速度。

诊断任务每10ms读取当前有效电机反馈，按200ms窗口求平均后输出；每个采样先计算：

```text
I_sum = sum(abs(current_raw)) / 819.2
P_est = 24 * I_sum
```

电压和换算系数在 `mecanum_config.h`。这是根据转矩电流计算的估算值，不是DC电源实测功率。计算只在诊断任务进行，不改轮速参考、PID或CAN指令。

## 3. 构建和烧录

```sh
./scripts/build.sh build --target mecanum_demo
./scripts/upload.sh mecanum_demo
# 或一条命令构建并烧录
./scripts/build.sh upload --target mecanum_demo
```

产物 `build/h7-debug/mecanum_demo.{elf,bin,hex,map}`。优化版使用 `--preset h7-release`。烧录入口是原生Bash/PowerShell、默认BIN和DAP v2，不依赖Python。上述命令用于烧录本次无功率限幅版本；本次已用DAP v2完成烧录/校验并复位。

仅观察（电机任务只发零指令）：

```sh
./scripts/build.sh build --preset h7-mecanum-observe
./scripts/upload.sh --preset h7-mecanum-observe
```

观察版不能拿来证明实际驱动触限。SBUS preset 才生成麦轮固件，DBUS不生成。烧录前SA下档；首次架起车轮确认四个ID、正反向和IMU yaw极性，再落地低速试车。

## 4. VOFA+ JustFloat

USART1 PA9 TX / PA10 RX，115200 8N1，DAP交叉接线并共地。VOFA+选择串口连接，协议引擎 **JustFloat**，打开波形图并绑定所需通道。

每200ms发送一帧窗口均值（约5Hz），每窗通常20次10ms采样，首次窗口含边界样本可能为21次，7个小端IEEE754 float32加帧尾 `00 00 80 7F`，总长32 bytes。发送使用平台复制式DMA队列，繁忙时丢弃本次采样，不阻塞控制。数据格式遵循 [VOFA+官方JustFloat说明](https://www.vofa.plus/docs/learning/dataengines/justfloat/)。

| 通道（从0开始） | 内容 | 单位 |
| --- | --- | --- |
| CH0 | 窗口平均估算功率 `P_est` | W |
| CH1 | 四轮绝对电流和的窗口平均 | A |
| CH2 | ID1 后左电流均值，有符号 | A |
| CH3 | ID2 后右电流均值，有符号 | A |
| CH4 | ID3 前右电流均值，有符号 | A |
| CH5 | ID4 前左电流均值，有符号 | A |
| CH6 | 反馈在线掩码，bit0..3对应ID1..4 | 15表示全在线 |

功率/总电流先对每次采样取绝对值再平均；各轮电流通道保留正负并求平均。在线掩码变化时丢弃旧窗口重新积累，避免混入失效前数据。无效轮通道置0，CH6指示哪些轮的反馈有效；部分离线时CH0/CH1仅统计有效轮。CH0仍是24V×转矩电流绝对值之和的估算，不是电源实测功率。

USART1不再输出RM/MC/RC/YAW/BODY/PWR/M等文本，也不再有LIM行；框架文本诊断转到USART10。普通文本串口助手看到二进制乱码属于预期，请使用VOFA+ JustFloat。串口接收仍不执行控制命令。

## 5. 当前版本与历史记录

当前200ms窗口平均版的Debug、Release、观察目标构建通过，已用DAP v2烧录、校验并复位。串口被占用，未再次抓取验证实际输出频率，未中断现有串口程序。未添加/运行测试。

- [平均版烧录记录](../archive/hardware/2026-10-17/mecanum_justfloat_average/flash.json)
- [平均版构建快照](../archive/records/2026-10-17_mecanum_justfloat_average.json)
- [原50Hz瞬时JustFloat记录](../archive/hardware/2026-10-17/mecanum_justfloat/flash.json)
- [旧文本版记录](../archive/hardware/2026-10-17/mecanum_telemetry_only/flash.json)
- [早期限幅版问题](../archive/hardware/2026-10-17/mecanum_investigation/analysis.md)

旧版采样周期、文本字段与限流数据仅为历史证据。当前版本没有功率限幅，平均处理只影响显示。
