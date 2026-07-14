# CLAUDE.md

## 项目概述

ESP32S3 双臂复合机器人主控系统 — 基于 PlatformIO + Arduino + FreeRTOS。

**硬件平台**：ESP32S3 (freenove_esp32_s3_wroom)
**用户角色**：仅负责 ESP32S3 端代码，不涉及 Jetson/STM32/地面站代码

### 系统拓扑

```
地面站(无线串口) ─┐
                   ├→ ESP32S3 ──→ STM32F103 底盘 (麦克纳姆轮)
Jetson Nano ──────┘      │
                          └──→ 众灵总线舵机控制板 ×6 (双机械臂, 肩/肘/夹具×2)
```

### 通信方式

| 通道 | 串口 | 方向 | 协议 |
|------|------|------|------|
| 调试日志 | USB CDC (`Serial`) | 输出 | 文本 |
| 地面站/LORA | `Serial2` (RX=GPIO18, TX=GPIO17) | 双向 | 自定义二进制 |
| 舵机控制板 | `Serial1` (RX=GPIO11, TX=GPIO12) | 输出为主 | 众灵ASCII字符串 |
| Jetson | `Serial0` RX (GPIO1) | 输入 | 自定义二进制 |
| STM32底盘 | `Serial0` TX (GPIO2) | 输出 | 自定义二进制 |

**注意**：ESP32S3 只有 UART0/1/2 三路硬件 UART。当前部署优先保证地面站无线 TTL 串口和舵机板独立通信，调试日志使用 USB CDC 并通过 `platformio.ini` 启用 `ARDUINO_USB_CDC_ON_BOOT=1`，释放 UART0 给 Jetson RX 与 STM32 TX 单向复用。

## 核心架构

### FreeRTOS 任务分配

| 任务 | 函数 | 核 | 优先级 | 功能 |
|------|------|---|--------|------|
| GROUND_RX | `taskGroundStation` | Core 1 | 3 | 地面站串口接收+状态机解析 |
| JETSON_RX | `taskJetson` | Core 0 | 3 | Jetson串口接收+状态机解析 |
| CMD_ROUTER | `taskCommandRouter` | Core 1 | 3 | 命令分类路由 |
| ARM | `taskArm` | Core 1 | 4 (最高) | 机械臂动作组执行 |
| STATUS | `taskStatus` | Core 1 | 1 (最低) | 心跳500ms + 状态上报1000ms |

`loop()` 仅 `vTaskDelay(1000)`，所有逻辑在FreeRTOS任务中。

### 通信协议 (自定义二进制)

```
AA 55 | Ver | Src | Dst | Type | Func | Seq | Len | Data[0~64] | CRC16(LSB) | 0D 0A
```

- 帧头 `0xAA 0x55`，帧尾 `0x0D 0x0A`
- CRC-16/MODBUS (256项查表)
- 消息类型: CMD=0x01, ACK=0x02, NACK=0x03, DATA=0x04, HEARTBEAT=0x05
- 序列号回显: ACK/NACK必须回显原命令seqNum

### 设备地址

| 地址 | 设备 |
|------|------|
| 0x10 | BROADCAST (系统协议广播，非舵机广播ID) |
| 0x20 | ESP32S3 (本机) |
| 0x30 | Jetson Nano |
| 0x40 | STM32F103 底盘 |
| 0x50 | GROUND 地面站 |
| 0x60 | SERVO 舵机控制板 (逻辑地址) |

### 功能码分类

- **系统类 0x00~0x0F**: 心跳、ACK/NACK、状态查询/上报、急停
- **底盘类 0x10~0x1F**: 前进/后退/左移/右移/左转/右转/停止/矢量移动
- **机械臂 0x20~0x2F**: HOME/夹取后上抬/放下/停止/扭力控制
- **视觉类 0x30~0x3F**: OBJECT_INFO (Jetson→ESP32)

### ESP32 路由逻辑

- 底盘命令 → 转发至STM32 (修改src=ESP32, dst=STM32，其余不变)
- 机械臂命令 → 本地执行 (armCommandQueue)
- 物体识别信息 → 更新RobotState + 转发地面站
- 系统命令 → 本地处理 (心跳ACK、状态查询回复等)

## 文件结构

```
ESP_planner/
├── platformio.ini                    # board=freenove_esp32_s3_wroom, 无第三方依赖
├── include/
│   ├── config/
│   │   ├── HardwareConfig.h          # 串口引脚、波特率、GPIO
│   │   ├── DeviceConfig.h            # 设备地址、协议版本
│   │   ├── ArmConfig.h               # 舵机ID(000~005)、PWM范围(0500~2500)、动作组结构体
│   │   └── TaskConfig.h              # 任务栈(3072~4096)、优先级(1~4)、队列长度
│   ├── protocol/
│   │   ├── Protocol.h                # 帧结构、功能码枚举、协议函数声明
│   │   ├── ProtocolParser.h          # 状态机解析器 (13个状态, 每通道独立实例)
│   │   └── Crc16.h                   # CRC-16/MODBUS
│   ├── servo/BusServoDriver.h        # 众灵舵机驱动 (全静态方法)
│   ├── arm/
│   │   ├── ArmActionGroups.h         # ArmAction枚举, 预设动作组声明
│   │   └── ArmController.h           # 动作组执行器
│   ├── comm/
│   │   ├── SerialManager.h           # 串口管理 (Mutex保护发送, sendFrame路由)
│   │   └── CommandRouter.h           # 命令路由任务
│   └── app/RobotState.h              # 线程安全状态存储 (spinlock)
├── src/                              # 与 include/ 结构一一对应
│   ├── main.cpp                      # 入口, setup(), 全局队列, FreeRTOS任务
│   ├── protocol/{Protocol,ProtocolParser,Crc16}.cpp
│   ├── servo/BusServoDriver.cpp
│   ├── arm/{ArmActionGroups,ArmController}.cpp
│   ├── comm/{SerialManager,CommandRouter}.cpp
│   └── app/RobotState.cpp
└── docs/
    └── 通信协议规范.md                # 922行完整协议文档 (14章+2附录)
```

## 关键技术细节

### 协议解析器状态机 (ProtocolParser)
13个状态: WAIT_HEAD1 → WAIT_HEAD2 → READ_VERSION → READ_SRC → READ_DST → READ_MSG_TYPE → READ_FUNC → READ_SEQ → READ_LEN → READ_PAYLOAD → READ_CRC_LOW → READ_CRC_HIGH → WAIT_TAIL1 → WAIT_TAIL2

每个串口通道独立实例 (`groundParser`, `jetsonParser`)，互不干扰。CRC校验使用MODBUS特性 (完整帧的CRC结果为0)。

### 舵机控制 (众灵协议)
- 单舵机: `#000P1500T1000!` (ID=000, 位置=1500, 时间=1000ms)
- 多舵机: `{#000P1500T1000!#001P1600T1000!}` (花括号包裹)
- 舵机ID 000~002=左臂(肩/肘/夹具), 003~005=右臂(肩/肘/夹具)
- 发送方式已按 `../Servo_test` 实测工程对齐：`ServoSerial.print(cmd); ServoSerial.println(); ServoSerial.flush();`
- 舵机串口必须使用 `Serial1.setPins(11, 12)` 这一组合；物理引脚仍为 RX=GPIO11、TX=GPIO12，不需要改线。
- 动作组中的 `0000` 不是有效舵机位置，而是 `SERVO_PWM_UNUSED` 占位值；发送组合命令时会过滤，不会生成 `P0000`。

### 动作组 (ArmActionGroups)
11个预设动作仍保留: HOME / LEFT_PICK / LEFT_PLACE / RIGHT_PICK / RIGHT_PLACE / BOTH_PICK / BOTH_PLACE / STOP_ALL / EMERGENCY_STOP / TORQUE_OFF_ALL / TORQUE_ON_ALL

当前机械臂已更新为 6 舵机：
- 000 / 003 = 肩关节
- 001 / 004 = 肘关节
- 002 / 005 = 夹具

动作组表来自 `动作组.md`，当前写入值为：
- 准备: `1233 0951 0000 1818 0890 0000`
- 夹取: `0541 0951 0000 2377 0890 0000`
- 上抬: `0541 0513 0000 2377 1474 0000`
- 放下: `0870 0513 0000 1892 1474 0000`

地面站逻辑为：
- 点击“夹取”时，ESP32 依次执行“夹取”->“上抬”
- 点击“放下”时，ESP32 执行“放下”->“准备”
- 其余时间默认保持“准备”动作

动作组在 `src/arm/ArmActionGroups.cpp` 中修改。

### setPins() 参数顺序
Arduino-ESP32 `HardwareSerial::setPins(rxPin, txPin)` 当前按 RX 在前、TX 在后使用。当前地面站无线串口为 `Serial2.setPins(18, 17)`，表示 RX=GPIO18、TX=GPIO17；舵机板为 `Serial1.setPins(11, 12)`，表示 RX=GPIO11、TX=GPIO12。

### 舵机通信实测基准
`../Servo_test` 工程已现场验证可稳定驱动所有已连接舵机。主工程舵机通道必须与该工程保持一致：
- 物理引脚：RX=GPIO11、TX=GPIO12
- UART 对象：`Serial1`
- 波特率：115200, `SERIAL_8N1`
- 发送尾部：ASCII 命令后调用空 `println()`，即追加 CRLF

不要为排查舵机问题随意改动 `PIN_SERVO_RX` / `PIN_SERVO_TX` 的物理引脚号；如需调整，只能先确认现场接线同步修改。

### 地面站接收 LED 验证
当前板载语音/LED 外设使用 GPIO10，舵机串口已避开 GPIO10，物理引脚为 RX=GPIO11、TX=GPIO12。`PIN_GROUND_RX_LED = GPIO10` 用于现场最小验证：任意字节触发阶段已确认地面站物理接收链路可让 LED 闪烁；当前正式逻辑为只有 `groundParser.inputByte(byteIn, frame)` 解析出完整有效协议帧后才短闪，表示地面站到 ESP32 的物理层与协议层均已通过。

### 编译验证
- 编译通过: RAM 9.7% (31856/327680), Flash 10.1% (336485/3342336)
- 无第三方依赖，仅需 platform = espressif32 + framework = arduino

## 项目状态

- [x] 完整工程代码 (编译通过)
- [x] 通信协议文档 (docs/通信协议规范.md)
- [ ] 舵机最终机械标定与安全范围复核
- [ ] 硬件接线验证
- [ ] 系统联调

## 重要提醒

1. `动作组.md` 中的 PWM 已写入代码，但修改 `ArmActionGroups.cpp` 前仍需先在实体舵机上测试安全范围
2. 串口接线时注意TX/RX交叉、共地、波特率一致
3. ESP32S3 硬件 UART 有限，当前 ServoSerial 使用 Serial1 + GPIO11/12（对齐 Servo_test 实测工程），GroundSerial/LoraSerial 使用 Serial2 + GPIO18/17，JetsonSerial 与 ChassisSerial 共用 Serial0 的 RX/TX 两个方向
4. DebugSerial 独立使用 USB CDC；不要让地面站协议复用 USB 调试口
5. `BROADCAST_ADDR = 0x10` 是系统协议广播，众灵舵机自身广播ID=255，两者不同
6. 不在 `loop()` 中放任何业务逻辑，不用阻塞式 `readString()`
7. 串口发送全部经过 `SerialManager` (Mutex保护)，禁止业务代码直接操作串口
