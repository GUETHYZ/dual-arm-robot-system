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
                          └──→ 众灵总线舵机控制板 ×8 (双机械臂, 3DOF×2)
```

### 通信方式

| 通道 | 串口 | 方向 | 协议 |
|------|------|------|------|
| 调试日志 | USB CDC (Serial) | 输出 | 文本 |
| 地面站 | Serial (复用USB CDC) | 双向 | 自定义二进制 |
| Jetson | Serial1 | 输入 | 自定义二进制 |
| STM32底盘 | Serial2 | 输出 | 自定义二进制 |
| 舵机控制板 | Serial1 (与Jetson共享) | 输出 | 众灵ASCII字符串 |

**注意**：ESP32S3 硬件仅 USB CDC + 2路UART，舵机与Jetson共`Serial1`（默认配置，实际部署需调整）。

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
- **机械臂 0x20~0x2F**: HOME/左臂夹取/左臂放下/右臂夹取/右臂放下/双臂夹放/停止/扭力控制
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
│   │   ├── ArmConfig.h               # 舵机ID(1~8)、PWM范围(500~2500)、动作组结构体
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
- 舵机ID 1~4=左臂(底座/肩/肘/夹爪), 5~8=右臂(底座/肩/肘/夹爪)

### 动作组 (ArmActionGroups)
11个预设动作: HOME / LEFT_PICK / LEFT_PLACE / RIGHT_PICK / RIGHT_PLACE / BOTH_PICK / BOTH_PLACE / STOP_ALL / EMERGENCY_STOP / TORQUE_OFF_ALL / TORQUE_ON_ALL

**所有PWM值为TODO占位值，需现场标定。** 动作组在 `src/arm/ArmActionGroups.cpp` 中修改。

### setPins() 参数顺序风险
Arduino Core 3.x 的 `setPins(RX, TX)` 签名可能与旧版不同。当前代码使用 `setPins(TX, RX)`，如串口无数据请尝试交换顺序。

### 编译验证
- 编译通过: RAM 6.0% (19548/327680), Flash 9.0% (301013/3342336)
- 无第三方依赖，仅需 platform = espressif32 + framework = arduino

## 项目状态

- [x] 完整工程代码 (编译通过)
- [x] 通信协议文档 (docs/通信协议规范.md)
- [ ] 舵机实际标定 (PWM值全为占位)
- [ ] 硬件接线验证
- [ ] 系统联调

## 重要提醒

1. 修改 `ArmActionGroups.cpp` 中的PWM前必须先在实体舵机上测试安全范围
2. 串口接线时注意TX/RX交叉、共地、波特率一致
3. ESP32S3 硬件串口有限，ServoSerial 和 JetsonSerial 默认共享 Serial1
4. `BROADCAST_ADDR = 0x10` 是系统协议广播，众灵舵机自身广播ID=255，两者不同
5. 不在 `loop()` 中放任何业务逻辑，不用阻塞式 `readString()`
6. 串口发送全部经过 `SerialManager` (Mutex保护)，禁止业务代码直接操作串口
