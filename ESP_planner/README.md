# ESP32S3 双臂复合机器人主控系统

## 一、项目简介

本项目是"小型双臂复合机器人系统"的 **ESP32S3 主控端**代码，基于 **PlatformIO + Arduino + FreeRTOS** 开发。

ESP32S3 作为系统中心控制节点，负责：

- 接收地面站人工控制指令
- 接收 Jetson Nano 物体识别结果
- 向 STM32F103 底盘转发运动指令
- 控制双机械臂（众灵总线舵机）
- 心跳与状态上报
- 急停处理

## 二、系统分工

| 设备 | 功能 | 通信方式 |
|------|------|----------|
| **ESP32S3**（本工程） | 中心控制、双机械臂控制、指令转发 | — |
| Jetson Nano + USB 摄像头 | 物体识别 | 串口 → ESP32S3 |
| STM32F103 + 麦克纳姆轮 | 底盘全向移动 | 串口 ← ESP32S3 |
| 地面站（ESP32 + 串口屏） | 人工控制 | 无线串口 ↔ ESP32S3 |
| 众灵总线舵机控制板 | 双机械臂 8 路舵机 | 串口 ← ESP32S3 |

**本工程不包含**：Jetson 视觉识别代码、STM32 底盘控制代码、地面站 UI 代码。

## 三、文件目录说明

```
ESP_planner/
├── platformio.ini                  # PlatformIO 项目配置
├── README.md                       # 本文件
├── include/                        # 头文件
│   ├── config/
│   │   ├── HardwareConfig.h        # 串口引脚、波特率等硬件配置
│   │   ├── DeviceConfig.h          # 设备地址、协议版本
│   │   ├── ArmConfig.h             # 舵机 ID、姿态结构体、动作组结构体
│   │   └── TaskConfig.h            # 任务栈大小、优先级、队列长度
│   ├── protocol/
│   │   ├── Protocol.h              # 协议帧格式、功能码、消息类型
│   │   ├── ProtocolParser.h        # 协议帧解析器（状态机）
│   │   └── Crc16.h                 # CRC-16/MODBUS 校验
│   ├── servo/
│   │   └── BusServoDriver.h        # 众灵总线舵机驱动
│   ├── arm/
│   │   ├── ArmActionGroups.h       # 预设动作组声明
│   │   └── ArmController.h         # 机械臂控制器
│   ├── comm/
│   │   ├── SerialManager.h         # 串口管理器
│   │   └── CommandRouter.h         # 命令路由任务
│   └── app/
│       └── RobotState.h            # 系统状态管理
├── src/                            # 源文件（与 include/ 结构对应）
│   ├── main.cpp                    # 入口，任务创建
│   ├── protocol/
│   │   ├── Protocol.cpp
│   │   ├── ProtocolParser.cpp
│   │   └── Crc16.cpp
│   ├── servo/
│   │   └── BusServoDriver.cpp
│   ├── arm/
│   │   ├── ArmActionGroups.cpp     # ★ 动作组参数（现场调参重点）
│   │   └── ArmController.cpp
│   ├── comm/
│   │   ├── SerialManager.cpp
│   │   └── CommandRouter.cpp
│   └── app/
│       └── RobotState.cpp
└── test/
    └── README
```

## 四、串口连接与配置说明

### 4.1 逻辑通信通道

本工程需要四个逻辑通信通道：

| 通道 | 用途 | 方向 | 默认串口对象 |
|------|------|------|-------------|
| 调试串口 | USB 日志输出 | 输出 | `Serial` (USB CDC) |
| 地面站 | 人工控制指令 | 双向 | `Serial` (USB CDC, 与调试复用) |
| Jetson | 物体识别结果 | 输入 | `Serial1` |
| STM32 底盘 | 运动指令 | 输出 | `Serial2` |
| 舵机控制板 | 舵机字符串指令 | 输出 | `Serial1` (与 Jetson 共享) |

### 4.2 硬件串口数量限制

**ESP32S3 实际可用硬件串口有限**（USB CDC + UART1 + UART2 ≈ 3 个独立通道）。

默认配置中 **ServoSerial 与 JetsonSerial 共享 Serial1**，仅保证代码可编译。实际部署时请根据接线调整：

**可选方案**：
1. 使用 USB CDC 作为调试+地面站通道，释放更多硬件 UART
2. 地面站使用独立无线串口模块（如 HC-12），接入 Serial1 或 Serial2
3. 使用外接串口扩展模块（如 I2C 转 UART：SC16IS750）
4. 通过宏切换：在 `HardwareConfig.h` 中修改 `#define ServoSerial Serial1` 为其他可用串口
5. 舵机控制板和 Jetson 分时复用同一串口（需协调通信时序）

### 4.3 当前默认映射

所有串口配置在 [`include/config/HardwareConfig.h`](include/config/HardwareConfig.h) 中：

```cpp
#define DebugSerial   Serial       // USB CDC
#define GroundSerial  Serial       // USB CDC，与调试复用
#define JetsonSerial  Serial1      // TX=GPIO17, RX=GPIO18
#define ChassisSerial Serial2      // TX=GPIO4,  RX=GPIO5
#define ServoSerial   Serial1      // TX=GPIO16, RX=GPIO15（与 Jetson 共享 Serial1）
```

### 4.4 修改 RX/TX 引脚

在 `HardwareConfig.h` 中修改对应 `PIN_*_TX` / `PIN_*_RX` 常量即可：

```cpp
constexpr int PIN_JETSON_TX = 17;   // 改为你的实际 TX 引脚
constexpr int PIN_JETSON_RX = 18;   // 改为你的实际 RX 引脚
```

### 4.5 `setPins()` 参数顺序说明

**重要**：ESP32 Arduino Core 的 `HardwareSerial::setPins()` 参数顺序为：

```cpp
// 标准 Arduino Core 3.x 签名
void setPins(int8_t rxPin, int8_t txPin, int8_t ctsPin = -1, int8_t rtsPin = -1);
//            RX first ^^^^^^    TX second ^^^^^^
```

**但本工程使用顺序可能与之不同**。实际工程中 `setPins()` 的有效参数顺序取决于 Arduino Core 版本。

在 [`src/comm/SerialManager.cpp`](src/comm/SerialManager.cpp) 的 `initSerial()` 函数中，当前使用：

```cpp
serial.setPins(txPin, rxPin);   // 第1个参数是TX，第2个是RX
```

### 4.6 串口无数据时的检查清单

如果某个串口没有数据，请按以下顺序排查：

1. **TX/RX 是否接反** — 最常⭒的问题，交叉连接试试
2. **GND 是否共地** — 两个设备必须共地
3. **波特率是否一致** — 双方波特率必须相同
4. **`setPins()` 参数顺序是否正确** — 尝试交换为 `setPins(RX, TX)` 或 `setPins(TX, RX)`
5. **是否占用了启动相关 GPIO** — ESP32S3 的 GPIO0, GPIO2, GPIO12, GPIO15 等有特殊功能
6. **是否多个设备共用了同一串口** — 检查 `HardwareConfig.h` 中是否有串口对象冲突
7. **是否需要外接串口扩展模块** — 如果硬件串口确实不够用

## 五、通信协议说明

### 5.1 协议帧格式

```
┌────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────┬────────────┬────────┬────────┬────────┐
│ Head1  │ Head2  │ Version│ SrcAddr│ DstAddr│MsgType │FuncCode│ SeqNum │  DataLen   │ Data   │ CRC16  │ Tail1  │ Tail2  │
│ 0xAA   │ 0x55   │ 1 Byte │ 1 Byte │ 1 Byte │ 1 Byte │ 1 Byte │ 1 Byte │  1 Byte    │0~64Byte│ 2 Byte │ 0x0D   │ 0x0A   │
└────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────┴────────────┴────────┴────────┴────────┴────────┘
                                                                                                      │ LSB first │
                                                                                                      └───────────┘
```

- **帧头**：2 字节，固定 `0xAA 0x55`
- **版本号**：1 字节，当前 `0x01`
- **源地址**：1 字节，发送方设备地址
- **目标地址**：1 字节，接收方设备地址
- **消息类型**：1 字节（命令/ACK/NACK/数据/心跳）
- **功能码**：1 字节，具体命令
- **序列号**：1 字节，滚动递增
- **数据长度**：1 字节，数据区字节数（0~64）
- **数据区**：0~64 字节，变长
- **CRC16**：2 字节，CRC-16/MODBUS，低字节在前
- **帧尾**：2 字节，固定 `0x0D 0x0A`

### 5.2 设备地址

| 地址 | 名称 | 说明 |
|------|------|------|
| `0x10` | BROADCAST | **系统协议广播地址**（非舵机广播ID） |
| `0x20` | ESP32S3 | 本机 |
| `0x30` | Jetson Nano | 视觉识别 |
| `0x40` | STM32F103 | 小车底盘 |
| `0x50` | GROUND | 地面站 |
| `0x60` | SERVO | 舵机控制板（逻辑地址） |

**注意**：`BROADCAST_ADDR = 0x10` 是系统协议的广播地址，**众灵舵机自身的广播 ID = 255**，两者不同，请勿混淆。

### 5.3 消息类型

| 值 | 类型 | 说明 |
|----|------|------|
| `0x01` | CMD | 命令 |
| `0x02` | ACK | 确认应答 |
| `0x03` | NACK | 否认应答（数据区含错误码） |
| `0x04` | DATA | 数据/状态上报 |
| `0x05` | HEARTBEAT | 心跳 |

### 5.4 功能码

#### 系统类（0x00 ~ 0x0F）
| 码值 | 名称 | 说明 |
|------|------|------|
| `0x01` | HEARTBEAT | 心跳 |
| `0x02` | ACK | 确认 |
| `0x03` | NACK | 否认 |
| `0x04` | STATUS_QUERY | 状态查询 |
| `0x05` | STATUS_REPORT | 状态上报 |
| `0x06` | ERROR_REPORT | 错误上报 |
| `0x07` | EMERGENCY_STOP | 急停 |

#### 小车底盘类（0x10 ~ 0x1F）
| 码值 | 名称 | 说明 |
|------|------|------|
| `0x10` | CHASSIS_FORWARD | 前进 |
| `0x11` | CHASSIS_BACKWARD | 后退 |
| `0x12` | CHASSIS_LEFT | 左移 |
| `0x13` | CHASSIS_RIGHT | 右移 |
| `0x14` | CHASSIS_ROTATE_LEFT | 左转 |
| `0x15` | CHASSIS_ROTATE_RIGHT | 右转 |
| `0x16` | CHASSIS_STOP | 停止 |
| `0x17` | CHASSIS_MOVE_VECTOR | 矢量移动 |

#### 机械臂类（0x20 ~ 0x2F）
| 码值 | 名称 | 说明 |
|------|------|------|
| `0x20` | ARM_HOME | 双臂复位 |
| `0x21` | ARM_LEFT_PICK | 左臂夹取 |
| `0x22` | ARM_LEFT_PLACE | 左臂放下 |
| `0x23` | ARM_RIGHT_PICK | 右臂夹取 |
| `0x24` | ARM_RIGHT_PLACE | 右臂放下 |
| `0x25` | ARM_BOTH_PICK | 双臂夹取 |
| `0x26` | ARM_BOTH_PLACE | 双臂放下 |
| `0x27` | ARM_STOP | 停止所有舵机 |
| `0x28` | ARM_TORQUE_OFF | 释放扭力 |
| `0x29` | ARM_TORQUE_ON | 恢复扭力 |

#### 视觉识别类（0x30 ~ 0x3F）
| 码值 | 名称 | 说明 |
|------|------|------|
| `0x30` | OBJECT_INFO | 物体识别信息（Jetson → ESP32） |

**CMD_OBJECT_INFO 数据区格式**：
- `data[0]`：objectType（`0x01`=货物A, `0x02`=货物B, `0x03`=未知）
- `data[1]`：objectCount（0~255）

## 六、舵机配置

### 6.1 舵机 ID 修改

舵机 ID 定义在 [`include/config/ArmConfig.h`](include/config/ArmConfig.h)：

```cpp
constexpr uint8_t LEFT_BASE_ID     = 1;   // 左臂底座旋转
constexpr uint8_t LEFT_SHOULDER_ID = 2;   // 左臂肩关节
constexpr uint8_t LEFT_ELBOW_ID    = 3;   // 左臂肘关节
constexpr uint8_t LEFT_GRIPPER_ID  = 4;   // 左臂夹爪
// ... 右臂同理，ID 5~8
```

**修改方式**：直接修改对应 `constexpr` 值为实际舵机 ID 即可，业务代码自动同步。

### 6.2 舵机参数限制

- ID 范围：0 ~ 254（255 为舵机广播 ID，特殊控制用）
- PWM 范围：500 ~ 2500（1500 为中位）
- 运动时间：0 ~ 9999 ms

### 6.3 众灵舵机协议格式

| 指令 | 格式 | 说明 |
|------|------|------|
| 单舵机控制 | `#000P1500T1000!` | ID=000, 位置=1500, 时间=1000ms |
| 多舵机组合 | `{#000P1500T1000!#001P1600T1000!}` | 花括号包裹多条指令 |
| 读取角度 | `#000PRAD!` | 预留接口 |
| 停止 | `#000PDST!` | 立即停止 |
| 暂停 | `#000PDPT!` | 暂停运动 |
| 继续 | `#000PDCT!` | 继续运动 |
| 释力 | `#000PULK!` | 释放扭力 |
| 恢复 | `#000PULR!` | 恢复扭力 |

## 七、机械臂动作组修改

所有预设动作组的 PWM 参数集中在 [`src/arm/ArmActionGroups.cpp`](src/arm/ArmActionGroups.cpp)。

**现场调参步骤**：
1. 先单独控制每个舵机：通过串口发送 `moveServo(id, pwm, time)` 确定安全范围
2. 确定夹取位置、放下位置、复位位置对应的 PWM 值
3. 确定合适的运动时间（T 值）
4. 确定步骤间等待时间（waitMs）
5. 修改 `ArmActionGroups.cpp` 中对应动作组的参数

**注意**：当前所有动作组 PWM 值为示例值（大部分为 1500 中位），**必须连接实际舵机后根据机械结构重新标定**，否则可能导致机械臂碰撞或损坏。

## 八、如何编译

### 前提条件

1. 安装 [Visual Studio Code](https://code.visualstudio.com/)
2. 安装 [PlatformIO IDE 扩展](https://platformio.org/install/ide?install=vscode)
3. 或安装 [PlatformIO Core (CLI)](https://docs.platformio.org/en/latest/core/installation.html)

### 编译步骤

```bash
# 在项目根目录下
cd ESP_planner

# 编译
platformio run

# 或使用 pio 命令
pio run
```

在 VSCode 中：点击底部状态栏的 **✓** (Build) 按钮。

## 九、如何烧录

```bash
# 编译并烧录
platformio run --target upload

# 或
pio run -t upload
```

在 VSCode 中：点击底部状态栏的 **→** (Upload) 按钮。

### 串口监视

```bash
platformio device monitor
# 或
pio device monitor
```

在 VSCode 中：点击底部状态栏的 **🔌** (Monitor) 按钮。

## 十、如何调试

1. **USB 串口日志**：`DebugSerial`（USB CDC）输出详细运行日志，波特率 115200
2. **状态摘要**：每 5 秒自动打印系统状态摘要（模式、机械臂状态、识别信息、错误计数等）
3. **LED 指示**：急停状态下 LED 闪烁（需配置 `PIN_LED_STATUS`）
4. **错误日志**：CRC 错误、帧格式错误、队列溢出等均有相应日志输出
5. **串口调试**：可使用任意串口助手接入 `DebugSerial` 查看日志

## 十一、FreeRTOS 任务一览

| 任务 | 函数 | 优先级 | 核心 | 功能 |
|------|------|--------|------|------|
| GROUND_RX | `taskGroundStation` | 3 | Core 1 | 地面站串口接收+解析 |
| JETSON_RX | `taskJetson` | 3 | Core 0 | Jetson 串口接收+解析 |
| CMD_ROUTER | `taskCommandRouter` | 3 | Core 1 | 命令分类路由 |
| ARM | `taskArm` | 4 | Core 1 | 机械臂动作组执行 |
| STATUS | `taskStatus` | 1 | Core 1 | 心跳+状态上报 |

## 十二、后续现场调参步骤

1. **上电前检查**
   - 确认所有接线正确（TX/RX、GND、电源）
   - 确认舵机 ID 与代码中配置一致
   - 确认波特率一致

2. **串口通信验证**
   - 先只连接调试 USB，确认系统正常启动
   - 逐步接入 Jetson、STM32、地面站，每次只接入一个设备
   - 通过日志观察是否有帧接收、CRC 错误等情况

3. **舵机标定**
   - 使用 `BusServoDriver::moveServo()` 逐个测试每个舵机
   - 记录各关节的安全 PWM 范围
   - 确定夹爪"夹紧"和"张开"的 PWM 值

4. **动作组调参**
   - 修改 `ArmActionGroups.cpp` 中的 PWM 参数
   - 从复位（HOME）开始，逐个测试每个动作组
   - 调整运动时间和步骤等待时间

5. **系统联调**
   - 地面站 → ESP32 → STM32 底盘控制链路
   - 地面站 → ESP32 → 机械臂控制链路
   - Jetson → ESP32 视觉信息接收链路
   - 急停功能验证

## 十三、设计要点

1. **不使用阻塞式 `readString()`** — 串口接收使用状态机逐字节解析
2. **不在 `loop()` 中接收串口数据** — 所有串口接收在独立 FreeRTOS 任务中
3. **串口发送 Mutex 保护** — 多任务写同一串口时不会冲突
4. **单次接收限制** — 每次最多处理 96 字节，防止某个串口任务长时间占用 CPU
5. **不使用动态内存** — 所有缓冲区在编译期确定大小
6. **配置集中管理** — 引脚、地址、优先级等集中定义在 `include/config/` 中
7. **舵机通信封装** — 业务代码不直接拼接舵机字符串，通过 `BusServoDriver` 操作
