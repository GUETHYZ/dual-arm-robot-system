# CLAUDE.md

> 小型双臂复合机器人系统 · 地面站端 · 工程速查手册
>
> 更新日期：2026-07-06

---

## 项目概述

本项目是"小型双臂复合机器人系统"的**地面站**，基于 ESP32S3 + 陶晶驰 TJC 串口屏 + LoRa 无线模块，实现人工遥控小车移动和双臂动作。

- **平台**: PlatformIO + Arduino (espressif32 @ 6.11.0)
- **开发板**: Freenove ESP32-S3 WROOM N8R8
- **框架**: Arduino (framework-arduinoespressif32 @ 3.20017)
- **MCU**: ESP32S3 240MHz, 320KB RAM, 8MB Flash


## 硬件引脚

### 串口分配

| 串口     | 用途           | RX    | TX    | 波特率   |
| -------- | -------------- | ----- | ----- | -------- |
| Serial   | USB 调试输出   | -     | -     | 115200   |
| Serial1  | LoRa 无线模块  | GPIO18| GPIO17| 115200   |
| Serial2  | TJC 串口屏     | GPIO1 | GPIO2 | 115200   |

### 注意事项
- `setPins(rx, tx)` 参数顺序可能因 Arduino Core 版本而异，实测无数据时尝试交换。
- TJC 屏幕与 ESP32 交叉连接：TJC TX → ESP32 RX，TJC RX → ESP32 TX。
- `#define DebugSerial Serial` — 所有调试输出通过 USB Serial。

## 工程结构

```text
├── platformio.ini                    # PlatformIO 配置
├── CLAUDE.md                         # 本文件
├── docs/
│   └── 通信协议规范.md                # 协议规范（与代码常量一致）
├── include/
│   ├── config/
│   │   ├── HardwareConfig.h          # 引脚/串口/波特率
│   │   ├── DeviceConfig.h            # 地址/消息类型/功能码/协议常量
│   │   └── TaskConfig.h              # FreeRTOS 栈/优先级/周期/队列长度
│   ├── protocol/
│   │   ├── Protocol.h                # ProtocolFrame + packFrame + nextSeq
│   │   ├── ProtocolParser.h          # 14 状态状态机解析器
│   │   └── Crc16.h                   # CRC-16/MODBUS
│   ├── tjc/
│   │   └── TjcPanel.h                # TJC 串口屏驱动（4 状态状态机）
│   ├── wireless/
│   │   └── WirelessLink.h            # LoRa 收发（Mutex 保护）
│   └── app/
│       ├── GroundStationState.h      # 全局状态管理（Mutex 线程安全）
│       └── CommandMapper.h           # TJC CMD → 协议 funcCode 映射
└── src/
    ├── main.cpp                      # 入口 + 4 个 FreeRTOS 任务
    ├── protocol/
    │   ├── Protocol.cpp              # 打包 + 序列号 + 字符串工具
    │   ├── ProtocolParser.cpp        # 逐字节状态机实现
    │   └── Crc16.cpp                 # 查表法 CRC16 实现
    ├── tjc/
    │   └── TjcPanel.cpp              # TJC 帧解析 + 指令发送
    ├── wireless/
    │   └── WirelessLink.cpp          # LoRa 收发实现
    └── app/
        ├── GroundStationState.cpp    # 状态读写 + 调试打印
        └── CommandMapper.cpp         # 9 条映射表
```

## FreeRTOS 任务一览

| 任务     | 函数                 | 周期    | 优先级 | 栈     |
| -------- | -------------------- | ------- | ------ | ------ |
| tjc_rx   | tjcReceiveTask       | 5ms     | 3      | 4096   |
| lora_rx  | wirelessReceiveTask  | 5ms     | 3      | 4096   |
| lora_tx  | wirelessSendTask     | 10ms    | 3      | 4096   |
| status   | statusTask           | 200ms   | 1      | 3072   |
| main_task| mainTask             | 一次性  | 5      | 8192   |

### 数据流

```text
TJC 屏幕 → (Serial2) → tjcReceiveTask → CommandMapper → txCommandQueue
                                                              ↓
LoRa ← (Serial1) ← wirelessSendTask ← xQueueReceive ← txCommandQueue

LoRa → (Serial1) → wirelessReceiveTask → ProtocolParser → GroundStationState
                                                              ↓
                                                        DebugSerial (打印)
```

### 同步原语

| 名称            | 类型              | 用途           |
| --------------- | ----------------- | -------------- |
| txCommandQueue  | QueueHandle_t     | TJC→LoRa 命令传递 |
| loraTxMutex     | SemaphoreHandle_t | LoRa 串口发送保护 |
| stateMutex      | SemaphoreHandle_t | 状态读写保护   |

## 通信协议摘要

完整文档见 [docs/通信协议规范.md](docs/通信协议规范.md)。

### 统一协议帧

```text
AA 55 | Ver | Src | Dst | Type | Func | Seq | Len | Data(0~64B) | CRC16(LE) | 0D 0A
```

- 地面站地址: `0x50` (GROUND_ADDR)
- 车载主控地址: `0x20` (ESP32_ADDR)
- CRC: CRC-16/MODBUS (初始 0xFFFF, 多项式 0xA001)，计算范围 0xAA ~ Data 末尾
- CRC 低字节在前

### TJC 按钮事件帧

```text
DD CMD CMD DE
```

- 帧头 `0xDD`，帧尾 `0xDE`，CMD 重复一次做校验
- 4 状态状态机解析，错误帧自动重同步

### TJC CMD → 协议 funcCode 映射

| TJC CMD | 来源             | funcCode          | 含义     |
| ------- | ---------------- | ----------------- | -------- |
| `0x01`  | b_forward 按下   | `0x10` FORWARD    | 前进     |
| `0x02`  | b_back 按下      | `0x11` BACKWARD   | 后退     |
| `0x03`  | b_right 按下     | `0x13` RIGHT      | 右移     |
| `0x04`  | b_left 按下      | `0x12` LEFT       | 左移     |
| `0x05`  | b_clamp 按下     | `0x25` BOTH_PICK  | 双臂夹取 |
| `0x06`  | b_put_down 按下  | `0x26` BOTH_PLACE | 双臂放下 |
| `0x10`  | 移动弹起/b_stop  | `0x16` STOP       | 底盘停止 |
| `0x11`  | b_emergency      | `0x07` EMERGENCY  | 急停     |
| `0x12`  | b_arm_stop       | `0x27` ARM_STOP   | 机械臂停止 |

**交互逻辑**：
- 移动按钮（forward/back/left/right）：**按下发送方向 → 弹起发送停止(0x10)**，模拟遥控器行为。
- 机械臂按钮（clamp/put_down）：**仅按下事件**，按下即发送一次动作命令。
- 预留按钮（stop/emergency/arm_stop）：代码已包含映射，只需在 TJC 设计软件中添加按钮并配置 `printh`。

## TJC 按钮 printh 配置速查

### 移动按钮（按下 + 弹起）

| 按钮      | 事件       | printh 指令          |
| --------- | ---------- | -------------------- |
| b_forward | 按下事件   | `printh DD 01 01 DE` |
| b_forward | 弹起事件   | `printh DD 10 10 DE` |
| b_back    | 按下事件   | `printh DD 02 02 DE` |
| b_back    | 弹起事件   | `printh DD 10 10 DE` |
| b_right   | 按下事件   | `printh DD 03 03 DE` |
| b_right   | 弹起事件   | `printh DD 10 10 DE` |
| b_left    | 按下事件   | `printh DD 04 04 DE` |
| b_left    | 弹起事件   | `printh DD 10 10 DE` |

### 机械臂按钮（仅按下）

| 按钮       | 事件       | printh 指令          |
| ---------- | ---------- | -------------------- |
| b_clamp    | 按下事件   | `printh DD 05 05 DE` |
| b_put_down | 按下事件   | `printh DD 06 06 DE` |

### 预留按钮（仅按下，代码已就绪）

| 按钮        | 事件       | printh 指令          |
| ----------- | ---------- | -------------------- |
| b_stop      | 按下事件   | `printh DD 10 10 DE` |
| b_emergency | 按下事件   | `printh DD 11 11 DE` |
| b_arm_stop  | 按下事件   | `printh DD 12 12 DE` |

## 当前实现状态

### 已完成
- [x] 多文件工程结构（config / protocol / tjc / wireless / app 五层分离）
- [x] 统一通信协议帧打包（CRC-16/MODBUS）
- [x] ProtocolParser 14 状态状态机解析器
- [x] TJC 按钮事件 4 状态状态机解析
- [x] CommandMapper 9 条 TJC→协议映射
- [x] FreeRTOS 四任务架构（TJC 收 / LoRa 收 / LoRa 发 / 状态监控）
- [x] Queue + Mutex 线程安全
- [x] 心跳超时检测（2000ms 离线判定）
- [x] ACK/NACK 解析与调试打印
- [x] STATUS_REPORT / OBJECT_INFO / ERROR_REPORT 解析
- [x] 离线状态变化串口打印（防刷屏）
- [x] 完整中文注释
- [x] 可编译通过

### 当前版本限制
- [ ] **TJC 屏幕 UI 不更新**：所有状态仅在 USB Serial 打印，不发送到屏幕显示
- [ ] **不做 ACK 超时重传**：仅打印 ACK/NACK，不实现自动重发
- [ ] **不发送 STATUS_QUERY**：地面站不主动查询车载状态
- [ ] **移动按钮只发启停**：不发送速度/矢量参数

### 预留扩展点
- `TjcPanel::refreshReservedStatus()` — 后续添加屏幕 UI 刷新
- `TjcPanel::setText()` — 已实现，可直接用于更新 TJC 文本控件
- `GroundCommand.data[]` / `dataLen` — 队列元素支持携带数据区
- 预留按钮 `b_stop`/`b_emergency`/`b_arm_stop` 映射已就绪

## 代码约定

- 语言: C++ (Arduino 框架)
- 注释: 中文
- 配置: 集中在 `include/config/` 下三个头文件
- 串口解析: 状态机，不用 `readString`，不阻塞
- 线程安全: Queue 用于任务间通信，Mutex 用于共享资源
- 无动态内存: 不使用 `new`/`malloc`，固定大小缓冲区
- 调试前缀: `[TJC RX]` `[LORA TX]` `[LORA RX]` `[STATE]` `[LINK]` `[ERROR]` `[MAPPER]` `[INIT]` `[TASK]`

## 常见调试问题

1. **LoRa 无数据**: 检查 TX/RX 交叉、GND 共地、波特率一致、模块已配对、`setPins` 参数顺序
2. **TJC 无数据**: 检查交叉接线、波特率 115200、`printh` 两位大写十六进制、按钮事件配置正确
3. **编译报错 `DebugSerial`**: 确认 `.cpp` 文件 include 了 `config/HardwareConfig.h`
4. **编译报错 `size_t`**: 确认 `DeviceConfig.h` include 了 `<cstddef>`
