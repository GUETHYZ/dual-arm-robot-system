/**
 * @file HardwareConfig.h
 * @brief 硬件引脚和串口配置
 *
 * 所有串口对象、引脚、波特率集中定义在此文件。
 * 修改硬件接线时，只需修改此文件中的引脚定义。
 *
 * 当前部署映射：
 * 1. DebugSerial = Serial (USB CDC)
 *    - 调试日志独立走 ESP32S3 USB CDC，不与任何业务串口复用。
 *    - platformio.ini 启用 ARDUINO_USB_CDC_ON_BOOT=1，用于释放 UART0 给业务串口。
 *
 * 2. GroundSerial / LoraSerial = Serial2，无线 TTL 串口桥接地面站
 *    - Serial2 RX = GPIO18，连接无线串口模块 TX
 *    - Serial2 TX = GPIO17，连接无线串口模块 RX
 *    - 地面站协议不再复用 USB 调试口。
 *
 * 3. ServoSerial = Serial1，众灵 ZP 舵机控制板
 *    - Serial1 RX = GPIO11，连接舵机驱动板 TX（如驱动板无回传可不接）
 *    - Serial1 TX = GPIO12，连接舵机驱动板 RX
 *
 * 4. JetsonSerial / ChassisSerial = Serial0，单 UART 单向复用
 *    - Serial0 RX = GPIO1，接 Jetson TX，仅接收视觉识别帧
 *    - Serial0 TX = GPIO2，接 STM32 RX，仅发送底盘控制帧
 *    - GPIO19 / GPIO20 预留给 USB CDC 调试口；Jetson 与 STM32 不在同一根信号线上通信。
 *
 * 重要排查项：TX/RX 交叉、所有设备共地、波特率一致、外部供电充足。
 */

#pragma once

#include <Arduino.h>
#include "soc/soc_caps.h"

#ifndef ARDUINO_USB_CDC_ON_BOOT
#define ARDUINO_USB_CDC_ON_BOOT 0
#endif

static_assert(SOC_UART_NUM >= 3, "当前串口映射需要 ESP32S3 至少 3 个硬件 UART 控制器");
#if !ARDUINO_USB_CDC_ON_BOOT
#error "当前串口映射要求 DebugSerial 使用 USB CDC；请在 platformio.ini 中启用 ARDUINO_USB_CDC_ON_BOOT=1"
#endif

// ============================================================
// 调试串口（USB CDC，用于日志输出）
// ============================================================
#define DebugSerial Serial
constexpr uint32_t DEBUG_BAUD = 115200;

// ============================================================
// 地面站 / LORA 无线 TTL 串口
// 地面站通过无线 TTL 串口桥接，不再复用 USB 调试口。
// Arduino-ESP32 HardwareSerial::setPins(rxPin, txPin)：RX 在前，TX 在后。
// ============================================================
#define LoraSerial Serial2
#define GroundSerial LoraSerial
constexpr int LORA_RX_PIN = 18;     // RX: 无线串口模块 TX -> ESP32 RX
constexpr int LORA_TX_PIN = 17;     // TX: ESP32 TX -> 无线串口模块 RX
constexpr int PIN_GROUND_RX = LORA_RX_PIN;
constexpr int PIN_GROUND_TX = LORA_TX_PIN;
constexpr uint32_t GROUND_BAUD = 115200;

// ============================================================
// Jetson / STM32 共享串口
// 当前仅接收 Jetson 识别结果、仅向 STM32 发送底盘命令，因此复用 UART0。
// 注意：DebugSerial 已切换为 USB CDC，UART0 不再作为调试串口使用。
// ============================================================
#define JetsonSerial Serial0
#define ChassisSerial Serial0
constexpr int PIN_SHARED_RX = 1;    // Serial0 RX: Jetson TX -> ESP32 RX
constexpr int PIN_SHARED_TX = 2;    // Serial0 TX: ESP32 TX -> STM32 RX
constexpr int PIN_JETSON_RX = PIN_SHARED_RX;
constexpr int PIN_JETSON_TX = PIN_SHARED_TX;
constexpr uint32_t JETSON_BAUD = 115200;
constexpr int PIN_CHASSIS_RX = PIN_SHARED_RX;
constexpr int PIN_CHASSIS_TX = PIN_SHARED_TX;
constexpr uint32_t CHASSIS_BAUD = 115200;
static_assert(JETSON_BAUD == CHASSIS_BAUD, "Jetson 和 STM32 复用同一个 UART 时必须使用相同波特率");

// ============================================================
// 众灵舵机控制板串口
// 优先保证舵机板独立串口；如驱动板无回传，RX 可不接。
// ============================================================
#define ServoSerial Serial1
constexpr int PIN_SERVO_RX = 11;    // RX: 舵机驱动板 TX -> ESP32 RX（可选）
constexpr int PIN_SERVO_TX = 12;    // TX: ESP32 TX -> 舵机驱动板 RX
constexpr uint32_t SERVO_BAUD = 115200;

// ============================================================
// 舵机控制板 IO 电平控制引脚（可选）
// 当前众灵驱动板负责半双工转换，不需要方向控制引脚。
// 如果不需要，保持为 -1 即可。
// ============================================================
constexpr int PIN_SERVO_DIR = -1;   // TODO: 根据实际接线修改，-1 表示不使用

// ============================================================
// 状态指示灯（可选）
// ============================================================
constexpr int PIN_LED_STATUS = 48;  // TODO: 根据实际接线修改，-1 表示不使用
constexpr int PIN_BUZZER     = -1;  // TODO: 根据实际接线修改

// ============================================================
// 地面站接收指示灯（现场最小通信验证用）
// ============================================================
constexpr int PIN_VOICE_AND_LED = 10;       // 当前板载语音/LED 外设引脚，用于地面站有效帧接收指示
constexpr int PIN_GROUND_RX_LED = PIN_VOICE_AND_LED;
constexpr int GROUND_RX_LED_ON_LEVEL = HIGH;
constexpr int GROUND_RX_LED_OFF_LEVEL = LOW;
constexpr uint32_t GROUND_RX_LED_PULSE_MS = 80;

// ============================================================
// 急停按钮引脚（可选）
// ============================================================
constexpr int PIN_EMERGENCY_STOP = -1;  // TODO: 根据实际接线修改，-1 表示不使用
