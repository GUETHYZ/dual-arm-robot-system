/**
 * @file HardwareConfig.h
 * @brief 硬件引脚和串口配置
 *
 * 所有串口对象、引脚、波特率集中定义在此文件。
 * 修改硬件接线时，只需修改此文件中的引脚定义。
 *
 * 重要说明：
 * 1. ESP32S3 的 setPins() 参数顺序可能因 Arduino Core 版本不同而异。
 *    本工程默认使用 setPins(TX, RX) 顺序。
 *    如果串口无数据，请首先检查：
 *    - TX/RX 是否接反
 *    - GND 是否共地
 *    - 波特率是否一致
 *    - setPins() 参数顺序是否需要交换为 setPins(RX, TX)
 *    - 是否占用了启动相关 GPIO（如 GPIO0, GPIO2, GPIO12, GPIO15 等）
 *    - 是否多个设备共用了同一串口
 *
 * 2. 当前默认映射（可编译方案）：
 *    - DebugSerial  = Serial      (USB CDC，调试日志)
 *    - GroundSerial = Serial      (USB CDC，与调试复用)
 *    - JetsonSerial = Serial1     (GPIO 可自定义)
 *    - ChassisSerial = Serial2    (GPIO 可自定义)
 *    - ServoSerial  = Serial1     (与 Jetson 共享 Serial1，实际部署需拆分)
 *
 *    说明：ESP32S3 实际可用硬件串口有限（USB CDC + UART1 + UART2）。
 *    默认方案中 ServoSerial 与 JetsonSerial 共享 Serial1 硬件，
 *    仅用于保证代码可编译。实际部署时请根据接线调整。
 */

#pragma once

#include <Arduino.h>

// ============================================================
// 调试串口（USB CDC，用于日志输出）
// ============================================================
#define DebugSerial Serial
constexpr uint32_t DEBUG_BAUD = 115200;

// ============================================================
// 地面站串口
// 默认复用 USB Serial，与调试串口共用同一物理通道
// 如果地面站使用独立无线串口模块，请修改为 Serial1 或 Serial2
// 并配置对应引脚
// ============================================================
#define GroundSerial Serial
constexpr uint32_t GROUND_BAUD = 115200;
// 地面站独立引脚（仅当 GroundSerial 不为 Serial 时使用）
constexpr int PIN_GROUND_TX = 10;   // TODO: 根据实际接线修改
constexpr int PIN_GROUND_RX = 9;    // TODO: 根据实际接线修改

// ============================================================
// Jetson 串口
// 接收 Jetson Nano 发送的物体识别结果
// ============================================================
#define JetsonSerial Serial1
constexpr int PIN_JETSON_TX = 17;   // TODO: 根据实际接线修改
constexpr int PIN_JETSON_RX = 18;   // TODO: 根据实际接线修改
constexpr uint32_t JETSON_BAUD = 115200;

// ============================================================
// STM32 小车底盘串口
// 向 STM32F103 底盘发送运动指令
// ============================================================
#define ChassisSerial Serial2
constexpr int PIN_CHASSIS_TX = 4;   // TODO: 根据实际接线修改
constexpr int PIN_CHASSIS_RX = 5;   // TODO: 根据实际接线修改
constexpr uint32_t CHASSIS_BAUD = 115200;

// ============================================================
// 众灵舵机控制板串口
// 发送舵机控制字符串指令
//
// 注意：默认与 JetsonSerial 共享 Serial1，仅保证代码可编译。
// 实际部署时，如果硬件串口不足，可选方案：
//   1. 将 ServoSerial 改为与 ChassisSerial 共享 Serial2
//   2. 使用外接串口扩展模块（如 I2C 转 UART）
//   3. 将地面站改为 USB CDC，释放 Serial1 给舵机
//   4. 根据实际接线调整通信拓扑
// ============================================================
#define ServoSerial Serial1
constexpr int PIN_SERVO_TX = 16;    // TODO: 根据实际接线修改
constexpr int PIN_SERVO_RX = 15;    // TODO: 根据实际接线修改
constexpr uint32_t SERVO_BAUD = 115200;

// ============================================================
// 舵机控制板 IO 电平控制引脚（可选）
// 部分舵机控制板需要通过 IO 口切换 TTL/RS485 方向
// 如果不需要，保持为 -1 即可
// ============================================================
constexpr int PIN_SERVO_DIR = -1;   // TODO: 根据实际接线修改，-1 表示不使用

// ============================================================
// 状态指示灯（可选）
// ============================================================
constexpr int PIN_LED_STATUS = 48;  // TODO: 根据实际接线修改，-1 表示不使用
constexpr int PIN_BUZZER     = -1;  // TODO: 根据实际接线修改

// ============================================================
// 急停按钮引脚（可选）
// ============================================================
constexpr int PIN_EMERGENCY_STOP = -1;  // TODO: 根据实际接线修改，-1 表示不使用
