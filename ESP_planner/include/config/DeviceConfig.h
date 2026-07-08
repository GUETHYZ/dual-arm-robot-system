/**
 * @file DeviceConfig.h
 * @brief 设备地址和系统标识配置
 *
 * 定义系统中所有设备的逻辑地址。
 * 这些地址用于统一通信协议中的源地址和目标地址字段。
 *
 * 注意：
 * - BROADCAST_ADDR = 0x10 是系统通信协议的广播地址
 * - 众灵舵机自身的广播 ID = 255，两者不是一回事
 * - 系统协议广播地址不要与舵机广播 ID 混淆
 */

#pragma once

#include <Arduino.h>

// ============================================================
// 系统设备地址
// ============================================================
constexpr uint8_t BROADCAST_ADDR = 0x10;  // 系统协议广播地址（非舵机广播ID）
constexpr uint8_t ESP32_ADDR     = 0x20;  // ESP32S3 主控
constexpr uint8_t JETSON_ADDR    = 0x30;  // Jetson Nano 视觉识别
constexpr uint8_t STM32_ADDR     = 0x40;  // STM32F103 小车底盘
constexpr uint8_t GROUND_ADDR    = 0x50;  // 地面站
constexpr uint8_t SERVO_ADDR     = 0x60;  // 舵机控制板（逻辑地址，用于协议层）

// 本机地址
constexpr uint8_t LOCAL_ADDR = ESP32_ADDR;

// ============================================================
// 协议版本号
// ============================================================
constexpr uint8_t PROTOCOL_VERSION = 0x01;
