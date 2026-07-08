/**
 * @file DeviceConfig.h
 * @brief 设备地址、消息类型、功能码等协议常量定义
 *
 * 本文件中的常量必须与 docs/通信协议规范.md 完全一致。
 * 所有设备地址、消息类型码和功能码在此集中管理。
 */

#pragma once

#include <cstdint>
#include <cstddef>

// ============================================================================
// 协议版本
// ============================================================================

/// 当前协议版本号
constexpr uint8_t PROTOCOL_VERSION = 0x01;

/// 数据区最大长度（字节）
constexpr uint8_t PROTOCOL_MAX_PAYLOAD_LEN = 64;

// ============================================================================
// 帧格式常量
// ============================================================================

/// 帧头第一字节
constexpr uint8_t FRAME_HEAD1 = 0xAA;
/// 帧头第二字节
constexpr uint8_t FRAME_HEAD2 = 0x55;
/// 帧尾第一字节（回车）
constexpr uint8_t FRAME_TAIL1 = 0x0D;
/// 帧尾第二字节（换行）
constexpr uint8_t FRAME_TAIL2 = 0x0A;

/// 统一协议帧固定开销（帧头2 + 版本1 + 源地址1 + 目标地址1 + 消息类型1 + 功能码1 + 序列号1 + 数据长度1 + CRC2 + 帧尾2）
constexpr size_t FRAME_OVERHEAD = 13;

/// 最大帧长度（开销 + 数据区）
constexpr size_t FRAME_MAX_LEN = FRAME_OVERHEAD + PROTOCOL_MAX_PAYLOAD_LEN;

// ============================================================================
// TJC 内部事件帧常量
// ============================================================================

/// TJC 按钮事件帧头
constexpr uint8_t TJC_FRAME_HEAD = 0xDD;

/// TJC 按钮事件帧尾
constexpr uint8_t TJC_FRAME_TAIL = 0xDE;

/// TJC 按钮事件帧长度
constexpr uint8_t TJC_FRAME_LEN = 4;

/// TJC 串口屏指令结束字节（连续三个 0xFF）
constexpr uint8_t TJC_CMD_END_BYTE = 0xFF;

// ============================================================================
// 设备地址
// ============================================================================

/// 广播地址
constexpr uint8_t BROADCAST_ADDR = 0x10;

/// 车载 ESP32S3 主控地址
constexpr uint8_t ESP32_ADDR     = 0x20;

/// Jetson Nano 地址
constexpr uint8_t JETSON_ADDR    = 0x30;

/// STM32 小车底盘地址
constexpr uint8_t STM32_ADDR     = 0x40;

/// 地面站地址（本机地址）
constexpr uint8_t GROUND_ADDR    = 0x50;

/// 舵机控制板地址
constexpr uint8_t SERVO_ADDR     = 0x60;

// ============================================================================
// 消息类型
// ============================================================================

/// 命令帧
constexpr uint8_t MSG_TYPE_CMD       = 0x01;
/// 确认帧（ACK）
constexpr uint8_t MSG_TYPE_ACK       = 0x02;
/// 否认帧（NACK）
constexpr uint8_t MSG_TYPE_NACK      = 0x03;
/// 数据帧
constexpr uint8_t MSG_TYPE_DATA      = 0x04;
/// 心跳帧
constexpr uint8_t MSG_TYPE_HEARTBEAT = 0x05;

// ============================================================================
// 系统类功能码
// ============================================================================

constexpr uint8_t CMD_HEARTBEAT      = 0x01;  ///< 心跳
constexpr uint8_t CMD_ACK            = 0x02;  ///< ACK 确认
constexpr uint8_t CMD_NACK           = 0x03;  ///< NACK 否认
constexpr uint8_t CMD_STATUS_QUERY   = 0x04;  ///< 状态查询
constexpr uint8_t CMD_STATUS_REPORT  = 0x05;  ///< 状态上报
constexpr uint8_t CMD_ERROR_REPORT   = 0x06;  ///< 错误上报
constexpr uint8_t CMD_EMERGENCY_STOP = 0x07;  ///< 急停

// ============================================================================
// 底盘类功能码
// ============================================================================

constexpr uint8_t CMD_CHASSIS_FORWARD      = 0x10;  ///< 底盘前进
constexpr uint8_t CMD_CHASSIS_BACKWARD     = 0x11;  ///< 底盘后退
constexpr uint8_t CMD_CHASSIS_LEFT         = 0x12;  ///< 底盘左移
constexpr uint8_t CMD_CHASSIS_RIGHT        = 0x13;  ///< 底盘右移
constexpr uint8_t CMD_CHASSIS_ROTATE_LEFT  = 0x14;  ///< 底盘左转
constexpr uint8_t CMD_CHASSIS_ROTATE_RIGHT = 0x15;  ///< 底盘右转
constexpr uint8_t CMD_CHASSIS_STOP         = 0x16;  ///< 底盘停止
constexpr uint8_t CMD_CHASSIS_MOVE_VECTOR  = 0x17;  ///< 底盘矢量运动

// ============================================================================
// 机械臂类功能码
// ============================================================================

constexpr uint8_t CMD_ARM_HOME        = 0x20;  ///< 机械臂归零
constexpr uint8_t CMD_ARM_LEFT_PICK   = 0x21;  ///< 左臂夹取
constexpr uint8_t CMD_ARM_LEFT_PLACE  = 0x22;  ///< 左臂放置
constexpr uint8_t CMD_ARM_RIGHT_PICK  = 0x23;  ///< 右臂夹取
constexpr uint8_t CMD_ARM_RIGHT_PLACE = 0x24;  ///< 右臂放置
constexpr uint8_t CMD_ARM_BOTH_PICK   = 0x25;  ///< 双臂夹取
constexpr uint8_t CMD_ARM_BOTH_PLACE  = 0x26;  ///< 双臂放下
constexpr uint8_t CMD_ARM_STOP        = 0x27;  ///< 机械臂停止
constexpr uint8_t CMD_ARM_TORQUE_OFF  = 0x28;  ///< 舵机卸力
constexpr uint8_t CMD_ARM_TORQUE_ON   = 0x29;  ///< 舵机上力

// ============================================================================
// 视觉识别类功能码
// ============================================================================

constexpr uint8_t CMD_OBJECT_INFO = 0x30;  ///< 目标物信息

// ============================================================================
// NACK 错误码
// ============================================================================

constexpr uint8_t NACK_ERR_UNKNOWN     = 0x00;  ///< 未知错误
constexpr uint8_t NACK_ERR_CHECKSUM    = 0x01;  ///< CRC 校验错误
constexpr uint8_t NACK_ERR_INVALID_CMD = 0x02;  ///< 无效命令
constexpr uint8_t NACK_ERR_BUSY        = 0x03;  ///< 设备忙
constexpr uint8_t NACK_ERR_EMERGENCY   = 0x04;  ///< 急停状态拒绝命令
