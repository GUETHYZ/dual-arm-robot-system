/**
 * @file Protocol.h
 * @brief 统一通信协议定义
 *
 * 帧格式：
 *   帧头(2) + 版本(1) + 源地址(1) + 目标地址(1) + 消息类型(1) + 功能码(1)
 *   + 序列号(1) + 数据长度(1) + 数据区(0~64) + CRC16(2, 低字节在前) + 帧尾(2)
 *
 * 设备地址：
 *   BROADCAST = 0x10, ESP32 = 0x20, JETSON = 0x30,
 *   STM32 = 0x40, GROUND = 0x50, SERVO = 0x60
 */

#pragma once

#include <Arduino.h>
#include "config/TaskConfig.h"

// ============================================================
// 帧边界常量
// ============================================================
constexpr uint8_t  FRAME_HEAD1   = 0xAA;  // 帧头第1字节
constexpr uint8_t  FRAME_HEAD2   = 0x55;  // 帧头第2字节
constexpr uint8_t  FRAME_TAIL1   = 0x0D;  // 帧尾第1字节
constexpr uint8_t  FRAME_TAIL2   = 0x0A;  // 帧尾第2字节

// 帧固定开销：帧头(2) + 版本(1) + 源(1) + 目标(1) + 消息类型(1) + 功能码(1)
//            + 序列号(1) + 长度(1) + CRC(2) + 帧尾(2) = 13 字节
constexpr uint8_t FRAME_OVERHEAD = 13;

// ============================================================
// 消息类型
// ============================================================
namespace MsgType {
    constexpr uint8_t CMD       = 0x01;  // 命令
    constexpr uint8_t ACK       = 0x02;  // 确认应答
    constexpr uint8_t NACK      = 0x03;  // 否认应答
    constexpr uint8_t DATA      = 0x04;  // 数据/上报
    constexpr uint8_t HEARTBEAT = 0x05;  // 心跳
}

// ============================================================
// 功能码
// ============================================================
namespace FuncCode {

    // --- 系统类 (0x00 ~ 0x0F) ---
    constexpr uint8_t HEARTBEAT       = 0x01;  // 心跳
    constexpr uint8_t ACK             = 0x02;  // 确认
    constexpr uint8_t NACK            = 0x03;  // 否认
    constexpr uint8_t STATUS_QUERY    = 0x04;  // 状态查询
    constexpr uint8_t STATUS_REPORT   = 0x05;  // 状态上报
    constexpr uint8_t ERROR_REPORT    = 0x06;  // 错误上报
    constexpr uint8_t EMERGENCY_STOP  = 0x07;  // 急停

    // --- 小车底盘类 (0x10 ~ 0x1F) ---
    constexpr uint8_t CHASSIS_FORWARD      = 0x10;  // 前进
    constexpr uint8_t CHASSIS_BACKWARD     = 0x11;  // 后退
    constexpr uint8_t CHASSIS_LEFT         = 0x12;  // 左移
    constexpr uint8_t CHASSIS_RIGHT        = 0x13;  // 右移
    constexpr uint8_t CHASSIS_ROTATE_LEFT  = 0x14;  // 左转
    constexpr uint8_t CHASSIS_ROTATE_RIGHT = 0x15;  // 右转
    constexpr uint8_t CHASSIS_STOP         = 0x16;  // 停止
    constexpr uint8_t CHASSIS_MOVE_VECTOR  = 0x17;  // 矢量移动（数据区含角度和速度）

    // --- 机械臂类 (0x20 ~ 0x2F) ---
    constexpr uint8_t ARM_HOME        = 0x20;  // 双臂复位
    constexpr uint8_t ARM_LEFT_PICK   = 0x21;  // 左臂夹取
    constexpr uint8_t ARM_LEFT_PLACE  = 0x22;  // 左臂放下
    constexpr uint8_t ARM_RIGHT_PICK  = 0x23;  // 右臂夹取
    constexpr uint8_t ARM_RIGHT_PLACE = 0x24;  // 右臂放下
    constexpr uint8_t ARM_BOTH_PICK   = 0x25;  // 双臂夹取
    constexpr uint8_t ARM_BOTH_PLACE  = 0x26;  // 双臂放下
    constexpr uint8_t ARM_STOP        = 0x27;  // 停止所有舵机
    constexpr uint8_t ARM_TORQUE_OFF  = 0x28;  // 释放舵机扭力
    constexpr uint8_t ARM_TORQUE_ON   = 0x29;  // 恢复舵机扭力

    // --- 视觉识别类 (0x30 ~ 0x3F) ---
    constexpr uint8_t OBJECT_INFO = 0x30;  // 物体识别信息（Jetson → ESP32）
}

// ============================================================
// 物体类型（CMD_OBJECT_INFO 数据区）
// ============================================================
namespace ObjectType {
    constexpr uint8_t CARGO_A  = 0x01;  // 货物 A
    constexpr uint8_t CARGO_B  = 0x02;  // 货物 B
    constexpr uint8_t UNKNOWN  = 0x03;  // 未知货物
}

// ============================================================
// 错误码
// ============================================================
namespace ErrorCode {
    constexpr uint8_t NONE              = 0x00;  // 无错误
    constexpr uint8_t CRC_ERROR         = 0x01;  // CRC 校验错误
    constexpr uint8_t FRAME_ERROR       = 0x02;  // 帧格式错误
    constexpr uint8_t QUEUE_FULL        = 0x03;  // 队列满
    constexpr uint8_t UNKNOWN_CMD       = 0x04;  // 未知命令
    constexpr uint8_t SERVO_ERROR       = 0x05;  // 舵机错误
    constexpr uint8_t TIMEOUT           = 0x06;  // 超时
    constexpr uint8_t EMERGENCY_ACTIVE  = 0x07;  // 急停激活中
    constexpr uint8_t INVALID_PARAM     = 0x08;  // 参数非法
}

// ============================================================
// 协议帧结构体
// ============================================================
struct ProtocolFrame {
    uint8_t version;                            // 协议版本
    uint8_t srcAddr;                            // 源设备地址
    uint8_t dstAddr;                            // 目标设备地址
    uint8_t msgType;                            // 消息类型
    uint8_t funcCode;                           // 功能码
    uint8_t seqNum;                             // 序列号
    uint8_t dataLen;                            // 数据长度 (0~64)
    uint8_t data[PROTOCOL_MAX_PAYLOAD];         // 数据区
    bool    valid;                              // 帧是否有效（解析结果）

    ProtocolFrame() : version(0), srcAddr(0), dstAddr(0), msgType(0),
                      funcCode(0), seqNum(0), dataLen(0), valid(false) {
        memset(data, 0, sizeof(data));
    }
};

// ============================================================
// 协议辅助函数声明
// ============================================================

/**
 * @brief 将 ProtocolFrame 打包为二进制数据，写入缓冲区
 * @param frame  待打包的协议帧（需预先填充各字段）
 * @param buffer 输出缓冲区
 * @param bufSize 缓冲区大小
 * @return 打包后的字节数，0 表示失败
 */
size_t protocolPack(const ProtocolFrame& frame, uint8_t* buffer, size_t bufSize);

/**
 * @brief 快速构建 ACK 帧
 * @param srcAddr 本机地址
 * @param dstAddr 目标地址
 * @param seqNum  确认的序列号
 * @param out     输出帧
 */
void buildAckFrame(uint8_t srcAddr, uint8_t dstAddr, uint8_t seqNum,
                   ProtocolFrame& out);

/**
 * @brief 快速构建 NACK 帧
 * @param srcAddr  本机地址
 * @param dstAddr  目标地址
 * @param seqNum   序列号
 * @param errorCode 错误码
 * @param out      输出帧
 */
void buildNackFrame(uint8_t srcAddr, uint8_t dstAddr, uint8_t seqNum,
                    uint8_t errorCode, ProtocolFrame& out);

/**
 * @brief 快速构建心跳帧
 * @param out 输出帧
 */
void buildHeartbeatFrame(ProtocolFrame& out);

/**
 * @brief 快速构建状态上报帧
 * @param statusData 状态数据（最多 8 字节）
 * @param len        状态数据长度
 * @param out        输出帧
 */
void buildStatusReportFrame(const uint8_t* statusData, uint8_t len,
                            ProtocolFrame& out);

/**
 * @brief 检查功能码是否为底盘类命令
 */
bool isChassisCommand(uint8_t funcCode);

/**
 * @brief 检查功能码是否为机械臂类命令
 */
bool isArmCommand(uint8_t funcCode);

/**
 * @brief 检查功能码是否为系统类命令
 */
bool isSystemCommand(uint8_t funcCode);
