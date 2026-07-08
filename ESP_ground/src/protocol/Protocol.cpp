/**
 * @file Protocol.cpp
 * @brief 统一通信协议——帧打包、序列号管理、工具函数
 */

#include "protocol/Protocol.h"
#include "protocol/Crc16.h"

// ============================================================================
// 序列号管理
// ============================================================================

/// 全局序列号，0~255 滚动递增
static uint8_t gSeqNum = 0;

uint8_t nextSeq() {
    return gSeqNum++;
}

// ============================================================================
// 帧打包
// ============================================================================

bool packFrame(const ProtocolFrame &frame, uint8_t *outBuf, size_t outSize, size_t &outLen) {
    // 计算完整帧长度
    size_t totalLen = FRAME_OVERHEAD + frame.dataLen;

    // 检查缓冲区大小
    if (outSize < totalLen) {
        return false;
    }

    // 检查数据长度合法性
    if (frame.dataLen > PROTOCOL_MAX_PAYLOAD_LEN) {
        return false;
    }

    uint8_t *p = outBuf;

    // 帧头
    *p++ = FRAME_HEAD1;     // 0xAA
    *p++ = FRAME_HEAD2;     // 0x55

    // 协议字段
    *p++ = frame.version;   // 版本号
    *p++ = frame.srcAddr;   // 源地址
    *p++ = frame.dstAddr;   // 目标地址
    *p++ = frame.msgType;   // 消息类型
    *p++ = frame.funcCode;  // 功能码
    *p++ = frame.seqNum;    // 序列号
    *p++ = frame.dataLen;   // 数据长度

    // 数据区
    for (uint8_t i = 0; i < frame.dataLen; i++) {
        *p++ = frame.data[i];
    }

    // 计算 CRC16（从帧头开始到数据区末尾）
    size_t crcRange = (p - outBuf);
    uint16_t crc = crc16Modbus(outBuf, crcRange);

    // CRC 低字节在前
    *p++ = static_cast<uint8_t>(crc & 0xFF);        // CRC 低字节
    *p++ = static_cast<uint8_t>((crc >> 8) & 0xFF); // CRC 高字节

    // 帧尾
    *p++ = FRAME_TAIL1;     // 0x0D
    *p++ = FRAME_TAIL2;     // 0x0A

    outLen = static_cast<size_t>(p - outBuf);
    return true;
}

// ============================================================================
// 地址过滤
// ============================================================================

bool isFrameForGround(const ProtocolFrame &frame) {
    return (frame.dstAddr == GROUND_ADDR) || (frame.dstAddr == BROADCAST_ADDR);
}

// ============================================================================
// 消息类型字符串
// ============================================================================

const char* msgTypeToString(uint8_t msgType) {
    switch (msgType) {
        case MSG_TYPE_CMD:       return "CMD";
        case MSG_TYPE_ACK:       return "ACK";
        case MSG_TYPE_NACK:      return "NACK";
        case MSG_TYPE_DATA:      return "DATA";
        case MSG_TYPE_HEARTBEAT: return "HEARTBEAT";
        default:                 return "UNKNOWN";
    }
}

// ============================================================================
// 功能码字符串
// ============================================================================

const char* funcCodeToString(uint8_t funcCode) {
    switch (funcCode) {
        // 系统类
        case CMD_HEARTBEAT:      return "HEARTBEAT";
        case CMD_ACK:            return "ACK";
        case CMD_NACK:           return "NACK";
        case CMD_STATUS_QUERY:   return "STATUS_QUERY";
        case CMD_STATUS_REPORT:  return "STATUS_REPORT";
        case CMD_ERROR_REPORT:   return "ERROR_REPORT";
        case CMD_EMERGENCY_STOP: return "EMERGENCY_STOP";

        // 底盘类
        case CMD_CHASSIS_FORWARD:      return "CHASSIS_FORWARD";
        case CMD_CHASSIS_BACKWARD:     return "CHASSIS_BACKWARD";
        case CMD_CHASSIS_LEFT:         return "CHASSIS_LEFT";
        case CMD_CHASSIS_RIGHT:        return "CHASSIS_RIGHT";
        case CMD_CHASSIS_ROTATE_LEFT:  return "CHASSIS_ROTATE_LEFT";
        case CMD_CHASSIS_ROTATE_RIGHT: return "CHASSIS_ROTATE_RIGHT";
        case CMD_CHASSIS_STOP:         return "CHASSIS_STOP";
        case CMD_CHASSIS_MOVE_VECTOR:  return "CHASSIS_MOVE_VECTOR";

        // 机械臂类
        case CMD_ARM_HOME:        return "ARM_HOME";
        case CMD_ARM_LEFT_PICK:   return "ARM_LEFT_PICK";
        case CMD_ARM_LEFT_PLACE:  return "ARM_LEFT_PLACE";
        case CMD_ARM_RIGHT_PICK:  return "ARM_RIGHT_PICK";
        case CMD_ARM_RIGHT_PLACE: return "ARM_RIGHT_PLACE";
        case CMD_ARM_BOTH_PICK:   return "ARM_BOTH_PICK";
        case CMD_ARM_BOTH_PLACE:  return "ARM_BOTH_PLACE";
        case CMD_ARM_STOP:        return "ARM_STOP";
        case CMD_ARM_TORQUE_OFF:  return "ARM_TORQUE_OFF";
        case CMD_ARM_TORQUE_ON:   return "ARM_TORQUE_ON";

        // 视觉识别类
        case CMD_OBJECT_INFO:     return "OBJECT_INFO";

        default:                  return "UNKNOWN";
    }
}
