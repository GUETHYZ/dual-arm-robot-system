/**
 * @file Protocol.cpp
 * @brief 协议辅助函数实现
 *
 * 包括帧打包、ACK/NACK/心跳/状态帧构建等。
 */

#include "protocol/Protocol.h"
#include "protocol/Crc16.h"
#include "config/DeviceConfig.h"

size_t protocolPack(const ProtocolFrame& frame, uint8_t* buffer, size_t bufSize) {
    size_t totalLen = FRAME_OVERHEAD + frame.dataLen;
    if (bufSize < totalLen) {
        return 0;  // 缓冲区不足
    }

    size_t pos = 0;

    // 帧头
    buffer[pos++] = FRAME_HEAD1;
    buffer[pos++] = FRAME_HEAD2;
    // 协议字段
    buffer[pos++] = frame.version;
    buffer[pos++] = frame.srcAddr;
    buffer[pos++] = frame.dstAddr;
    buffer[pos++] = frame.msgType;
    buffer[pos++] = frame.funcCode;
    buffer[pos++] = frame.seqNum;
    buffer[pos++] = frame.dataLen;
    // payload
    for (uint8_t i = 0; i < frame.dataLen; i++) {
        buffer[pos++] = frame.data[i];
    }

    // CRC16（低字节在前）
    uint16_t crc = crc16Compute(buffer, pos);
    buffer[pos++] = (uint8_t)(crc & 0xFF);
    buffer[pos++] = (uint8_t)(crc >> 8);

    // 帧尾
    buffer[pos++] = FRAME_TAIL1;
    buffer[pos++] = FRAME_TAIL2;

    return pos;
}

void buildAckFrame(uint8_t srcAddr, uint8_t dstAddr, uint8_t seqNum,
                   ProtocolFrame& out) {
    out = ProtocolFrame();
    out.version  = PROTOCOL_VERSION;
    out.srcAddr  = srcAddr;
    out.dstAddr  = dstAddr;
    out.msgType  = MsgType::ACK;
    out.funcCode = FuncCode::ACK;
    out.seqNum   = seqNum;
    out.dataLen  = 0;
    out.valid    = true;
}

void buildNackFrame(uint8_t srcAddr, uint8_t dstAddr, uint8_t seqNum,
                    uint8_t errorCode, ProtocolFrame& out) {
    out = ProtocolFrame();
    out.version  = PROTOCOL_VERSION;
    out.srcAddr  = srcAddr;
    out.dstAddr  = dstAddr;
    out.msgType  = MsgType::NACK;
    out.funcCode = FuncCode::NACK;
    out.seqNum   = seqNum;
    out.dataLen  = 1;
    out.data[0]  = errorCode;
    out.valid    = true;
}

void buildHeartbeatFrame(ProtocolFrame& out) {
    out = ProtocolFrame();
    out.version  = PROTOCOL_VERSION;
    out.srcAddr  = LOCAL_ADDR;
    out.dstAddr  = GROUND_ADDR;
    out.msgType  = MsgType::HEARTBEAT;
    out.funcCode = FuncCode::HEARTBEAT;
    out.seqNum   = 0;
    out.dataLen  = 0;
    out.valid    = true;
}

void buildStatusReportFrame(const uint8_t* statusData, uint8_t len,
                            ProtocolFrame& out) {
    out = ProtocolFrame();
    out.version  = PROTOCOL_VERSION;
    out.srcAddr  = LOCAL_ADDR;
    out.dstAddr  = GROUND_ADDR;
    out.msgType  = MsgType::DATA;
    out.funcCode = FuncCode::STATUS_REPORT;
    out.seqNum   = 0;
    out.dataLen  = (len > PROTOCOL_MAX_PAYLOAD) ? PROTOCOL_MAX_PAYLOAD : len;
    if (len > 0 && statusData != nullptr) {
        memcpy(out.data, statusData, out.dataLen);
    }
    out.valid = true;
}

bool isChassisCommand(uint8_t funcCode) {
    return (funcCode >= 0x10 && funcCode <= 0x1F);
}

bool isArmCommand(uint8_t funcCode) {
    return (funcCode >= 0x20 && funcCode <= 0x2F);
}

bool isSystemCommand(uint8_t funcCode) {
    return (funcCode >= 0x00 && funcCode <= 0x0F);
}
