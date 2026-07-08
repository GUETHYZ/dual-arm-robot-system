/**
 * @file ProtocolParser.cpp
 * @brief 协议帧解析器实现
 *
 * 状态机逐字节解析，收到完整合法帧后返回 true。
 * CRC 错误、帧尾错误时自动丢弃当前帧并回到等待帧头状态。
 */

#include "protocol/ProtocolParser.h"
#include "protocol/Crc16.h"

ProtocolParser::ProtocolParser()
    : state(State::WAIT_HEAD1)
    , payloadIndex(0)
    , receivedCrc(0)
    , crcCalcLen(0)
    , crcErrorCount(0)
    , frameErrorCount(0)
{
}

void ProtocolParser::reset() {
    state = State::WAIT_HEAD1;
    payloadIndex = 0;
    receivedCrc = 0;
    crcCalcLen = 0;
    frame = ProtocolFrame();
}

void ProtocolParser::prepareCrcCalc() {
    // 将帧头到 payload 末尾的所有字节拷贝到 CRC 计算缓冲区
    // 格式：帧头2 + 版本1 + 源1 + 目标1 + 消息类型1 + 功能码1 + 序列号1 + 长度1 + payload[N]
    crcCalcLen = 0;

    // 帧头
    crcCalcBuffer[crcCalcLen++] = FRAME_HEAD1;
    crcCalcBuffer[crcCalcLen++] = FRAME_HEAD2;
    // 各字段
    crcCalcBuffer[crcCalcLen++] = frame.version;
    crcCalcBuffer[crcCalcLen++] = frame.srcAddr;
    crcCalcBuffer[crcCalcLen++] = frame.dstAddr;
    crcCalcBuffer[crcCalcLen++] = frame.msgType;
    crcCalcBuffer[crcCalcLen++] = frame.funcCode;
    crcCalcBuffer[crcCalcLen++] = frame.seqNum;
    crcCalcBuffer[crcCalcLen++] = frame.dataLen;
    // payload
    for (uint8_t i = 0; i < frame.dataLen; i++) {
        crcCalcBuffer[crcCalcLen++] = frame.data[i];
    }
}

bool ProtocolParser::verifyCrc() {
    // 将收到的 CRC 也加入计算缓冲区
    uint8_t tempBuf[2];
    tempBuf[0] = (uint8_t)(receivedCrc & 0xFF);
    tempBuf[1] = (uint8_t)(receivedCrc >> 8);

    // 追加到现有计算缓冲区
    crcCalcBuffer[crcCalcLen] = tempBuf[0];
    crcCalcBuffer[crcCalcLen + 1] = tempBuf[1];

    // CRC-16/MODBUS: 对(含CRC在内的)全部数据计算CRC，结果应为0
    return crc16Verify(crcCalcBuffer, crcCalcLen + 2);
}

bool ProtocolParser::inputByte(uint8_t byte, ProtocolFrame& outFrame) {
    switch (state) {

    case State::WAIT_HEAD1:
        if (byte == FRAME_HEAD1) {
            state = State::WAIT_HEAD2;
        }
        // 否则保持 WAIT_HEAD1，丢弃该字节
        break;

    case State::WAIT_HEAD2:
        if (byte == FRAME_HEAD2) {
            // 帧头正确，初始化新帧
            frame = ProtocolFrame();
            payloadIndex = 0;
            receivedCrc = 0;
            state = State::READ_VERSION;
        } else {
            // 帧头第2字节错误，回到等待帧头
            state = State::WAIT_HEAD1;
            frameErrorCount++;
        }
        break;

    case State::READ_VERSION:
        frame.version = byte;
        state = State::READ_SRC;
        break;

    case State::READ_SRC:
        frame.srcAddr = byte;
        state = State::READ_DST;
        break;

    case State::READ_DST:
        frame.dstAddr = byte;
        state = State::READ_MSG_TYPE;
        break;

    case State::READ_MSG_TYPE:
        frame.msgType = byte;
        state = State::READ_FUNC;
        break;

    case State::READ_FUNC:
        frame.funcCode = byte;
        state = State::READ_SEQ;
        break;

    case State::READ_SEQ:
        frame.seqNum = byte;
        state = State::READ_LEN;
        break;

    case State::READ_LEN:
        frame.dataLen = byte;
        if (frame.dataLen > PROTOCOL_MAX_PAYLOAD) {
            // 数据长度非法，丢弃当前帧
            frameErrorCount++;
            state = State::WAIT_HEAD1;
        } else if (frame.dataLen == 0) {
            // 无 payload，直接跳到 CRC
            prepareCrcCalc();
            state = State::READ_CRC_LOW;
        } else {
            payloadIndex = 0;
            state = State::READ_PAYLOAD;
        }
        break;

    case State::READ_PAYLOAD:
        frame.data[payloadIndex++] = byte;
        if (payloadIndex >= frame.dataLen) {
            // payload 收齐，准备 CRC 计算
            prepareCrcCalc();
            state = State::READ_CRC_LOW;
        }
        break;

    case State::READ_CRC_LOW:
        receivedCrc = byte;  // 低字节在前
        state = State::READ_CRC_HIGH;
        break;

    case State::READ_CRC_HIGH:
        receivedCrc |= ((uint16_t)byte << 8);  // 高字节
        state = State::WAIT_TAIL1;
        break;

    case State::WAIT_TAIL1:
        if (byte == FRAME_TAIL1) {
            state = State::WAIT_TAIL2;
        } else {
            // 帧尾错误
            frameErrorCount++;
            state = State::WAIT_HEAD1;
        }
        break;

    case State::WAIT_TAIL2:
        if (byte == FRAME_TAIL2) {
            // 帧尾正确，验证 CRC
            if (verifyCrc()) {
                frame.valid = true;
                outFrame = frame;
                state = State::WAIT_HEAD1;
                return true;
            } else {
                // CRC 错误
                crcErrorCount++;
                state = State::WAIT_HEAD1;
            }
        } else {
            // 帧尾第2字节错误
            frameErrorCount++;
            state = State::WAIT_HEAD1;
        }
        break;

    } // switch

    return false;
}
