/**
 * @file ProtocolParser.cpp
 * @brief 统一协议帧解析器——逐字节状态机实现
 *
 * 对于每个输入字节，状态机按顺序逐步解析。
 * 若某一状态检测到非法值（如数据区长度超过上限、CRC 失败、
 * 帧尾不匹配等），立即丢弃当前帧并回到 WAIT_HEAD1 重新同步。
 */

#include "protocol/ProtocolParser.h"
#include "protocol/Crc16.h"

// ============================================================================
// 构造与重置
// ============================================================================

ProtocolParser::ProtocolParser()
    : state(State::WAIT_HEAD1)
    , payloadIndex(0)
    , receivedCrc(0)
    , rawIndex(0)
    , discardCount(0)
{
}

void ProtocolParser::reset() {
    state = State::WAIT_HEAD1;
    payloadIndex = 0;
    receivedCrc = 0;
    rawIndex = 0;
}

// ============================================================================
// 逐字节输入
// ============================================================================

bool ProtocolParser::inputByte(uint8_t byte, ProtocolFrame &outFrame) {
    switch (state) {

        // --------------------------------------------------------------------
        // 等待帧头第一字节 0xAA
        // --------------------------------------------------------------------
        case State::WAIT_HEAD1:
            if (byte == FRAME_HEAD1) {
                rawBuffer[0] = byte;
                rawIndex = 1;
                state = State::WAIT_HEAD2;
            }
            // 不是 0xAA 则保持在此状态，继续等待
            break;

        // --------------------------------------------------------------------
        // 等待帧头第二字节 0x55
        // --------------------------------------------------------------------
        case State::WAIT_HEAD2:
            if (byte == FRAME_HEAD2) {
                rawBuffer[rawIndex++] = byte;
                state = State::READ_VERSION;
            } else {
                // 帧头第二字节不匹配，丢弃，可能重新与下一个 0xAA 对齐
                discardCount++;
                // 如果这一字节恰好是 0xAA，从它开始重新等待帧头
                if (byte == FRAME_HEAD1) {
                    rawBuffer[0] = byte;
                    rawIndex = 1;
                    state = State::WAIT_HEAD2;
                } else {
                    state = State::WAIT_HEAD1;
                }
            }
            break;

        // --------------------------------------------------------------------
        // 读取版本号
        // --------------------------------------------------------------------
        case State::READ_VERSION:
            rawBuffer[rawIndex++] = byte;
            frame.version = byte;
            state = State::READ_SRC;
            break;

        // --------------------------------------------------------------------
        // 读取源地址
        // --------------------------------------------------------------------
        case State::READ_SRC:
            rawBuffer[rawIndex++] = byte;
            frame.srcAddr = byte;
            state = State::READ_DST;
            break;

        // --------------------------------------------------------------------
        // 读取目标地址
        // --------------------------------------------------------------------
        case State::READ_DST:
            rawBuffer[rawIndex++] = byte;
            frame.dstAddr = byte;
            state = State::READ_MSG_TYPE;
            break;

        // --------------------------------------------------------------------
        // 读取消息类型
        // --------------------------------------------------------------------
        case State::READ_MSG_TYPE:
            rawBuffer[rawIndex++] = byte;
            frame.msgType = byte;
            state = State::READ_FUNC;
            break;

        // --------------------------------------------------------------------
        // 读取功能码
        // --------------------------------------------------------------------
        case State::READ_FUNC:
            rawBuffer[rawIndex++] = byte;
            frame.funcCode = byte;
            state = State::READ_SEQ;
            break;

        // --------------------------------------------------------------------
        // 读取序列号
        // --------------------------------------------------------------------
        case State::READ_SEQ:
            rawBuffer[rawIndex++] = byte;
            frame.seqNum = byte;
            state = State::READ_LEN;
            break;

        // --------------------------------------------------------------------
        // 读取数据长度
        // --------------------------------------------------------------------
        case State::READ_LEN:
            rawBuffer[rawIndex++] = byte;
            frame.dataLen = byte;

            if (frame.dataLen > PROTOCOL_MAX_PAYLOAD_LEN) {
                // 数据长度非法，丢弃帧
                discardCount++;
                if (byte == FRAME_HEAD1) {
                    rawBuffer[0] = byte;
                    rawIndex = 1;
                    state = State::WAIT_HEAD2;
                } else {
                    state = State::WAIT_HEAD1;
                }
                break;
            }

            payloadIndex = 0;

            if (frame.dataLen == 0) {
                // 无数据区，直接跳到 CRC
                state = State::READ_CRC_LOW;
            } else {
                state = State::READ_PAYLOAD;
            }
            break;

        // --------------------------------------------------------------------
        // 读取数据区
        // --------------------------------------------------------------------
        case State::READ_PAYLOAD:
            rawBuffer[rawIndex++] = byte;
            frame.data[payloadIndex++] = byte;

            if (payloadIndex >= frame.dataLen) {
                state = State::READ_CRC_LOW;
            }
            break;

        // --------------------------------------------------------------------
        // 读取 CRC 低字节
        // --------------------------------------------------------------------
        case State::READ_CRC_LOW:
            rawBuffer[rawIndex++] = byte;
            receivedCrc = byte;  // 低字节先存
            state = State::READ_CRC_HIGH;
            break;

        // --------------------------------------------------------------------
        // 读取 CRC 高字节
        // --------------------------------------------------------------------
        case State::READ_CRC_HIGH:
            rawBuffer[rawIndex++] = byte;
            receivedCrc |= (static_cast<uint16_t>(byte) << 8);
            state = State::WAIT_TAIL1;
            break;

        // --------------------------------------------------------------------
        // 等待帧尾第一字节 0x0D
        // --------------------------------------------------------------------
        case State::WAIT_TAIL1:
            if (byte == FRAME_TAIL1) {
                state = State::WAIT_TAIL2;
            } else {
                // 帧尾不匹配，CRC 错误或数据损坏
                discardCount++;
                if (byte == FRAME_HEAD1) {
                    rawBuffer[0] = byte;
                    rawIndex = 1;
                    state = State::WAIT_HEAD2;
                } else {
                    state = State::WAIT_HEAD1;
                }
            }
            break;

        // --------------------------------------------------------------------
        // 等待帧尾第二字节 0x0A → 帧解析完成
        // --------------------------------------------------------------------
        case State::WAIT_TAIL2:
            if (byte == FRAME_TAIL2) {
                // 验证 CRC
                // crcRange = rawBuffer 中从帧头到数据区末尾的长度
                size_t crcRange = rawIndex - 2; // 减去已写入的两个 CRC 字节
                uint8_t crcLow = static_cast<uint8_t>(receivedCrc & 0xFF);
                uint8_t crcHigh = static_cast<uint8_t>((receivedCrc >> 8) & 0xFF);

                if (crc16Verify(rawBuffer, crcRange, crcLow, crcHigh)) {
                    // CRC 通过，输出帧
                    outFrame = frame;
                    reset();
                    return true;
                } else {
                    // CRC 校验失败
                    discardCount++;
                    reset();
                    return false;
                }
            } else {
                // 帧尾第二字节不匹配
                discardCount++;
                if (byte == FRAME_HEAD1) {
                    rawBuffer[0] = byte;
                    rawIndex = 1;
                    state = State::WAIT_HEAD2;
                } else {
                    state = State::WAIT_HEAD1;
                }
            }
            break;
    }

    return false;
}
