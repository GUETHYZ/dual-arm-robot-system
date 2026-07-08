/**
 * @file ProtocolParser.h
 * @brief 协议帧解析器（状态机实现）
 *
 * 每个串口通道拥有独立的 ProtocolParser 实例，逐字节输入，
 * 内部使用状态机解析帧头、帧尾、CRC 等字段。
 * 收到完整合法帧后返回 true，调用者可读取 outFrame。
 *
 * 特点：
 * - 不使用动态内存
 * - 固定长度缓冲区
 * - CRC 错误、帧尾错误时自动丢弃并回到等待帧头状态
 * - 最大 payload 64 字节
 */

#pragma once

#include <Arduino.h>
#include "protocol/Protocol.h"

class ProtocolParser {
public:
    ProtocolParser();

    /**
     * @brief 输入一个字节进行解析
     * @param byte     输入字节
     * @param outFrame 输出帧（仅在返回 true 时有效）
     * @return true = 收到完整合法帧
     */
    bool inputByte(uint8_t byte, ProtocolFrame& outFrame);

    /**
     * @brief 重置解析器状态
     */
    void reset();

    /**
     * @brief 获取 CRC 错误计数
     */
    uint32_t getCrcErrorCount() const { return crcErrorCount; }

    /**
     * @brief 获取帧尾错误计数
     */
    uint32_t getFrameErrorCount() const { return frameErrorCount; }

private:
    enum class State : uint8_t {
        WAIT_HEAD1,      // 等待帧头第1字节 0xAA
        WAIT_HEAD2,      // 等待帧头第2字节 0x55
        READ_VERSION,    // 读取协议版本
        READ_SRC,        // 读取源地址
        READ_DST,        // 读取目标地址
        READ_MSG_TYPE,   // 读取消息类型
        READ_FUNC,       // 读取功能码
        READ_SEQ,        // 读取序列号
        READ_LEN,        // 读取数据长度
        READ_PAYLOAD,    // 读取数据区
        READ_CRC_LOW,    // 读取 CRC 低字节
        READ_CRC_HIGH,   // 读取 CRC 高字节
        WAIT_TAIL1,      // 等待帧尾第1字节 0x0D
        WAIT_TAIL2,      // 等待帧尾第2字节 0x0A
    };

    State     state;            // 当前状态
    ProtocolFrame frame;       // 正在构建的帧
    uint8_t   payloadIndex;    // payload 已读取位置
    uint16_t  receivedCrc;     // 收到的 CRC 值
    uint8_t   crcCalcBuffer[PROTOCOL_FRAME_MAX]; // CRC 计算缓冲区
    uint8_t   crcCalcLen;      // CRC 计算数据长度

    // 错误统计
    uint32_t  crcErrorCount;
    uint32_t  frameErrorCount;

    /**
     * @brief 将已收齐的帧头字段拷贝到 CRC 计算缓冲区
     */
    void prepareCrcCalc();

    /**
     * @brief 验证收到的 CRC 与计算的 CRC 是否一致
     */
    bool verifyCrc();
};
