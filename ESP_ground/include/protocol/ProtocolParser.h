/**
 * @file ProtocolParser.h
 * @brief 统一协议帧解析器——逐字节状态机
 *
 * 每个 LoRa 接收通道应拥有一个独立的 ProtocolParser 实例。
 * 使用逐字节输入 + 状态机的方式解析，不使用 readString，
 * 不阻塞，不使用动态内存。
 *
 * 工作流程：
 *   1. 调用 inputByte() 逐字节喂入数据；
 *   2. 状态机自动跟踪解析进度；
 *   3. 当完整帧解析成功（CRC 通过、帧尾匹配），返回 true；
 *   4. 通过 outFrame 获取解析后的 ProtocolFrame。
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "Protocol.h"
#include "config/DeviceConfig.h"

class ProtocolParser {
public:
    /**
     * @brief 解析器状态枚举
     */
    enum class State {
        WAIT_HEAD1,       ///< 等待帧头第 1 字节 (0xAA)
        WAIT_HEAD2,       ///< 等待帧头第 2 字节 (0x55)
        READ_VERSION,     ///< 读取版本号
        READ_SRC,         ///< 读取源地址
        READ_DST,         ///< 读取目标地址
        READ_MSG_TYPE,    ///< 读取消息类型
        READ_FUNC,        ///< 读取功能码
        READ_SEQ,         ///< 读取序列号
        READ_LEN,         ///< 读取数据长度
        READ_PAYLOAD,     ///< 读取数据区
        READ_CRC_LOW,     ///< 读取 CRC 低字节
        READ_CRC_HIGH,    ///< 读取 CRC 高字节
        WAIT_TAIL1,       ///< 等待帧尾第 1 字节 (0x0D)
        WAIT_TAIL2        ///< 等待帧尾第 2 字节 (0x0A)
    };

    // ========================================================================
    // 原始缓冲区大小
    // ========================================================================

    /// 原始帧缓冲区：帧头2 + 版本1 + 源1 + 目标1 + 类型1 + 功能1 + 序列1 + 长度1 + 数据64 + CRC2
    static constexpr size_t RAW_BUF_SIZE = FRAME_OVERHEAD + PROTOCOL_MAX_PAYLOAD_LEN;

    // ========================================================================
    // 构造与重置
    // ========================================================================

    ProtocolParser();

    /**
     * @brief 重置解析器状态（用于错误恢复）
     */
    void reset();

    // ========================================================================
    // 核心解析接口
    // ========================================================================

    /**
     * @brief 输入一个字节到解析器
     *
     * 每收到一个字节就调用一次。当解析出完整合法的帧时返回 true。
     *
     * @param byte     输入的一个字节
     * @param outFrame 输出解析后的协议帧（仅返回 true 时有效）
     * @return true  完整帧解析成功
     * @return false 还需更多字节或帧被丢弃
     */
    bool inputByte(uint8_t byte, ProtocolFrame &outFrame);

    // ========================================================================
    // 状态查询
    // ========================================================================

    /// 获取当前解析状态
    State getState() const { return state; }

    /// 统计丢弃帧数（用于调试）
    uint32_t getDiscardCount() const { return discardCount; }

private:
    State state;                        ///< 当前状态
    ProtocolFrame frame;                ///< 正在构建的帧
    uint8_t payloadIndex;               ///< 数据区当前写入位置
    uint16_t receivedCrc;               ///< 接收到的 CRC 值
    uint8_t rawBuffer[RAW_BUF_SIZE];    ///< 原始帧缓冲区（用于重算 CRC）
    uint8_t rawIndex;                   ///< 原始帧缓冲区当前写入位置
    uint32_t discardCount;              ///< 丢弃帧计数
};
