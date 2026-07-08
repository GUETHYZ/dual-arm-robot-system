/**
 * @file Protocol.h
 * @brief 统一通信协议——帧结构、打包、序列号管理
 *
 * 本文件定义了地面站与车载 ESP32S3 之间的统一通信协议帧结构，
 * 以及帧打包、序列号管理、地址过滤等功能。
 *
 * 协议常量定义在 include/config/DeviceConfig.h 中。
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "config/DeviceConfig.h"

// ============================================================================
// 协议帧结构体
// ============================================================================

/**
 * @brief 统一通信协议帧
 *
 * 帧格式（字节序）：
 *   Byte 0:   帧头1 = 0xAA
 *   Byte 1:   帧头2 = 0x55
 *   Byte 2:   版本号
 *   Byte 3:   源地址
 *   Byte 4:   目标地址
 *   Byte 5:   消息类型
 *   Byte 6:   功能码
 *   Byte 7:   序列号
 *   Byte 8:   数据长度
 *   Byte 9~N: 数据区（0~64 字节）
 *   之后 2 字节: CRC16（低字节在前）
 *   最后 2 字节: 帧尾 0x0D 0x0A
 */
struct ProtocolFrame {
    uint8_t version;                                    ///< 协议版本号
    uint8_t srcAddr;                                    ///< 源设备地址
    uint8_t dstAddr;                                    ///< 目标设备地址
    uint8_t msgType;                                    ///< 消息类型
    uint8_t funcCode;                                   ///< 功能码
    uint8_t seqNum;                                     ///< 序列号
    uint8_t dataLen;                                    ///< 数据区实际长度
    uint8_t data[PROTOCOL_MAX_PAYLOAD_LEN];             ///< 数据区
};

// ============================================================================
// 协议工具函数
// ============================================================================

/**
 * @brief 获取下一个发送序列号（0~255 滚动递增）
 * @return 当前序列号
 */
uint8_t nextSeq();

/**
 * @brief 将 ProtocolFrame 打包为发送缓冲区
 *
 * @param frame   要打包的协议帧
 * @param outBuf  输出缓冲区指针
 * @param outSize 输出缓冲区大小
 * @param outLen  输出实际打包长度
 * @return true 打包成功
 * @return false 打包失败（缓冲区不足等）
 */
bool packFrame(const ProtocolFrame &frame, uint8_t *outBuf, size_t outSize, size_t &outLen);

/**
 * @brief 判断帧是否属于地面站（目标地址匹配）
 *
 * 地面站只处理目标地址为 GROUND_ADDR 或 BROADCAST_ADDR 的帧。
 *
 * @param frame 接收到的协议帧
 * @return true  该帧应被地面站处理
 * @return false 该帧应丢弃
 */
bool isFrameForGround(const ProtocolFrame &frame);

/**
 * @brief 将消息类型转换为可读字符串
 * @param msgType 消息类型码
 * @return 消息类型字符串
 */
const char* msgTypeToString(uint8_t msgType);

/**
 * @brief 将功能码转换为可读字符串
 * @param funcCode 功能码
 * @return 功能码字符串
 */
const char* funcCodeToString(uint8_t funcCode);

/**
 * @brief 计算协议帧所需缓冲区大小
 * @param dataLen 数据区长度
 * @return 完整帧的字节数
 */
constexpr size_t frameBufferSize(uint8_t dataLen) {
    return FRAME_OVERHEAD + dataLen;
}
