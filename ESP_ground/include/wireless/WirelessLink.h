/**
 * @file WirelessLink.h
 * @brief LoRa 无线串口通信模块
 *
 * 封装 LoRa 无线串口（Serial1）的收发逻辑。
 * 发送使用协议打包，接收使用 ProtocolParser 状态机。
 *
 * 线程安全：
 *   - 发送使用互斥锁（loraTxMutex），多任务可安全调用；
 *   - 接收使用独立的 ProtocolParser 实例，每个通道一个解析器。
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>
#include "config/HardwareConfig.h"
#include "config/DeviceConfig.h"
#include "protocol/Protocol.h"
#include "protocol/ProtocolParser.h"

// ============================================================================
// 前向声明
// ============================================================================

extern SemaphoreHandle_t loraTxMutex;

class WirelessLink {
public:
    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化 LoRa 无线串口
     *
     * 配置 Serial1 引脚、波特率，并创建互斥锁。
     */
    static void begin();

    // ========================================================================
    // 发送
    // ========================================================================

    /**
     * @brief 发送协议帧（线程安全）
     *
     * 先将 ProtocolFrame 打包为二进制缓冲区，再通过 Serial1 发送。
     * 使用 loraTxMutex 保护串口访问。
     *
     * @param frame 待发送的协议帧
     * @return true  发送成功
     * @return false 打包或发送失败
     */
    static bool sendFrame(const ProtocolFrame &frame);

    // ========================================================================
    // 接收
    // ========================================================================

    /**
     * @brief 轮询接收协议帧（非阻塞）
     *
     * 从 Serial1 读取可用字节，通过 ProtocolParser 解析。
     * 单次最多处理 LORA_RX_MAX_BYTES_PER_CYCLE 字节，
     * 避免在接收回调中长时间占用 CPU。
     *
     * @param frame 输出解析后的协议帧（仅返回 true 时有效）
     * @return true  接收到一个完整合法的协议帧
     * @return false 无新帧（或帧被丢弃）
     */
    static bool pollFrame(ProtocolFrame &frame);

    /**
     * @brief 获取协议解析器引用（用于查询丢弃帧数等）
     */
    static ProtocolParser& getParser() { return parser; }

    /**
     * @brief 检查 LoRa 串口是否已初始化
     */
    static bool isReady() { return initialized; }

private:
    static ProtocolParser parser;   ///< 协议解析器实例
    static bool initialized;        ///< 初始化标志
};
