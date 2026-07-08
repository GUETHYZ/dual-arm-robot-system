/**
 * @file CommandMapper.h
 * @brief TJC 按钮命令码到统一协议功能码的映射模块
 *
 * 功能：
 *   1. 将 TJC 按钮事件命令码映射为统一协议功能码
 *   2. 提供 TJC cmd / 功能码 → 可读名称的转换
 *   3. 提供 GroundCommand 结构体用于队列传递
 *
 * GroundCommand 是发送队列的元素类型，
 * 包含功能码、可选数据区和数据长度。
 */

#pragma once

#include <cstdint>
#include "config/DeviceConfig.h"

// ============================================================================
// 发送队列元素
// ============================================================================

/**
 * @brief 地面站发出的控制命令（队列元素）
 *
 * funcCode: 统一协议功能码
 * data:     可选数据区
 * dataLen:  数据区长度（0 表示无数据）
 */
struct GroundCommand {
    uint8_t funcCode;
    uint8_t data[PROTOCOL_MAX_PAYLOAD_LEN];
    uint8_t dataLen;
};

// ============================================================================
// CommandMapper 类
// ============================================================================

class CommandMapper {
public:
    /**
     * @brief 将 TJC 按钮命令码映射为 GroundCommand
     *
     * @param tjcCmd TJC 按钮事件命令码（DD CMD CMD DE 中的 CMD）
     * @param outCmd 输出的 GroundCommand
     * @return true  映射成功
     * @return false 无效的 TJC 命令码
     */
    static bool mapTjcCommand(uint8_t tjcCmd, GroundCommand &outCmd);

    /**
     * @brief 将 TJC 命令码转换为可读名称
     *
     * @param tjcCmd TJC 按钮事件命令码
     * @return 名称字符串（如 "b_forward"），未知则返回 "UNKNOWN"
     */
    static const char* tjcCmdToName(uint8_t tjcCmd);

    /**
     * @brief 将统一协议功能码转换为可读名称
     *
     * 委托给 Protocol::funcCodeToString。
     *
     * @param funcCode 统一协议功能码
     * @return 名称字符串
     */
    static const char* funcCodeToName(uint8_t funcCode);
};
