/**
 * @file TjcPanel.h
 * @brief 陶晶驰 TJC 串口屏驱动模块
 *
 * 功能：
 *   1. 初始化 TJC 串口（Serial2）
 *   2. 解析 TJC 按钮事件帧（DD CMD CMD DE）
 *   3. 向 TJC 发送原始字符串指令
 *   4. 设置 TJC 文本控件文本
 *   5. 预留 UI 状态刷新接口
 *
 * TJC 按钮事件帧格式：
 *   字节0: 0xDD（帧头）
 *   字节1: 命令码
 *   字节2: 命令码（重复校验）
 *   字节3: 0xDE（帧尾）
 *
 * 发送给 TJC 的指令需要追加 0xFF 0xFF 0xFF 结束。
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include <Arduino.h>
#include "config/HardwareConfig.h"
#include "config/DeviceConfig.h"

class TjcPanel {
public:
    // ========================================================================
    // 初始化
    // ========================================================================

    /**
     * @brief 初始化 TJC 串口屏
     *
     * 配置 Serial2 引脚和波特率，清空缓冲区。
     */
    static void begin();

    // ========================================================================
    // 按钮事件轮询
    // ========================================================================

    /**
     * @brief 轮询 TJC 按钮事件（非阻塞）
     *
     * 每次调用尝试从 Serial2 接收缓冲区读取数据，
     * 使用状态机解析 TJC 按钮事件帧。
     *
     * @param cmd 输出解析出的按钮命令码（仅返回 true 时有效）
     * @return true  解析出一个合法的按钮事件
     * @return false 无新事件（或帧被丢弃）
     */
    static bool pollButtonEvent(uint8_t &cmd);

    // ========================================================================
    // 向 TJC 发送指令
    // ========================================================================

    /**
     * @brief 向 TJC 发送原始字符串指令
     *
     * 自动在指令末尾追加 0xFF 0xFF 0xFF。
     *
     * @param cmd 以 null 结尾的指令字符串
     */
    static void sendRaw(const char *cmd);

    /**
     * @brief 设置 TJC 文本控件的文本内容
     *
     * 使用 TJC 的 t0.txt=\"...\" 或 val=\"...\" 语法。
     *
     * @param component TJC 控件名称（如 "t0"）
     * @param text      要显示的文本内容
     */
    static void setText(const char *component, const char *text);

    // ========================================================================
    // 预留 UI 刷新
    // ========================================================================

    /**
     * @brief 刷新预留的状态显示控件（当前版本为空实现）
     *
     * 后续可在此添加对 TJC 文本控件的批量刷新逻辑。
     * 例如更新连接状态、心跳计数、物体识别结果等。
     */
    static void refreshReservedStatus();

    // ========================================================================
    // 调试
    // ========================================================================

    /// 获取丢弃帧计数
    static uint32_t getDiscardCount() { return discardCount; }

private:
    // ========================================================================
    // TJC 按钮帧解析状态机状态
    // ========================================================================
    enum class ParseState {
        WAIT_HEAD,      ///< 等待 0xDD 帧头
        READ_CMD1,      ///< 读取命令码第 1 次
        READ_CMD2,      ///< 读取命令码第 2 次
        WAIT_TAIL       ///< 等待 0xDE 帧尾
    };

    static ParseState parseState;   ///< 当前解析状态
    static uint8_t cmdByte1;        ///< 暂存命令码第 1 次
    static uint32_t discardCount;   ///< 丢弃帧计数
};
