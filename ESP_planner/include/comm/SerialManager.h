/**
 * @file SerialManager.h
 * @brief 串口管理器
 *
 * 统一管理所有串口通道的初始化与发送。
 * 所有串口发送必须经过 SerialManager，不允许业务代码直接操作串口。
 *
 * 特性：
 * - 每个串口发送使用独立 Mutex 保护
 * - 舵机通道支持字符串指令发送
 * - 发送失败时通过 DebugSerial 打印错误
 * - 发送后自动 flush()
 */

#pragma once

#include <Arduino.h>
#include "config/HardwareConfig.h"
#include "protocol/Protocol.h"

class SerialManager {
public:
    /**
     * @brief 初始化所有串口
     *
     * 对每个业务串口：
     * 1. 调用 setPins(rxPin, txPin) 配置引脚映射
     * 2. 调用 begin(baud) 启动串口
     *
     * 注意：DebugSerial 独立使用 USB CDC；地面站、舵机、Jetson/STM32 均使用硬件 UART。
     * Arduino-ESP32 HardwareSerial::setPins() 参数顺序为 RX 在前、TX 在后。
     */
    static void beginAll();

    // ============================================================
    // 获取 Stream 引用（用于接收）
    // ============================================================
    static Stream& ground()  { return GroundSerial; }
    static Stream& jetson()  { return JetsonSerial; }
    static Stream& chassis() { return ChassisSerial; }
    static Stream& servo()   { return ServoSerial; }

    // ============================================================
    // 二进制数据发送
    // ============================================================

    /**
     * @brief 向地面站发送二进制数据
     * @param data 数据指针
     * @param len  数据长度
     * @return true = 发送成功
     */
    static bool sendToGround(const uint8_t* data, size_t len);

    /**
     * @brief 向 Jetson 发送二进制数据
     */
    static bool sendToJetson(const uint8_t* data, size_t len);

    /**
     * @brief 向 STM32 底盘发送二进制数据
     */
    static bool sendToChassis(const uint8_t* data, size_t len);

    /**
     * @brief 向舵机控制板发送字符串指令
     * @param cmd 以 null 结尾的 C 字符串
     * @return true = 发送成功
     */
    static bool sendToServo(const char* cmd);

    // ============================================================
    // 辅助函数
    // ============================================================

    /**
     * @brief 发送协议帧（自动打包）
     * @param frame  已填充的协议帧
     * @param target 目标串口标识：'G'=地面站, 'J'=Jetson, 'C'=底盘
     * @return true = 发送成功
     */
    static bool sendFrame(const ProtocolFrame& frame, char target);

private:
    // 每个发送通道的 Mutex
    static SemaphoreHandle_t groundMutex;
    static SemaphoreHandle_t jetsonMutex;
    static SemaphoreHandle_t chassisMutex;
    static SemaphoreHandle_t servoMutex;

    // 初始化单个串口
    static void initSerial(HardwareSerial& serial, int rxPin, int txPin,
                           uint32_t baud, const char* name);

    // 通用二进制发送
    static bool sendBinary(Stream& stream, SemaphoreHandle_t mutex,
                           const uint8_t* data, size_t len, const char* name);
};
