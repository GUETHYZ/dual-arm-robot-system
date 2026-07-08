/**
 * @file RobotState.h
 * @brief 系统状态管理模块
 *
 * 集中存储系统运行状态，提供线程安全的读写接口。
 * 多个 FreeRTOS 任务可能同时访问，使用 spinlock 保护。
 */

#pragma once

#include <Arduino.h>

// ============================================================
// 系统模式
// ============================================================
enum class SystemMode : uint8_t {
    MANUAL    = 0x00,  // 手动控制模式
    AUTO      = 0x01,  // 自动模式（预留）
    EMERGENCY = 0x02,  // 急停模式
};

// ============================================================
// 机械臂状态
// ============================================================
enum class ArmStatus : uint8_t {
    IDLE    = 0x00,  // 空闲
    MOVING  = 0x01,  // 执行动作中
    STOPPED = 0x02,  // 已停止
    ERROR   = 0x03,  // 错误
};

// ============================================================
// 机器人状态类（全静态成员，无需实例化）
// ============================================================
class RobotState {
public:
    /**
     * @brief 初始化状态
     */
    static void begin();

    // --- 系统模式 ---
    static SystemMode getSystemMode();
    static void setSystemMode(SystemMode mode);

    // --- 急停 ---
    static bool isEmergencyStopped();
    static void setEmergencyStop(bool stopped);

    // --- 机械臂状态 ---
    static ArmStatus getArmStatus();
    static void setArmStatus(ArmStatus status);

    // --- 识别物体信息 ---
    static uint8_t getDetectedObjectType();
    static uint8_t getDetectedObjectCount();
    static void setDetectedObject(uint8_t type, uint8_t count);

    // --- 心跳计数 ---
    static uint32_t getHeartbeatCount();
    static void incrementHeartbeat();

    // --- 错误计数 ---
    static uint32_t getErrorCount();
    static void incrementErrorCount();

    // --- 序列号 ---
    static uint8_t getNextSeqNum();

    /**
     * @brief 构建状态上报 payload
     * @param buf  输出缓冲区（至少 8 字节）
     * @return 实际写入的字节数
     */
    static uint8_t buildStatusPayload(uint8_t* buf);

private:
    // 使用 FreeRTOS spinlock 保护多任务访问
    static portMUX_TYPE spinlock;

    static SystemMode systemMode;
    static ArmStatus   armStatus;
    static bool        emergencyStopFlag;
    static uint8_t     objectType;
    static uint8_t     objectCount;
    static uint32_t    heartbeatCount;
    static uint32_t    errorCount;
    static uint8_t     seqNum;
};
