/**
 * @file GroundStationState.h
 * @brief 地面站全局状态管理模块
 *
 * 维护地面站的运行时状态，包括：
 *   - 车载在线状态与心跳时间
 *   - 物体识别结果（类型、数量）
 *   - 系统模式、机械臂状态、急停标志
 *   - ACK/NACK 序列号与错误码
 *   - 收发帧计数、错误计数
 *   - UI 脏标志（预留屏幕刷新）
 *
 * 线程安全：
 *   所有 public 方法内部使用 stateMutex 保护。
 *   多任务可安全读写状态。
 */

#pragma once

#include <cstdint>
#include <Arduino.h>

// ============================================================================
// 地面站状态结构体
// ============================================================================

struct GroundStationState {
    // ---- 连接状态 ----
    bool vehicleOnline;             ///< 车载 ESP32S3 在线标志
    uint32_t lastHeartbeatMs;       ///< 最近一次收到心跳的时间戳（ms）

    // ---- 物体识别 ----
    uint8_t lastObjectType;         ///< 最近识别到的物体类型
    uint8_t lastObjectCount;        ///< 最近识别到的物体数量

    // ---- 系统状态 ----
    uint8_t systemMode;             ///< 系统当前模式（0=待机，1=手动，2=自动等）
    uint8_t armStatus;              ///< 机械臂状态（0=空闲，1=运动中，2=错误等）
    uint8_t emergencyFlag;          ///< 急停标志（0=正常，非0=急停）

    // ---- ACK/NACK ----
    uint8_t lastAckSeq;             ///< 最近一次 ACK 对应的序列号
    uint8_t lastNackSeq;            ///< 最近一次 NACK 对应的序列号
    uint8_t lastNackError;          ///< 最近一次 NACK 错误码

    // ---- 统计计数 ----
    uint32_t txCount;               ///< 发送帧累计
    uint32_t rxCount;               ///< 接收帧累计
    uint32_t errorCount;            ///< 错误帧累计

    // ---- UI ----
    bool uiDirty;                   ///< UI 脏标志（预留屏幕刷新）
};

// ============================================================================
// 前向声明
// ============================================================================

extern SemaphoreHandle_t stateMutex;

class GroundStationStateManager {
public:
    /**
     * @brief 初始化状态管理器并创建互斥锁
     */
    static void begin();

    // ========================================================================
    // 状态更新
    // ========================================================================

    /// 更新心跳时间戳，标记车辆在线
    static void updateHeartbeat();

    /// 标记车辆离线（心跳超时时调用）
    static void setVehicleOffline();

    /// 更新状态上报（STATUS_REPORT）
    static void updateStatusReport(uint8_t sysMode, uint8_t armStat,
                                   uint8_t emergFlag, uint8_t objType,
                                   uint8_t objCount, uint8_t hbCount,
                                   uint8_t errCount);

    /// 更新物体识别结果（OBJECT_INFO）
    static void updateObjectInfo(uint8_t objType, uint8_t objCount);

    /// 更新 ACK
    static void updateAck(uint8_t seqNum);

    /// 更新 NACK
    static void updateNack(uint8_t seqNum, uint8_t errorCode);

    /// 增加发送帧计数
    static void incrementTxCount();

    /// 增加接收帧计数
    static void incrementRxCount();

    /// 增加错误计数
    static void incrementErrorCount();

    // ========================================================================
    // 状态读取（获取副本，线程安全）
    // ========================================================================

    /// 获取车辆在线状态
    static bool isVehicleOnline();

    /// 获取最后一次心跳时间戳
    static uint32_t getLastHeartbeatMs();

    /// 获取完整状态快照
    static GroundStationState getState();

    /// 获取发送帧累计
    static uint32_t getTxCount();

    /// 获取接收帧累计
    static uint32_t getRxCount();

    // ========================================================================
    // 调试打印
    // ========================================================================

    /// 通过 Serial 打印当前状态摘要
    static void printStatus();

    /// 打印 STATUS_REPORT 详细内容
    static void printStatusReport(const GroundStationState &s);

    /// 打印物体识别信息
    static void printObjectInfo();

private:
    static GroundStationState state; ///< 全局状态实例
};
