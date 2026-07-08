/**
 * @file GroundStationState.cpp
 * @brief 地面站全局状态管理实现
 */

#include "app/GroundStationState.h"
#include "config/HardwareConfig.h"

// ============================================================================
// 静态成员初始化
// ============================================================================

GroundStationState GroundStationStateManager::state = {
    false,          // vehicleOnline
    0,              // lastHeartbeatMs
    0,              // lastObjectType
    0,              // lastObjectCount
    0,              // systemMode
    0,              // armStatus
    0,              // emergencyFlag
    0,              // lastAckSeq
    0,              // lastNackSeq
    0,              // lastNackError
    0,              // txCount
    0,              // rxCount
    0,              // errorCount
    false           // uiDirty
};

// ============================================================================
// 互斥锁辅助宏
// ============================================================================

/// 带超时的获取互斥锁（50ms）
#define TAKE_MUTEX(m, timeoutMs) xSemaphoreTake((m), pdMS_TO_TICKS(timeoutMs))

/// 释放互斥锁
#define GIVE_MUTEX(m) xSemaphoreGive(m)

// ============================================================================
// 初始化
// ============================================================================

void GroundStationStateManager::begin() {
    if (stateMutex == nullptr) {
        stateMutex = xSemaphoreCreateMutex();
    }

    // 清空状态
    state.vehicleOnline    = false;
    state.lastHeartbeatMs  = 0;
    state.lastObjectType   = 0;
    state.lastObjectCount  = 0;
    state.systemMode       = 0;
    state.armStatus        = 0;
    state.emergencyFlag    = 0;
    state.lastAckSeq       = 0;
    state.lastNackSeq      = 0;
    state.lastNackError    = 0;
    state.txCount          = 0;
    state.rxCount          = 0;
    state.errorCount       = 0;
    state.uiDirty          = false;

    DebugSerial.println(F("[STATE] state manager initialized"));
}

// ============================================================================
// 状态更新
// ============================================================================

void GroundStationStateManager::updateHeartbeat() {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.lastHeartbeatMs = millis();
        state.vehicleOnline = true;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }
}

void GroundStationStateManager::setVehicleOffline() {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.vehicleOnline = false;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }
}

void GroundStationStateManager::updateStatusReport(
        uint8_t sysMode, uint8_t armStat,
        uint8_t emergFlag, uint8_t objType,
        uint8_t objCount, uint8_t hbCount,
        uint8_t errCount)
{
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.systemMode    = sysMode;
        state.armStatus     = armStat;
        state.emergencyFlag = emergFlag;
        state.lastObjectType  = objType;
        state.lastObjectCount = objCount;
        state.lastHeartbeatMs = millis();
        state.vehicleOnline = true;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }

    // 打印状态
    GroundStationState snapshot = getState();
    printStatusReport(snapshot);
}

void GroundStationStateManager::updateObjectInfo(uint8_t objType, uint8_t objCount) {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.lastObjectType  = objType;
        state.lastObjectCount = objCount;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }

    printObjectInfo();
}

void GroundStationStateManager::updateAck(uint8_t seqNum) {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.lastAckSeq = seqNum;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }

    DebugSerial.printf("[LORA RX] ACK received for seq=%u\n", seqNum);
}

void GroundStationStateManager::updateNack(uint8_t seqNum, uint8_t errorCode) {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.lastNackSeq   = seqNum;
        state.lastNackError = errorCode;
        state.errorCount++;
        state.uiDirty = true;
        GIVE_MUTEX(stateMutex);
    }

    DebugSerial.printf("[LORA RX] NACK received seq=%u, error=0x%02X\n",
                       seqNum, errorCode);
}

void GroundStationStateManager::incrementTxCount() {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.txCount++;
        GIVE_MUTEX(stateMutex);
    }
}

void GroundStationStateManager::incrementRxCount() {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.rxCount++;
        GIVE_MUTEX(stateMutex);
    }
}

void GroundStationStateManager::incrementErrorCount() {
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        state.errorCount++;
        GIVE_MUTEX(stateMutex);
    }
}

// ============================================================================
// 状态读取
// ============================================================================

bool GroundStationStateManager::isVehicleOnline() {
    bool result = false;
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        result = state.vehicleOnline;
        GIVE_MUTEX(stateMutex);
    }
    return result;
}

uint32_t GroundStationStateManager::getLastHeartbeatMs() {
    uint32_t result = 0;
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        result = state.lastHeartbeatMs;
        GIVE_MUTEX(stateMutex);
    }
    return result;
}

GroundStationState GroundStationStateManager::getState() {
    GroundStationState snapshot;
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        snapshot = state;
        GIVE_MUTEX(stateMutex);
    }
    return snapshot;
}

uint32_t GroundStationStateManager::getTxCount() {
    uint32_t result = 0;
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        result = state.txCount;
        GIVE_MUTEX(stateMutex);
    }
    return result;
}

uint32_t GroundStationStateManager::getRxCount() {
    uint32_t result = 0;
    if (TAKE_MUTEX(stateMutex, 50) == pdTRUE) {
        result = state.rxCount;
        GIVE_MUTEX(stateMutex);
    }
    return result;
}

// ============================================================================
// 调试打印
// ============================================================================

void GroundStationStateManager::printStatus() {
    GroundStationState s = getState();

    DebugSerial.println(F("========== [STATE] Ground Station Status =========="));
    DebugSerial.printf("  vehicleOnline  : %s\n", s.vehicleOnline ? "true" : "false");
    DebugSerial.printf("  lastHeartbeat  : %lu ms\n", s.lastHeartbeatMs);
    DebugSerial.printf("  systemMode     : %u\n", s.systemMode);
    DebugSerial.printf("  armStatus      : %u\n", s.armStatus);
    DebugSerial.printf("  emergencyFlag  : %u\n", s.emergencyFlag);
    DebugSerial.printf("  objectType     : %u\n", s.lastObjectType);
    DebugSerial.printf("  objectCount    : %u\n", s.lastObjectCount);
    DebugSerial.printf("  lastAckSeq     : %u\n", s.lastAckSeq);
    DebugSerial.printf("  lastNackSeq    : %u (err=0x%02X)\n", s.lastNackSeq, s.lastNackError);
    DebugSerial.printf("  txCount        : %lu\n", s.txCount);
    DebugSerial.printf("  rxCount        : %lu\n", s.rxCount);
    DebugSerial.printf("  errorCount     : %lu\n", s.errorCount);
    DebugSerial.println(F("==================================================="));
}

void GroundStationStateManager::printStatusReport(const GroundStationState &s) {
    DebugSerial.println(F("[LORA RX] STATUS_REPORT:"));
    DebugSerial.printf("  sysMode=%u, armStatus=%u, emerg=%u\n",
                       s.systemMode, s.armStatus, s.emergencyFlag);
    DebugSerial.printf("  objType=%u, objCount=%u\n",
                       s.lastObjectType, s.lastObjectCount);
}

void GroundStationStateManager::printObjectInfo() {
    GroundStationState s = getState();
    DebugSerial.printf("[LORA RX] OBJECT_INFO: type=%u, count=%u\n",
                       s.lastObjectType, s.lastObjectCount);
}
