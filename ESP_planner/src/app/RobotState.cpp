/**
 * @file RobotState.cpp
 * @brief 系统状态管理实现
 *
 * 线程安全的状态存储，使用 FreeRTOS spinlock 保护。
 */

#include "app/RobotState.h"
#include "config/DeviceConfig.h"

// 静态成员定义
portMUX_TYPE RobotState::spinlock = portMUX_INITIALIZER_UNLOCKED;

SystemMode RobotState::systemMode       = SystemMode::MANUAL;
ArmStatus   RobotState::armStatus       = ArmStatus::IDLE;
bool        RobotState::emergencyStopFlag = false;
uint8_t     RobotState::objectType      = 0;
uint8_t     RobotState::objectCount     = 0;
uint32_t    RobotState::heartbeatCount  = 0;
uint32_t    RobotState::errorCount      = 0;
uint8_t     RobotState::seqNum          = 0;

void RobotState::begin() {
    portENTER_CRITICAL(&spinlock);
    systemMode        = SystemMode::MANUAL;
    armStatus         = ArmStatus::IDLE;
    emergencyStopFlag = false;
    objectType        = 0;
    objectCount       = 0;
    heartbeatCount    = 0;
    errorCount        = 0;
    seqNum            = 0;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 系统模式
// ============================================================

SystemMode RobotState::getSystemMode() {
    portENTER_CRITICAL(&spinlock);
    SystemMode m = systemMode;
    portEXIT_CRITICAL(&spinlock);
    return m;
}

void RobotState::setSystemMode(SystemMode mode) {
    portENTER_CRITICAL(&spinlock);
    systemMode = mode;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 急停
// ============================================================

bool RobotState::isEmergencyStopped() {
    portENTER_CRITICAL(&spinlock);
    bool flag = emergencyStopFlag;
    portEXIT_CRITICAL(&spinlock);
    return flag;
}

void RobotState::setEmergencyStop(bool stopped) {
    portENTER_CRITICAL(&spinlock);
    emergencyStopFlag = stopped;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 机械臂状态
// ============================================================

ArmStatus RobotState::getArmStatus() {
    portENTER_CRITICAL(&spinlock);
    ArmStatus s = armStatus;
    portEXIT_CRITICAL(&spinlock);
    return s;
}

void RobotState::setArmStatus(ArmStatus status) {
    portENTER_CRITICAL(&spinlock);
    armStatus = status;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 识别物体信息
// ============================================================

uint8_t RobotState::getDetectedObjectType() {
    portENTER_CRITICAL(&spinlock);
    uint8_t t = objectType;
    portEXIT_CRITICAL(&spinlock);
    return t;
}

uint8_t RobotState::getDetectedObjectCount() {
    portENTER_CRITICAL(&spinlock);
    uint8_t c = objectCount;
    portEXIT_CRITICAL(&spinlock);
    return c;
}

void RobotState::setDetectedObject(uint8_t type, uint8_t count) {
    portENTER_CRITICAL(&spinlock);
    objectType  = type;
    objectCount = count;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 心跳
// ============================================================

uint32_t RobotState::getHeartbeatCount() {
    portENTER_CRITICAL(&spinlock);
    uint32_t c = heartbeatCount;
    portEXIT_CRITICAL(&spinlock);
    return c;
}

void RobotState::incrementHeartbeat() {
    portENTER_CRITICAL(&spinlock);
    heartbeatCount++;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 错误计数
// ============================================================

uint32_t RobotState::getErrorCount() {
    portENTER_CRITICAL(&spinlock);
    uint32_t c = errorCount;
    portEXIT_CRITICAL(&spinlock);
    return c;
}

void RobotState::incrementErrorCount() {
    portENTER_CRITICAL(&spinlock);
    errorCount++;
    portEXIT_CRITICAL(&spinlock);
}

// ============================================================
// 序列号
// ============================================================

uint8_t RobotState::getNextSeqNum() {
    portENTER_CRITICAL(&spinlock);
    uint8_t n = seqNum++;
    portEXIT_CRITICAL(&spinlock);
    return n;
}

// ============================================================
// 状态上报 payload 构建
// ============================================================

uint8_t RobotState::buildStatusPayload(uint8_t* buf) {
    if (buf == nullptr) return 0;

    portENTER_CRITICAL(&spinlock);
    buf[0] = (uint8_t)systemMode;
    buf[1] = (uint8_t)armStatus;
    buf[2] = emergencyStopFlag ? 1 : 0;
    buf[3] = objectType;
    buf[4] = objectCount;
    buf[5] = (uint8_t)(heartbeatCount & 0xFF);
    buf[6] = (uint8_t)(errorCount & 0xFF);
    buf[7] = 0;  // 保留
    portEXIT_CRITICAL(&spinlock);

    return 8;
}
