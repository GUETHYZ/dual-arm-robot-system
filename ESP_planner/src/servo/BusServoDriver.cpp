/**
 * @file BusServoDriver.cpp
 * @brief 众灵总线舵机驱动实现
 *
 * 通过 SerialManager::sendToServo() 发送字符串指令。
 * 所有舵机通信必须经过此类，业务代码不直接拼接指令字符串。
 */

#include "servo/BusServoDriver.h"
#include "comm/SerialManager.h"
#include "config/HardwareConfig.h"
#include <stdio.h>

// 静态成员定义
char BusServoDriver::cmdBuffer[SERVO_CMD_BUF_SIZE];

namespace {
bool isUnusedPose(const ServoPose& pose) {
    return pose.pwm == SERVO_PWM_UNUSED;
}
}

void BusServoDriver::begin() {
    DebugSerial.println("[SERVO] 总线舵机驱动已就绪");
}

// ============================================================
// 参数校验
// ============================================================

bool BusServoDriver::checkId(uint8_t id) {
    if (id > SERVO_ID_MAX) {
        DebugSerial.printf("[SERVO] 错误：舵机 ID %d 超出范围 (0~%d)\n",
                          id, SERVO_ID_MAX);
        return false;
    }
    return true;
}

bool BusServoDriver::checkPwm(uint16_t pwm) {
    if (pwm < SERVO_PWM_MIN || pwm > SERVO_PWM_MAX) {
        DebugSerial.printf("[SERVO] 错误：PWM %d 超出范围 (%d~%d)\n",
                          pwm, SERVO_PWM_MIN, SERVO_PWM_MAX);
        return false;
    }
    return true;
}

bool BusServoDriver::checkTime(uint16_t timeMs) {
    if (timeMs > SERVO_TIME_MAX) {
        DebugSerial.printf("[SERVO] 错误：时间 %d ms 超出范围 (0~%d)\n",
                          timeMs, SERVO_TIME_MAX);
        return false;
    }
    return true;
}

// ============================================================
// 单舵机控制
// ============================================================

bool BusServoDriver::moveServo(uint8_t id, uint16_t pwm, uint16_t timeMs) {
    if (!checkId(id) || !checkPwm(pwm) || !checkTime(timeMs)) {
        return false;
    }

    // 格式：#000P1500T1000!
    int len = snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE,
                       "#%03dP%04dT%04d!", id, pwm, timeMs);
    if (len < 0 || len >= SERVO_CMD_BUF_SIZE) {
        DebugSerial.println("[SERVO] 错误：命令字符串溢出");
        return false;
    }

    return sendCommand(cmdBuffer);
}

// ============================================================
// 多舵机控制
// ============================================================

size_t BusServoDriver::buildMultiServoCmd(const ServoPose* poses, uint8_t count,
                                          char* buffer, size_t bufSize) {
    if (count == 0 || poses == nullptr) return 0;
    if (count > TOTAL_SERVO_COUNT) count = TOTAL_SERVO_COUNT;
    if (bufSize < 3) return 0;

    size_t pos = 0;
    uint8_t activeCount = 0;

    // 开头 {
    buffer[pos++] = '{';

    for (uint8_t i = 0; i < count; i++) {
        if (isUnusedPose(poses[i])) {
            continue;
        }

        int written = snprintf(buffer + pos, bufSize - pos,
                               "#%03dP%04dT%04d!",
                               poses[i].id, poses[i].pwm, poses[i].timeMs);
        if (written < 0 || (size_t)written >= bufSize - pos) {
            return 0;  // 缓冲区不足
        }
        pos += written;
        activeCount++;
    }

    if (activeCount == 0) return 0;

    // 结尾 }
    if (pos >= bufSize - 1) return 0;
    buffer[pos++] = '}';

    buffer[pos] = '\0';
    return pos;
}

bool BusServoDriver::moveServos(const ServoPose* poses, uint8_t count) {
    if (poses == nullptr || count == 0) return false;

    // 校验所有有效姿态；PWM=0 是动作组占位，不发送给控制板。
    uint8_t activeCount = 0;
    const ServoPose* singleActivePose = nullptr;

    for (uint8_t i = 0; i < count; i++) {
        if (isUnusedPose(poses[i])) {
            continue;
        }

        if (!checkId(poses[i].id) || !checkPwm(poses[i].pwm)
            || !checkTime(poses[i].timeMs)) {
            return false;
        }

        activeCount++;
        singleActivePose = &poses[i];
    }

    if (activeCount == 0) {
        DebugSerial.println("[SERVO] 组合命令没有有效姿态，跳过发送");
        return true;
    }

    if (activeCount == 1 && singleActivePose != nullptr) {
        return moveServo(singleActivePose->id, singleActivePose->pwm,
                         singleActivePose->timeMs);
    }

    size_t len = buildMultiServoCmd(poses, count, cmdBuffer, SERVO_CMD_BUF_SIZE);
    if (len == 0) {
        DebugSerial.println("[SERVO] 错误：组合命令构建失败（缓冲区溢出？）");
        return false;
    }

    return sendCommand(cmdBuffer);
}

// ============================================================
// 特殊控制
// ============================================================

bool BusServoDriver::stopServo(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPDST!", id);
    return sendCommand(cmdBuffer);
}

bool BusServoDriver::stopAll(const uint8_t* ids, uint8_t count) {
    if (ids == nullptr || count == 0) return false;

    // 构建多条停止指令（但不使用花括号组合，逐条发送）
    bool allOk = true;
    for (uint8_t i = 0; i < count; i++) {
        if (!stopServo(ids[i])) {
            allOk = false;
        }
        // 少量延迟，避免串口缓冲区溢出
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return allOk;
}

bool BusServoDriver::pauseServo(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPDPT!", id);
    return sendCommand(cmdBuffer);
}

bool BusServoDriver::resumeServo(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPDCT!", id);
    return sendCommand(cmdBuffer);
}

bool BusServoDriver::torqueOff(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPULK!", id);
    return sendCommand(cmdBuffer);
}

bool BusServoDriver::torqueOn(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPULR!", id);
    return sendCommand(cmdBuffer);
}

bool BusServoDriver::torqueOffAll() {
    // 对全部 8 个舵机发送释力指令
    const uint8_t ids[TOTAL_SERVO_COUNT] = {
        LEFT_SHOULDER_ID, LEFT_ELBOW_ID, LEFT_GRIPPER_ID,
        RIGHT_SHOULDER_ID, RIGHT_ELBOW_ID, RIGHT_GRIPPER_ID
    };
    bool allOk = true;
    for (uint8_t i = 0; i < TOTAL_SERVO_COUNT; i++) {
        if (!torqueOff(ids[i])) allOk = false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return allOk;
}

bool BusServoDriver::torqueOnAll() {
    const uint8_t ids[TOTAL_SERVO_COUNT] = {
        LEFT_SHOULDER_ID, LEFT_ELBOW_ID, LEFT_GRIPPER_ID,
        RIGHT_SHOULDER_ID, RIGHT_ELBOW_ID, RIGHT_GRIPPER_ID
    };
    bool allOk = true;
    for (uint8_t i = 0; i < TOTAL_SERVO_COUNT; i++) {
        if (!torqueOn(ids[i])) allOk = false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    return allOk;
}

// ============================================================
// 读取接口
// ============================================================

bool BusServoDriver::readAngle(uint8_t id) {
    if (!checkId(id)) return false;

    snprintf(cmdBuffer, SERVO_CMD_BUF_SIZE, "#%03dPRAD!", id);
    return sendCommand(cmdBuffer);
}

// ============================================================
// 内部发送
// ============================================================

bool BusServoDriver::sendCommand(const char* cmd) {
    if (cmd == nullptr) return false;

    // 可选的舵机控制板方向引脚控制
    if (PIN_SERVO_DIR >= 0) {
        digitalWrite(PIN_SERVO_DIR, HIGH);  // 切换到发送模式
    }

    bool result = SerialManager::sendToServo(cmd);

    if (PIN_SERVO_DIR >= 0) {
        // 等待发送完成后切换回接收模式
        ServoSerial.flush();
        digitalWrite(PIN_SERVO_DIR, LOW);
    }

    return result;
}
