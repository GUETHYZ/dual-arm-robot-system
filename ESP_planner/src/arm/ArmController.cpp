/**
 * @file ArmController.cpp
 * @brief 机械臂控制器实现
 *
 * 执行预设动作组，将动作步骤转换为舵机指令。
 * 使用 vTaskDelay 而非 delay，可响应急停。
 */

#include "arm/ArmController.h"
#include "arm/ArmActionGroups.h"
#include "servo/BusServoDriver.h"
#include "app/RobotState.h"
#include "config/HardwareConfig.h"

// 静态成员
volatile bool ArmController::emergencyFlag = false;

void ArmController::begin() {
    emergencyFlag = false;
    BusServoDriver::begin();
    DebugSerial.println("[ARM] 机械臂控制器已就绪");
    DebugSerial.printf("[ARM] 已加载 %d 个动作组\n",
                       (int)ArmAction::ACTION_COUNT);
}

bool ArmController::executeAction(ArmAction action) {
    uint8_t index = (uint8_t)action;

    if (index >= (uint8_t)ArmAction::ACTION_COUNT) {
        DebugSerial.printf("[ARM] 错误：无效动作索引 %d\n", index);
        return false;
    }

    // 检查急停标志。急停动作本身必须允许执行，否则无法发送 PDST/PULK。
    if (emergencyFlag && action != ArmAction::EMERGENCY_STOP) {
        DebugSerial.println("[ARM] 急停状态，拒绝执行动作");
        return false;
    }

    const ActionGroup& group = actionGroups[index];

    DebugSerial.printf("[ARM] 开始执行动作组：%s (%d 步骤)\n",
                       group.name, group.stepCount);

    // 特殊处理：无步骤的动作组
    if (group.stepCount == 0) {
        switch (action) {
        case ArmAction::STOP_ALL:
            stopAllServos();
            return true;

        case ArmAction::EMERGENCY_STOP:
            emergencyStop();
            stopAllServos();
            torqueOffAll();
            RobotState::setEmergencyStop(true);
            RobotState::setSystemMode(SystemMode::EMERGENCY);
            // 通知地面站
            DebugSerial.println("[ARM] !! 紧急停止已触发 !!");
            return true;

        case ArmAction::TORQUE_OFF_ALL:
            torqueOffAll();
            return true;

        case ArmAction::TORQUE_ON_ALL:
            torqueOnAll();
            return true;

        default:
            DebugSerial.printf("[ARM] 警告：动作组 '%s' 无步骤\n", group.name);
            return true;
        }
    }

    // 设置状态为运动中
    RobotState::setArmStatus(ArmStatus::MOVING);

    // 逐步执行动作组
    for (uint8_t i = 0; i < group.stepCount; i++) {
        // 每步执行前检查急停
        if (emergencyFlag) {
            DebugSerial.printf("[ARM] 急停中断：步骤 %d/%d 前退出\n",
                              i + 1, group.stepCount);
            RobotState::setArmStatus(ArmStatus::STOPPED);
            return false;
        }

        if (!executeStep(group.steps[i])) {
            DebugSerial.printf("[ARM] 步骤 %d/%d 执行失败\n",
                              i + 1, group.stepCount);
            RobotState::setArmStatus(ArmStatus::ERROR);
            return false;
        }

        DebugSerial.printf("[ARM] 步骤 %d/%d 完成，等待 %d ms\n",
                          i + 1, group.stepCount, group.steps[i].waitMs);

        // 步骤间等待（使用 vTaskDelay，可响应 FreeRTOS 调度）
        if (group.steps[i].waitMs > 0) {
            vTaskDelay(pdMS_TO_TICKS(group.steps[i].waitMs));
        }
    }

    DebugSerial.printf("[ARM] 动作组 '%s' 执行完成\n", group.name);
    RobotState::setArmStatus(ArmStatus::IDLE);
    return true;
}

bool ArmController::executeStep(const ActionStep& step) {
    if (step.poseCount == 0) {
        return true;  // 空步骤
    }

    if (step.poseCount == 1) {
        // 单舵机，使用单条指令
        const ServoPose& p = step.poses[0];
        return BusServoDriver::moveServo(p.id, p.pwm, p.timeMs);
    } else {
        // 多舵机，使用组合指令
        return BusServoDriver::moveServos(step.poses, step.poseCount);
    }
}

void ArmController::emergencyStop() {
    emergencyFlag = true;
}

void ArmController::clearEmergencyStop() {
    emergencyFlag = false;
}

bool ArmController::isEmergencyStopped() {
    return emergencyFlag;
}

void ArmController::stopAllServos() {
    const uint8_t ids[TOTAL_SERVO_COUNT] = {
        LEFT_BASE_ID, LEFT_SHOULDER_ID, LEFT_ELBOW_ID, LEFT_GRIPPER_ID,
        RIGHT_BASE_ID, RIGHT_SHOULDER_ID, RIGHT_ELBOW_ID, RIGHT_GRIPPER_ID
    };
    BusServoDriver::stopAll(ids, TOTAL_SERVO_COUNT);
}

void ArmController::torqueOffAll() {
    BusServoDriver::torqueOffAll();
}

void ArmController::torqueOnAll() {
    BusServoDriver::torqueOnAll();
}
