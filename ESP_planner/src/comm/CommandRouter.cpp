/**
 * @file CommandRouter.cpp
 * @brief 命令路由任务实现
 *
 * 从系统命令队列取命令，根据功能码分类处理：
 * - 小车底盘类 → 转发 STM32
 * - 机械臂类   → 投递机械臂命令队列
 * - 系统类     → 本地处理
 * - 视觉识别类 → 更新 RobotState
 */

#include "comm/CommandRouter.h"
#include "comm/SerialManager.h"
#include "arm/ArmController.h"
#include "app/RobotState.h"
#include "config/DeviceConfig.h"
#include "config/HardwareConfig.h"
#include "config/TaskConfig.h"

// ============================================================
// 外部队列声明（定义在 main.cpp）
// ============================================================
extern QueueHandle_t systemCommandQueue;
extern QueueHandle_t armCommandQueue;

// ============================================================
// 命令路由任务
// ============================================================

void taskCommandRouter(void* param) {
    ProtocolFrame frame;
    DebugSerial.println("[CMD] 命令路由任务已启动");

    while (true) {
        // 从队列取命令（阻塞等待）
        if (xQueueReceive(systemCommandQueue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            routeCommand(frame);
        }

        // 检查急停按钮（可选硬件引脚）
        if (PIN_EMERGENCY_STOP >= 0) {
            if (digitalRead(PIN_EMERGENCY_STOP) == LOW) {  // 低电平有效
                // 触发急停
                ProtocolFrame estopFrame;
                estopFrame.version  = PROTOCOL_VERSION;
                estopFrame.srcAddr  = ESP32_ADDR;
                estopFrame.dstAddr  = ESP32_ADDR;
                estopFrame.msgType  = MsgType::CMD;
                estopFrame.funcCode = FuncCode::EMERGENCY_STOP;
                estopFrame.seqNum   = RobotState::getNextSeqNum();
                estopFrame.dataLen  = 0;
                estopFrame.valid    = true;
                routeCommand(estopFrame);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void routeCommand(const ProtocolFrame& frame) {
    if (!frame.valid) return;

    uint8_t funcCode = frame.funcCode;
    uint8_t srcAddr  = frame.srcAddr;
    uint8_t dstAddr  = frame.dstAddr;
    uint8_t seqNum   = frame.seqNum;

    // 检查是否是发给本机或广播
    if (dstAddr != ESP32_ADDR && dstAddr != BROADCAST_ADDR) {
        // 不是给我们的，忽略
        return;
    }

    // ============================================================
    // 系统类命令（本地处理）
    // ============================================================
    if (isSystemCommand(funcCode)) {
        switch (funcCode) {

        case FuncCode::HEARTBEAT:
            // 收到心跳，回复 ACK
            {
                ProtocolFrame ackFrame;
                buildAckFrame(ESP32_ADDR, srcAddr, seqNum, ackFrame);
                SerialManager::sendFrame(ackFrame, 'G');
            }
            break;

        case FuncCode::ACK:
        case FuncCode::NACK:
            // ACK/NACK 收到后由上层根据 seqNum 处理，此处仅记录
            break;

        case FuncCode::STATUS_QUERY:
            // 收到状态查询，回复状态上报
            {
                uint8_t statusData[8];
                uint8_t len = RobotState::buildStatusPayload(statusData);
                ProtocolFrame statusFrame;
                buildStatusReportFrame(statusData, len, statusFrame);
                statusFrame.seqNum = seqNum;
                SerialManager::sendFrame(statusFrame, 'G');
            }
            break;

        case FuncCode::EMERGENCY_STOP:
            // 急停命令
            DebugSerial.println("[CMD] !! 收到急停命令 !!");
            RobotState::setEmergencyStop(true);
            RobotState::setSystemMode(SystemMode::EMERGENCY);
            ArmController::emergencyStop();

            // 立即下发舵机停止/释力，避免等待 ARM 任务从当前动作组返回。
            ArmController::stopAllServos();
            ArmController::torqueOffAll();

            // 向 STM32 转发停止
            {
                ProtocolFrame stopFrame;
                stopFrame.version  = PROTOCOL_VERSION;
                stopFrame.srcAddr  = ESP32_ADDR;
                stopFrame.dstAddr  = STM32_ADDR;
                stopFrame.msgType  = MsgType::CMD;
                stopFrame.funcCode = FuncCode::CHASSIS_STOP;
                stopFrame.seqNum   = RobotState::getNextSeqNum();
                stopFrame.dataLen  = 0;
                stopFrame.valid    = true;
                SerialManager::sendFrame(stopFrame, 'C');
            }

            // 机械臂急停（投递到机械臂队列）
            {
                ProtocolFrame armStop;
                armStop.funcCode = FuncCode::EMERGENCY_STOP;
                armStop.valid = true;
                xQueueSend(armCommandQueue, &armStop, QUEUE_SEND_TIMEOUT);
            }

            // 回复 ACK
            {
                ProtocolFrame ackFrame;
                buildAckFrame(ESP32_ADDR, srcAddr, seqNum, ackFrame);
                SerialManager::sendFrame(ackFrame, 'G');
            }

            // LED 闪烁指示急停
            if (PIN_LED_STATUS >= 0) {
                digitalWrite(PIN_LED_STATUS, HIGH);
            }
            break;

        default:
            DebugSerial.printf("[CMD] 未知系统功能码: 0x%02X\n", funcCode);
            break;
        }

        return;
    }

    // 急停状态下拒绝非急停清除命令
    if (RobotState::isEmergencyStopped() && funcCode != FuncCode::EMERGENCY_STOP) {
        ProtocolFrame nackFrame;
        buildNackFrame(ESP32_ADDR, srcAddr, seqNum, ErrorCode::EMERGENCY_ACTIVE, nackFrame);
        SerialManager::sendFrame(nackFrame, 'G');
        return;
    }

    // ============================================================
    // 小车底盘类命令 → 转发给 STM32
    // ============================================================
    if (isChassisCommand(funcCode)) {
        // 重新打包帧，源/目标地址改为 ESP32 → STM32，并使用 ESP32 下游序列号。
        ProtocolFrame chassisFrame = frame;
        chassisFrame.srcAddr = ESP32_ADDR;
        chassisFrame.dstAddr = STM32_ADDR;
        chassisFrame.seqNum = RobotState::getNextSeqNum();

        if (funcCode == FuncCode::CHASSIS_STOP) {
            DebugSerial.println("[CMD] 优先转发底盘停止 0x16 → STM32");
        } else {
            DebugSerial.printf("[CMD] 转发底盘命令 0x%02X → STM32\n", funcCode);
        }
        SerialManager::sendFrame(chassisFrame, 'C');
        return;
    }

    // ============================================================
    // 机械臂类命令 → 投递到机械臂命令队列
    // ============================================================
    if (isArmCommand(funcCode)) {
        DebugSerial.printf("[CMD] 投递机械臂命令 0x%02X → ARM 队列\n", funcCode);

        if (xQueueSend(armCommandQueue, &frame, QUEUE_SEND_TIMEOUT) != pdTRUE) {
            DebugSerial.println("[CMD] 警告：机械臂命令队列已满，丢弃命令");
            RobotState::incrementErrorCount();

            ProtocolFrame nackFrame;
            buildNackFrame(ESP32_ADDR, srcAddr, seqNum, ErrorCode::QUEUE_FULL, nackFrame);
            SerialManager::sendFrame(nackFrame, 'G');
        }
        return;
    }

    // ============================================================
    // 视觉识别类命令 → 更新 RobotState
    // ============================================================
    if (funcCode == FuncCode::OBJECT_INFO) {
        uint8_t objType  = (frame.dataLen > 0) ? frame.data[0] : ObjectType::UNKNOWN;
        uint8_t objCount = (frame.dataLen > 1) ? frame.data[1] : 0;

        RobotState::setDetectedObject(objType, objCount);

        DebugSerial.printf("[CMD] 收到识别结果: 类型=0x%02X, 数量=%d\n",
                          objType, objCount);

        // 转发识别结果给地面站
        ProtocolFrame infoFrame = frame;
        infoFrame.srcAddr = ESP32_ADDR;
        infoFrame.dstAddr = GROUND_ADDR;
        SerialManager::sendFrame(infoFrame, 'G');
        return;
    }

    // ============================================================
    // 未知命令
    // ============================================================
    DebugSerial.printf("[CMD] 未知功能码: 0x%02X (来源: 0x%02X)\n",
                      funcCode, srcAddr);
    RobotState::incrementErrorCount();

    ProtocolFrame nackFrame;
    buildNackFrame(ESP32_ADDR, srcAddr, seqNum, ErrorCode::UNKNOWN_CMD, nackFrame);
    SerialManager::sendFrame(nackFrame, 'G');
}
