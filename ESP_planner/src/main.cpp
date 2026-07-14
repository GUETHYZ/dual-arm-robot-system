/**
 * @file main.cpp
 * @brief 小型双臂复合机器人系统 — ESP32S3 主控程序
 *
 * 功能：
 * - 接收地面站人工控制指令（统一协议）
 * - 接收 Jetson Nano 物体识别结果（统一协议）
 * - 向 STM32 底盘转发运动指令（统一协议）
 * - 控制双机械臂（众灵总线舵机，字符串协议）
 * - 心跳与状态上报
 * - 急停处理
 *
 * 架构：
 * - 使用 FreeRTOS 任务组织代码
 * - 每个串口通道独立接收任务 + 独立 ProtocolParser 实例
 * - 命令路由任务统一分发
 * - 机械臂任务独立执行动作组
 *
 * 注意：
 * - 串口接收不放在 loop() 中
 * - 不使用阻塞式 readString()
 * - 串口发送使用 Mutex 保护
 * - 接收任务每轮处理最多 96 字节后主动让出 CPU
 */

#include <Arduino.h>
#include "config/HardwareConfig.h"
#include "config/DeviceConfig.h"
#include "config/TaskConfig.h"
#include "protocol/Protocol.h"
#include "protocol/ProtocolParser.h"
#include "protocol/Crc16.h"
#include "servo/BusServoDriver.h"
#include "arm/ArmActionGroups.h"
#include "arm/ArmController.h"
#include "comm/SerialManager.h"
#include "comm/CommandRouter.h"
#include "app/RobotState.h"

// ============================================================
// 全局队列定义
// ============================================================
QueueHandle_t systemCommandQueue = nullptr;   // 系统命令队列（地面站/Jetson → 路由任务）
QueueHandle_t armCommandQueue    = nullptr;   // 机械臂命令队列（路由任务 → 机械臂任务）

// ============================================================
// 各串口协议解析器（每个通道独立实例）
// ============================================================
static ProtocolParser groundParser;   // 地面站解析器
static ProtocolParser jetsonParser;   // Jetson 解析器

// 错误统计
static uint32_t totalRxErrors = 0;

// 地面站接收指示灯：收到完整有效地面站协议帧后短闪一次，便于脱离调试串口做最小验证。
static volatile TickType_t groundRxLedOffTick = 0;

static void pulseGroundRxLed() {
    if (PIN_GROUND_RX_LED < 0) {
        return;
    }

    digitalWrite(PIN_GROUND_RX_LED, GROUND_RX_LED_ON_LEVEL);
    groundRxLedOffTick = xTaskGetTickCount() + pdMS_TO_TICKS(GROUND_RX_LED_PULSE_MS);
}

static void updateGroundRxLed() {
    if (PIN_GROUND_RX_LED < 0 || groundRxLedOffTick == 0) {
        return;
    }

    TickType_t now = xTaskGetTickCount();
    if ((TickType_t)(now - groundRxLedOffTick) < (TickType_t)(UINT32_MAX / 2)) {
        digitalWrite(PIN_GROUND_RX_LED, GROUND_RX_LED_OFF_LEVEL);
        groundRxLedOffTick = 0;
    }
}

// ============================================================
// 串口接收函数
// ============================================================

/**
 * @brief 将地面站帧投递到系统命令队列
 *
 * CHASSIS_STOP 是移动按钮弹起后的安全停止帧，必须优先于旧方向命令处理。
 */
static bool enqueueGroundFrame(const ProtocolFrame& frame) {
    if (systemCommandQueue == nullptr) {
        return false;
    }

    const bool isChassisStop = (frame.msgType == MsgType::CMD)
        && (frame.funcCode == FuncCode::CHASSIS_STOP)
        && (frame.dstAddr == ESP32_ADDR || frame.dstAddr == BROADCAST_ADDR);

    if (!isChassisStop) {
        return xQueueSend(systemCommandQueue, &frame, QUEUE_SEND_TIMEOUT) == pdTRUE;
    }

    // 清理尚未路由的旧底盘命令，避免 STOP 后又执行积压的方向命令。
    ProtocolFrame preserved[QUEUE_LEN_SYSTEM_CMD];
    UBaseType_t preservedCount = 0;
    UBaseType_t droppedChassisCount = 0;
    UBaseType_t droppedOverflowCount = 0;
    const UBaseType_t maxPreserved = QUEUE_LEN_SYSTEM_CMD - 1;

    ProtocolFrame queuedFrame;
    while (xQueueReceive(systemCommandQueue, &queuedFrame, 0) == pdTRUE) {
        if (isChassisCommand(queuedFrame.funcCode)) {
            droppedChassisCount++;
            continue;
        }

        if (preservedCount < maxPreserved) {
            preserved[preservedCount++] = queuedFrame;
        } else {
            droppedOverflowCount++;
        }
    }

    const BaseType_t stopQueued = xQueueSendToFront(systemCommandQueue, &frame, QUEUE_SEND_TIMEOUT);

    for (UBaseType_t i = 0; i < preservedCount; i++) {
        if (xQueueSend(systemCommandQueue, &preserved[i], QUEUE_SEND_TIMEOUT) != pdTRUE) {
            droppedOverflowCount++;
        }
    }

    if (droppedChassisCount > 0 || droppedOverflowCount > 0) {
        DebugSerial.printf("[RX] CHASSIS_STOP 优先入队：丢弃旧底盘命令 %u 条，非底盘溢出 %u 条\n",
                          (unsigned)droppedChassisCount, (unsigned)droppedOverflowCount);
    }

    if (droppedOverflowCount > 0) {
        totalRxErrors += droppedOverflowCount;
    }

    return stopQueued == pdTRUE;
}

/**
 * @brief 地面站串口接收
 *
 * 从地面站串口读取字节，逐字节喂给协议解析器。
 * 收到完整帧后投递到系统命令队列。
 * 单次最多处理 96 字节，防止长时间占用 CPU。
 */
void receiveGroundSerial() {
    int processed = 0;

    while (GroundSerial.available() > 0 && processed < MAX_RX_BYTES_PER_CYCLE) {
        processed++;
        uint8_t byteIn = GroundSerial.read();

        ProtocolFrame frame;
        if (groundParser.inputByte(byteIn, frame)) {
            // 收到完整有效帧后再闪烁，表示地面站物理层与协议层均已通过。
            pulseGroundRxLed();

            // 收到完整帧，投递到命令队列；底盘 STOP 需要高优先级处理。
            if (!enqueueGroundFrame(frame)) {
                DebugSerial.println("[RX] 警告：系统命令队列已满，丢弃地面站帧");
                totalRxErrors++;
            }
        }
    }
}

/**
 * @brief Jetson 串口接收
 *
 * 从 Jetson 串口读取字节，逐字节喂给协议解析器。
 * 收到完整帧后投递到系统命令队列。
 */
void receiveJetsonSerial() {
    int processed = 0;

    while (JetsonSerial.available() > 0 && processed < MAX_RX_BYTES_PER_CYCLE) {
        processed++;
        uint8_t byteIn = JetsonSerial.read();

        ProtocolFrame frame;
        if (jetsonParser.inputByte(byteIn, frame)) {
            if (xQueueSend(systemCommandQueue, &frame, QUEUE_SEND_TIMEOUT) != pdTRUE) {
                DebugSerial.println("[RX] 警告：系统命令队列已满，丢弃 Jetson 帧");
                totalRxErrors++;
            }
        }
    }
}

// ============================================================
// FreeRTOS 任务函数
// ============================================================

/**
 * @brief 地面站接收任务
 *
 * 每 1ms 调用一次 receiveGroundSerial()。
 * 只负责接收和投递，不做业务处理。
 */
void taskGroundStation(void* param) {
    DebugSerial.println("[TASK] 地面站接收任务已启动");

    while (true) {
        receiveGroundSerial();
        updateGroundRxLed();
        vTaskDelay(pdMS_TO_TICKS(RX_TASK_DELAY));
    }
}

/**
 * @brief Jetson 接收任务
 *
 * 每 1ms 调用一次 receiveJetsonSerial()。
 * 只负责接收和投递，不做业务处理。
 */
void taskJetson(void* param) {
    DebugSerial.println("[TASK] Jetson 接收任务已启动");

    while (true) {
        receiveJetsonSerial();
        vTaskDelay(pdMS_TO_TICKS(RX_TASK_DELAY));
    }
}

/**
 * @brief 机械臂控制任务
 *
 * 从机械臂命令队列取命令，调用 ArmController 执行。
 * 优先级最高（4），保证机械臂动作实时性。
 */
void taskArm(void* param) {
    ProtocolFrame frame;
    DebugSerial.println("[TASK] 机械臂控制任务已启动");

    while (true) {
        // 阻塞等待命令
        if (xQueueReceive(armCommandQueue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
            uint8_t funcCode = frame.funcCode;

            // 清除急停模式（收到非急停命令时）
            if (funcCode != FuncCode::EMERGENCY_STOP
                && RobotState::getSystemMode() == SystemMode::EMERGENCY) {
                // 仍在急停中，忽略命令
                DebugSerial.println("[ARM] 急停模式下忽略机械臂命令");
                continue;
            }

            ArmAction action;

            switch (funcCode) {

            case FuncCode::EMERGENCY_STOP:
                action = ArmAction::EMERGENCY_STOP;
                break;

            case FuncCode::ARM_HOME:
                action = ArmAction::HOME;
                break;

            case FuncCode::ARM_LEFT_PICK:
                action = ArmAction::LEFT_PICK;
                break;

            case FuncCode::ARM_LEFT_PLACE:
                action = ArmAction::LEFT_PLACE;
                break;

            case FuncCode::ARM_RIGHT_PICK:
                action = ArmAction::RIGHT_PICK;
                break;

            case FuncCode::ARM_RIGHT_PLACE:
                action = ArmAction::RIGHT_PLACE;
                break;

            case FuncCode::ARM_BOTH_PICK:
                action = ArmAction::BOTH_PICK;
                break;

            case FuncCode::ARM_BOTH_PLACE:
                action = ArmAction::BOTH_PLACE;
                break;

            case FuncCode::ARM_STOP:
                action = ArmAction::STOP_ALL;
                break;

            case FuncCode::ARM_TORQUE_OFF:
                action = ArmAction::TORQUE_OFF_ALL;
                break;

            case FuncCode::ARM_TORQUE_ON:
                action = ArmAction::TORQUE_ON_ALL;
                break;

            default:
                DebugSerial.printf("[ARM] 未知机械臂命令: 0x%02X\n", funcCode);
                continue;
            }

            // 执行动作组
            bool result = ArmController::executeAction(action);
            DebugSerial.printf("[ARM] 动作执行结果: %s\n",
                              result ? "成功" : "失败/中断");

            // 清除急停（只有急停动作会设置急停标志，其他动作不做处理）
            if (funcCode == FuncCode::EMERGENCY_STOP) {
                // 保持急停状态，等待人工恢复
            }
        }
    }
}

/**
 * @brief 状态上报任务
 *
 * 定时向地面站发送心跳和系统状态。
 * 优先级最低（1）。
 */
void taskStatus(void* param) {
    DebugSerial.println("[TASK] 状态上报任务已启动");
    TickType_t lastHeartbeat = xTaskGetTickCount();
    TickType_t lastStatus    = xTaskGetTickCount();

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // 周期性心跳
        if (pdTICKS_TO_MS(now - lastHeartbeat) >= HEARTBEAT_INTERVAL_MS) {
            lastHeartbeat = now;
            RobotState::incrementHeartbeat();

            // 仅当不在急停时才发送心跳
            if (!RobotState::isEmergencyStopped()) {
                ProtocolFrame hbFrame;
                buildHeartbeatFrame(hbFrame);
                hbFrame.seqNum = RobotState::getNextSeqNum();
                SerialManager::sendFrame(hbFrame, 'G');
            }
        }

        // 周期性状态上报
        if (pdTICKS_TO_MS(now - lastStatus) >= STATUS_REPORT_INTERVAL_MS) {
            lastStatus = now;

            uint8_t statusData[8];
            uint8_t len = RobotState::buildStatusPayload(statusData);

            ProtocolFrame statusFrame;
            buildStatusReportFrame(statusData, len, statusFrame);
            statusFrame.seqNum = RobotState::getNextSeqNum();
            SerialManager::sendFrame(statusFrame, 'G');

            // 急停状态通过 LED 指示
            if (PIN_LED_STATUS >= 0) {
                if (RobotState::isEmergencyStopped()) {
                    // 急停：LED 闪烁
                    digitalWrite(PIN_LED_STATUS,
                                 (now % 2) ? HIGH : LOW);
                } else {
                    digitalWrite(PIN_LED_STATUS, LOW);
                }
            }

            // 打印调试摘要（每5秒一次）
            static uint32_t debugTick = 0;
            if (pdTICKS_TO_MS(now - debugTick) >= 5000) {
                debugTick = now;
                DebugSerial.println("--- 系统状态摘要 ---");
                DebugSerial.printf("  模式: %d  机械臂: %d  急停: %d\n",
                                  (int)RobotState::getSystemMode(),
                                  (int)RobotState::getArmStatus(),
                                  RobotState::isEmergencyStopped());
                DebugSerial.printf("  识别物体: 类型=0x%02X 数量=%d\n",
                                  RobotState::getDetectedObjectType(),
                                  RobotState::getDetectedObjectCount());
                DebugSerial.printf("  心跳: %u  错误: %u  CRC错误: %u/%u\n",
                                  RobotState::getHeartbeatCount(),
                                  RobotState::getErrorCount(),
                                  groundParser.getCrcErrorCount(),
                                  jetsonParser.getCrcErrorCount());
                DebugSerial.printf("  帧错误: G=%u J=%u  队列溢出: %u\n",
                                  groundParser.getFrameErrorCount(),
                                  jetsonParser.getFrameErrorCount(),
                                  totalRxErrors);
                DebugSerial.println("---");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

// ============================================================
// 初始化
// ============================================================

void setup() {
    // --- 第0步：先启动调试串口，确保后续日志可见 ---
    DebugSerial.begin(DEBUG_BAUD);
    delay(500);  // 等待 USB CDC 稳定

    DebugSerial.println("");
    DebugSerial.println("========================================");
    DebugSerial.println("  ESP32S3 双臂复合机器人主控");
    DebugSerial.println("  v1.0 - PlatformIO + Arduino + FreeRTOS");
    DebugSerial.println("========================================");
    DebugSerial.println("");

    // --- 第1步：初始化 RobotState ---
    RobotState::begin();
    DebugSerial.println("[INIT] RobotState 已初始化");

    if (PIN_GROUND_RX_LED >= 0) {
        pinMode(PIN_GROUND_RX_LED, OUTPUT);
        digitalWrite(PIN_GROUND_RX_LED, GROUND_RX_LED_OFF_LEVEL);
        DebugSerial.printf("[INIT] 地面站接收指示灯 GPIO%d 已初始化\n", PIN_GROUND_RX_LED);
    }

    // --- 第2步：初始化所有串口 ---
    SerialManager::beginAll();

    // --- 第3步：初始化舵机驱动 ---
    BusServoDriver::begin();

    // --- 第4步：初始化机械臂控制器 ---
    ArmController::begin();

    // --- 第5步：创建队列 ---
    systemCommandQueue = xQueueCreate(QUEUE_LEN_SYSTEM_CMD, sizeof(ProtocolFrame));
    armCommandQueue    = xQueueCreate(QUEUE_LEN_ARM_CMD, sizeof(ProtocolFrame));

    if (!systemCommandQueue || !armCommandQueue) {
        DebugSerial.println("[INIT] 错误：队列创建失败！");
        while (1) {
            // 无法恢复，闪烁 LED 指示错误
            if (PIN_LED_STATUS >= 0) {
                digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
            }
            delay(200);
        }
    }
    DebugSerial.printf("[INIT] 系统命令队列: %d 槽, 机械臂命令队列: %d 槽\n",
                       QUEUE_LEN_SYSTEM_CMD, QUEUE_LEN_ARM_CMD);

    // --- 第6步：创建 FreeRTOS 任务 ---
    // 使用 xTaskCreatePinnedToCore 将任务固定到不同核心
    // ESP32S3 有双核：Core 0（Protocol CPU）和 Core 1（Application CPU）

    BaseType_t taskResult;

    // 地面站接收任务 → Core 1
    taskResult = xTaskCreatePinnedToCore(
        taskGroundStation, "GROUND_RX",
        STACK_GROUND_TASK, nullptr,
        PRIO_GROUND_TASK, nullptr, 1);
    if (taskResult != pdPASS) {
        DebugSerial.println("[INIT] 错误：地面站接收任务创建失败！");
    }

    // Jetson 接收任务 → Core 0
    taskResult = xTaskCreatePinnedToCore(
        taskJetson, "JETSON_RX",
        STACK_JETSON_TASK, nullptr,
        PRIO_JETSON_TASK, nullptr, 0);
    if (taskResult != pdPASS) {
        DebugSerial.println("[INIT] 错误：Jetson 接收任务创建失败！");
    }

    // 命令路由任务 → Core 1
    taskResult = xTaskCreatePinnedToCore(
        taskCommandRouter, "CMD_ROUTER",
        STACK_CMD_ROUTER, nullptr,
        PRIO_CMD_ROUTER, nullptr, 1);
    if (taskResult != pdPASS) {
        DebugSerial.println("[INIT] 错误：命令路由任务创建失败！");
    }

    // 机械臂控制任务 → Core 1（优先级最高）
    taskResult = xTaskCreatePinnedToCore(
        taskArm, "ARM",
        STACK_ARM_TASK, nullptr,
        PRIO_ARM_TASK, nullptr, 1);
    if (taskResult != pdPASS) {
        DebugSerial.println("[INIT] 错误：机械臂任务创建失败！");
    }

    // 状态上报任务 → Core 1
    taskResult = xTaskCreatePinnedToCore(
        taskStatus, "STATUS",
        STACK_STATUS_TASK, nullptr,
        PRIO_STATUS_TASK, nullptr, 1);
    if (taskResult != pdPASS) {
        DebugSerial.println("[INIT] 错误：状态上报任务创建失败！");
    }

    DebugSerial.println("[INIT] 所有 FreeRTOS 任务已创建");
    DebugSerial.println("[INIT] 系统初始化完成，进入运行状态");

    // 提示：动作组参数待调
    DebugSerial.println("");
    DebugSerial.println("========================================");
    DebugSerial.println("  系统就绪，等待指令...");
    DebugSerial.println("  重要：机械臂动作组 PWM 参数为占位值，");
    DebugSerial.println("  请在实际使用前进行舵机标定和参数调整。");
    DebugSerial.println("  修改位置: src/arm/ArmActionGroups.cpp");
    DebugSerial.println("========================================");
    DebugSerial.println("");

    // 蜂鸣器短响提示就绪（可选）
    if (PIN_BUZZER >= 0) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(100);
        digitalWrite(PIN_BUZZER, LOW);
    }
}

// ============================================================
// 主循环（仅保活，所有逻辑在 FreeRTOS 任务中）
// ============================================================

void loop() {
    // 不在 loop() 中放置任何串口接收或业务逻辑
    // 所有工作由 FreeRTOS 任务完成
    vTaskDelay(pdMS_TO_TICKS(1000));
}
