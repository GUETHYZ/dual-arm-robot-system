/**
 * @file main.cpp
 * @brief 地面站 ESP32S3 主入口
 *
 * 初始化流程：
 *   1. 初始化 DebugSerial（USB 串口）
 *   2. 初始化全局队列和互斥锁
 *   3. 初始化 TJC 串口屏
 *   4. 初始化 LoRa 无线串口
 *   5. 初始化地面站状态管理器
 *   6. 创建 FreeRTOS 任务
 *   7. main_task 删除自身
 *   8. loop() 仅做保活
 *
 * FreeRTOS 任务一览：
 *   - tjcReceiveTask:      轮询 TJC 按钮事件 → 映射 → 入发送队列
 *   - wirelessReceiveTask: 轮询 LoRa 接收 → 解析协议帧 → 处理
 *   - wirelessSendTask:    出队列 → 构造协议帧 → LoRa 发送
 *   - statusTask:          检查心跳超时 → 打印链路状态
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

#include "config/HardwareConfig.h"
#include "config/DeviceConfig.h"
#include "config/TaskConfig.h"
#include "protocol/Protocol.h"
#include "protocol/ProtocolParser.h"
#include "tjc/TjcPanel.h"
#include "wireless/WirelessLink.h"
#include "app/GroundStationState.h"
#include "app/CommandMapper.h"

// ============================================================================
// 全局队列与互斥锁
// ============================================================================

/// 发送命令队列（元素类型：GroundCommand）
QueueHandle_t txCommandQueue = nullptr;

/// LoRa 串口发送互斥锁
SemaphoreHandle_t loraTxMutex = nullptr;

/// 地面站状态互斥锁
SemaphoreHandle_t stateMutex = nullptr;

// ============================================================================
// 前向声明 FreeRTOS 任务函数
// ============================================================================

static void tjcReceiveTask(void *param);
static void wirelessReceiveTask(void *param);
static void wirelessSendTask(void *param);
static void statusTask(void *param);
static void mainTask(void *param);

// ============================================================================
// 辅助函数：创建所有 FreeRTOS 任务
// ============================================================================

static void createTasks() {
    BaseType_t result;

    // TJC 接收任务
    result = xTaskCreate(
        tjcReceiveTask,
        "tjc_rx",
        TASK_STACK_TJC_RX,
        nullptr,
        TASK_PRIO_TJC_RX,
        nullptr
    );
    if (result != pdPASS) {
        DebugSerial.println(F("[ERROR] failed to create tjc_rx task"));
    }

    // LoRa 接收任务
    result = xTaskCreate(
        wirelessReceiveTask,
        "lora_rx",
        TASK_STACK_LORA_RX,
        nullptr,
        TASK_PRIO_LORA_RX,
        nullptr
    );
    if (result != pdPASS) {
        DebugSerial.println(F("[ERROR] failed to create lora_rx task"));
    }

    // LoRa 发送任务
    result = xTaskCreate(
        wirelessSendTask,
        "lora_tx",
        TASK_STACK_LORA_TX,
        nullptr,
        TASK_PRIO_LORA_TX,
        nullptr
    );
    if (result != pdPASS) {
        DebugSerial.println(F("[ERROR] failed to create lora_tx task"));
    }

    // 状态监控任务
    result = xTaskCreate(
        statusTask,
        "status",
        TASK_STACK_STATUS,
        nullptr,
        TASK_PRIO_STATUS,
        nullptr
    );
    if (result != pdPASS) {
        DebugSerial.println(F("[ERROR] failed to create status task"));
    }

    DebugSerial.println(F("[TASK] all tasks created"));
}

// ============================================================================
// main_task：初始化所有子系统，创建任务，然后自删除
// ============================================================================

static void mainTask(void *param) {
    // ---- 1. 初始化调试串口 ----
    DebugSerial.begin(DEBUG_UART_BAUD);
    delay(100);  // 等待串口稳定

    DebugSerial.println(F(""));
    DebugSerial.println(F("============================================"));
    DebugSerial.println(F("  Ground Station - ESP32S3"));
    DebugSerial.println(F("  小型双臂复合机器人系统"));
    DebugSerial.println(F("============================================"));
    DebugSerial.println(F(""));

    // ---- 2. 创建全局队列与互斥锁 ----

    txCommandQueue = xQueueCreate(TX_CMD_QUEUE_LEN, sizeof(GroundCommand));
    if (txCommandQueue == nullptr) {
        DebugSerial.println(F("[ERROR] failed to create txCommandQueue"));
    } else {
        DebugSerial.printf("[INIT] txCommandQueue created (len=%u)\n", TX_CMD_QUEUE_LEN);
    }

    // loraTxMutex 和 stateMutex 由各模块的 begin() 创建（若尚未创建）

    // ---- 3. 初始化 TJC 串口屏 ----
    TjcPanel::begin();

    // ---- 4. 初始化 LoRa 无线串口 ----
    WirelessLink::begin();

    // ---- 5. 初始化地面站状态管理器 ----
    GroundStationStateManager::begin();

    // ---- 6. 创建 FreeRTOS 任务 ----
    createTasks();

    DebugSerial.println(F("[INIT] initialization complete"));
    DebugSerial.println(F("[INIT] main_task deleting itself..."));

    // 删除自身
    vTaskDelete(nullptr);
}

// ============================================================================
// TJC 接收任务：轮询按钮事件 → 映射 → 入发送队列
// ============================================================================

static void tjcReceiveTask(void *param) {
    (void)param;

    DebugSerial.println(F("[TASK] tjc_rx started"));

    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        uint8_t tjcCmd = 0;

        // 非阻塞轮询 TJC 按钮事件（一次处理一帧）
        if (TjcPanel::pollButtonEvent(tjcCmd)) {
            DebugSerial.printf("[TJC RX] button event: %s (0x%02X)\n",
                               CommandMapper::tjcCmdToName(tjcCmd), tjcCmd);

            // 映射为统一协议命令
            GroundCommand cmd;
            cmd.dataLen = 0;

            if (CommandMapper::mapTjcCommand(tjcCmd, cmd)) {
                // 放入发送队列
                if (xQueueSend(txCommandQueue, &cmd, pdMS_TO_TICKS(10)) != pdTRUE) {
                    DebugSerial.println(F("[TJC RX] ERROR: tx queue full, command dropped!"));
                    GroundStationStateManager::incrementErrorCount();
                }
            } else {
                DebugSerial.printf("[TJC RX] WARNING: unmapped command 0x%02X\n", tjcCmd);
            }
        }

        vTaskDelayUntil(&lastWake, TJC_RX_PERIOD_MS);
    }
}

// ============================================================================
// LoRa 接收任务：轮询接收 → 解析协议帧 → 处理
// ============================================================================

static void wirelessReceiveTask(void *param) {
    (void)param;

    DebugSerial.println(F("[TASK] lora_rx started"));

    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        ProtocolFrame frame;

        // 轮询接收协议帧（非阻塞）
        if (WirelessLink::pollFrame(frame)) {
            // 增量接收计数
            GroundStationStateManager::incrementRxCount();

            // 地址过滤：只处理目标地址为 GROUND_ADDR 或 BROADCAST_ADDR 的帧
            if (!isFrameForGround(frame)) {
                // 忽略不相关的帧
                vTaskDelayUntil(&lastWake, LORA_RX_PERIOD_MS);
                continue;
            }

            DebugSerial.printf("[LORA RX] %s from 0x%02X, seq=%u, len=%u\n",
                               funcCodeToString(frame.funcCode),
                               frame.srcAddr, frame.seqNum, frame.dataLen);

            // 根据消息类型分流处理
            switch (frame.funcCode) {
                // ---- 心跳 ----
                case CMD_HEARTBEAT:
                    GroundStationStateManager::updateHeartbeat();
                    DebugSerial.println(F("[LORA RX] heartbeat received"));
                    break;

                // ---- 状态上报 ----
                case CMD_STATUS_REPORT:
                    if (frame.dataLen >= 8) {
                        uint8_t sysMode   = frame.data[0];
                        uint8_t armStat   = frame.data[1];
                        uint8_t emergFlag = frame.data[2];
                        uint8_t objType   = frame.data[3];
                        uint8_t objCount  = frame.data[4];
                        uint8_t hbCount   = frame.data[5];
                        uint8_t errCount  = frame.data[6];
                        // byte7 预留

                        GroundStationStateManager::updateStatusReport(
                            sysMode, armStat, emergFlag,
                            objType, objCount, hbCount, errCount);
                    } else {
                        DebugSerial.printf("[LORA RX] ERROR: STATUS_REPORT data too short (%u bytes)\n",
                                           frame.dataLen);
                        GroundStationStateManager::incrementErrorCount();
                    }
                    break;

                // ---- 物体识别信息 ----
                case CMD_OBJECT_INFO:
                    if (frame.dataLen >= 2) {
                        uint8_t objType  = frame.data[0];
                        uint8_t objCount = frame.data[1];
                        GroundStationStateManager::updateObjectInfo(objType, objCount);
                    } else {
                        DebugSerial.printf("[LORA RX] ERROR: OBJECT_INFO data too short (%u bytes)\n",
                                           frame.dataLen);
                        GroundStationStateManager::incrementErrorCount();
                    }
                    break;

                // ---- ACK ----
                case CMD_ACK:
                    GroundStationStateManager::updateAck(frame.seqNum);
                    break;

                // ---- NACK ----
                case CMD_NACK:
                    if (frame.dataLen >= 1) {
                        GroundStationStateManager::updateNack(frame.seqNum, frame.data[0]);
                    } else {
                        GroundStationStateManager::updateNack(frame.seqNum, 0);
                    }
                    break;

                // ---- 错误上报 ----
                case CMD_ERROR_REPORT:
                    DebugSerial.printf("[LORA RX] ERROR_REPORT: dataLen=%u, data=", frame.dataLen);
                    for (uint8_t i = 0; i < frame.dataLen && i < PROTOCOL_MAX_PAYLOAD_LEN; i++) {
                        DebugSerial.printf("0x%02X ", frame.data[i]);
                    }
                    DebugSerial.println();
                    GroundStationStateManager::incrementErrorCount();
                    break;

                // ---- 未知 ----
                default:
                    DebugSerial.printf("[LORA RX] unhandled funcCode: 0x%02X\n", frame.funcCode);
                    break;
            }
        }

        vTaskDelayUntil(&lastWake, LORA_RX_PERIOD_MS);
    }
}

// ============================================================================
// LoRa 发送任务：出队列 → 构造协议帧 → 发送
// ============================================================================

static void wirelessSendTask(void *param) {
    (void)param;

    DebugSerial.println(F("[TASK] lora_tx started"));

    TickType_t lastWake = xTaskGetTickCount();

    for (;;) {
        GroundCommand cmd;

        // 非阻塞从队列取出命令
        if (xQueueReceive(txCommandQueue, &cmd, pdMS_TO_TICKS(10)) == pdTRUE) {
            // 构造统一协议帧
            ProtocolFrame frame;
            frame.version  = PROTOCOL_VERSION;
            frame.srcAddr  = GROUND_ADDR;       // 地面站自己的地址
            frame.dstAddr  = ESP32_ADDR;         // 目标：车载 ESP32S3
            frame.msgType  = MSG_TYPE_CMD;       // 命令帧
            frame.funcCode = cmd.funcCode;
            frame.seqNum   = nextSeq();          // 分配序列号
            frame.dataLen  = cmd.dataLen;

            // 拷贝数据区（如有）
            if (cmd.dataLen > 0 && cmd.dataLen <= PROTOCOL_MAX_PAYLOAD_LEN) {
                memcpy(frame.data, cmd.data, cmd.dataLen);
            }

            // 通过 LoRa 发送
            if (WirelessLink::sendFrame(frame)) {
                GroundStationStateManager::incrementTxCount();
            } else {
                DebugSerial.println(F("[LORA TX] ERROR: send failed"));
                GroundStationStateManager::incrementErrorCount();
            }
        }

        vTaskDelayUntil(&lastWake, LORA_TX_PERIOD_MS);
    }
}

// ============================================================================
// 状态监控任务：心跳超时检测与串口状态打印
// ============================================================================

static void statusTask(void *param) {
    (void)param;

    DebugSerial.println(F("[TASK] status started"));

    TickType_t lastWake = xTaskGetTickCount();

    // 记录上一次报告的在线状态，仅在状态变化时打印，避免刷屏
    bool lastReportedOnline = false;

    for (;;) {
        // 获取当前状态快照（线程安全）
        GroundStationState snapshot = GroundStationStateManager::getState();

        uint32_t now = millis();
        uint32_t elapsed = (snapshot.lastHeartbeatMs == 0)
                           ? UINT32_MAX
                           : (now - snapshot.lastHeartbeatMs);

        // 判断是否在线：
        //   1. 状态标记为在线
        //   2. 且心跳未超时
        bool currentlyOnline = snapshot.vehicleOnline
                               && (elapsed <= HEARTBEAT_TIMEOUT_MS);

        // 首次收到心跳后 elapsed 正常：如果 vehicleOnline 因为超时变成 false，
        // 我们还需要处理超时情况 —— 如果标记为在线但心跳已超时
        if (snapshot.vehicleOnline && elapsed > HEARTBEAT_TIMEOUT_MS) {
            // 标记离线
            GroundStationStateManager::setVehicleOffline();
            currentlyOnline = false;
        }

        // 只在状态变化时打印
        if (currentlyOnline && !lastReportedOnline) {
            lastReportedOnline = true;
            DebugSerial.println(F("[LINK] vehicle online"));
        } else if (!currentlyOnline && lastReportedOnline) {
            lastReportedOnline = false;
            DebugSerial.println(F("[LINK] vehicle offline"));
        }

        // 预留：后续可在此调用 TjcPanel::refreshReservedStatus()
        // 根据状态更新 TJC 屏幕控件

        vTaskDelayUntil(&lastWake, STATUS_PERIOD_MS);
    }
}

// ============================================================================
// Arduino 标准入口
// ============================================================================

void setup() {
    // 创建一个一次性初始化任务，栈大小给大一些用于串口初始化
    xTaskCreate(
        mainTask,
        "main_task",
        8192,
        nullptr,
        5,  // 最高优先级，确保初始化先完成
        nullptr
    );

    // 启动 FreeRTOS 调度器（Arduino 框架已自动启动）
    // 此处的 setup() 在 mainTask 创建后立即返回
}

void loop() {
    // loop 只做保活，不放置业务逻辑
    vTaskDelay(pdMS_TO_TICKS(1000));
}
