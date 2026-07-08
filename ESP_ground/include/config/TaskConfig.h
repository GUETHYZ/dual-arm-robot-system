/**
 * @file TaskConfig.h
 * @brief FreeRTOS 任务配置、队列长度、任务栈大小、任务优先级集中管理
 *
 * 所有任务相关的配置参数集中在此文件。
 * 修改任务行为时只需修改此处即可。
 */

#pragma once

#include <cstdint>

// ============================================================================
// 任务栈大小（单位：字，4 字节 / 字）
// ============================================================================

/// TJC 接收任务栈大小
constexpr uint32_t TASK_STACK_TJC_RX   = 4096;

/// LoRa 接收任务栈大小
constexpr uint32_t TASK_STACK_LORA_RX  = 4096;

/// LoRa 发送任务栈大小
constexpr uint32_t TASK_STACK_LORA_TX  = 4096;

/// 状态监控任务栈大小
constexpr uint32_t TASK_STACK_STATUS   = 3072;

// ============================================================================
// 任务优先级（数字越大优先级越高）
// ============================================================================

/// TJC 接收任务优先级
constexpr UBaseType_t TASK_PRIO_TJC_RX   = 3;

/// LoRa 接收任务优先级
constexpr UBaseType_t TASK_PRIO_LORA_RX  = 3;

/// LoRa 发送任务优先级
constexpr UBaseType_t TASK_PRIO_LORA_TX  = 3;

/// 状态监控任务优先级（最低，不影响实时通信）
constexpr UBaseType_t TASK_PRIO_STATUS   = 1;

// ============================================================================
// 任务周期（通过 vTaskDelay 调节）
// ============================================================================

/// TJC 接收任务轮询周期（ms）
constexpr TickType_t TJC_RX_PERIOD_MS   = pdMS_TO_TICKS(5);

/// LoRa 接收任务轮询周期（ms）
constexpr TickType_t LORA_RX_PERIOD_MS  = pdMS_TO_TICKS(5);

/// LoRa 发送任务轮询周期（ms）
constexpr TickType_t LORA_TX_PERIOD_MS  = pdMS_TO_TICKS(10);

/// 状态监控任务轮询周期（ms）
constexpr TickType_t STATUS_PERIOD_MS   = pdMS_TO_TICKS(200);

// ============================================================================
// 队列长度
// ============================================================================

/// 发送命令队列长度
constexpr UBaseType_t TX_CMD_QUEUE_LEN = 16;

// ============================================================================
// 心跳与超时
// ============================================================================

/// 心跳超时时间（ms），超过此值判定车载离线
constexpr uint32_t HEARTBEAT_TIMEOUT_MS = 2000;

// ============================================================================
// 接收缓冲区限制
// ============================================================================

/// LoRa 接收单次最大处理字节数（避免长时间占用 CPU）
constexpr size_t LORA_RX_MAX_BYTES_PER_CYCLE = 96;
