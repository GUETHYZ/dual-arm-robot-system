/**
 * @file TaskConfig.h
 * @brief FreeRTOS 任务配置
 *
 * 集中定义所有任务的栈大小、优先级、队列长度等参数。
 * 修改任务行为时只需修改此文件。
 */

#pragma once

#include <Arduino.h>

// ============================================================
// 任务栈大小（单位：字，ESP32 上 1 字 = 4 字节）
// ============================================================
constexpr uint32_t STACK_GROUND_TASK   = 4096;  // 地面站接收任务
constexpr uint32_t STACK_JETSON_TASK   = 4096;  // Jetson 接收任务
constexpr uint32_t STACK_CMD_ROUTER    = 4096;  // 命令路由任务
constexpr uint32_t STACK_ARM_TASK      = 4096;  // 机械臂控制任务
constexpr uint32_t STACK_STATUS_TASK   = 3072;  // 状态上报任务
constexpr uint32_t STACK_CHASSIS_TASK  = 3072;  // 底盘通信任务（可选）

// ============================================================
// 任务优先级（数字越大优先级越高）
// Arduino 中 loop() 优先级为 1，空闲任务优先级为 0
// ============================================================
constexpr UBaseType_t PRIO_GROUND_TASK  = 3;  // 地面站接收
constexpr UBaseType_t PRIO_JETSON_TASK  = 3;  // Jetson 接收
constexpr UBaseType_t PRIO_CMD_ROUTER   = 3;  // 命令路由
constexpr UBaseType_t PRIO_ARM_TASK     = 4;  // 机械臂控制（最高，保证动作实时性）
constexpr UBaseType_t PRIO_STATUS_TASK  = 1;  // 状态上报（最低）
constexpr UBaseType_t PRIO_CHASSIS_TASK = 2;  // 底盘通信

// ============================================================
// 队列长度
// ============================================================
constexpr UBaseType_t QUEUE_LEN_SYSTEM_CMD  = 16;  // 系统命令队列
constexpr UBaseType_t QUEUE_LEN_ARM_CMD     = 8;   // 机械臂命令队列
constexpr UBaseType_t QUEUE_LEN_STATUS      = 8;   // 状态消息队列

// ============================================================
// 串口接收参数
// ============================================================
constexpr int MAX_RX_BYTES_PER_CYCLE = 96;   // 单次接收循环最大处理字节数
constexpr TickType_t RX_TASK_DELAY   = 1;    // 接收任务让出 CPU 时间 (ms)

// ============================================================
// 状态上报周期 (ms)
// ============================================================
constexpr uint32_t STATUS_REPORT_INTERVAL_MS = 1000;  // 状态上报间隔
constexpr uint32_t HEARTBEAT_INTERVAL_MS     = 500;   // 心跳发送间隔

// ============================================================
// 通信超时 (ms)
// ============================================================
constexpr TickType_t TX_MUTEX_TIMEOUT = 100;  // 串口发送 Mutex 超时
constexpr TickType_t QUEUE_SEND_TIMEOUT = 0;  // 队列发送超时（0 = 不阻塞）

// ============================================================
// 协议缓冲区大小
// ============================================================
constexpr uint8_t PROTOCOL_MAX_PAYLOAD = 64;        // 协议帧最大 payload 长度
constexpr uint8_t PROTOCOL_FRAME_MAX   = 128;       // 协议帧最大总长度（含帧头帧尾）
constexpr uint8_t SERVO_CMD_BUF_SIZE   = 256;       // 舵机命令字符串缓冲区大小
