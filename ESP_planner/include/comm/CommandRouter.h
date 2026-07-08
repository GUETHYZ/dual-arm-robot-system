/**
 * @file CommandRouter.h
 * @brief 命令路由任务
 *
 * 从系统命令队列取命令，根据功能码分类处理：
 * - 小车底盘类命令 → 转发给 STM32
 * - 机械臂类命令   → 投递到机械臂命令队列
 * - 系统类命令     → 本地处理（ACK/NACK/心跳/状态查询/急停）
 * - Jetson 识别信息 → 更新 RobotState，转发给地面站
 */

#pragma once

#include <Arduino.h>
#include "protocol/Protocol.h"

/**
 * @brief 命令路由任务函数
 * @param param 未使用
 */
void taskCommandRouter(void* param);

/**
 * @brief 处理单条协议帧命令
 * @param frame 收到的协议帧
 */
void routeCommand(const ProtocolFrame& frame);
