/**
 * @file ArmController.h
 * @brief 机械臂控制器
 *
 * 负责执行预设动作组，将动作步骤转换为舵机指令。
 * 执行过程中可响应急停标志。
 */

#pragma once

#include <Arduino.h>
#include "arm/ArmActionGroups.h"

class ArmController {
public:
    /**
     * @brief 初始化机械臂控制器
     */
    static void begin();

    /**
     * @brief 执行指定的动作组
     * @param action 动作索引
     * @return true = 动作组执行完成，false = 被急停中断
     *
     * 注意：此函数内部使用 vTaskDelay，不可在中断中调用。
     * 执行期间会检查急停标志，一旦触发急停则立即返回 false。
     */
    static bool executeAction(ArmAction action);

    /**
     * @brief 触发急停
     * 设置急停标志，当前正在执行的动作组将在下一个步骤前退出。
     */
    static void emergencyStop();

    /**
     * @brief 清除急停标志
     */
    static void clearEmergencyStop();

    /**
     * @brief 查询是否处于急停状态
     */
    static bool isEmergencyStopped();

    /**
     * @brief 停止所有舵机（发送停止指令，不设急停标志）
     */
    static void stopAllServos();

    /**
     * @brief 释放所有舵机扭力
     */
    static void torqueOffAll();

    /**
     * @brief 恢复所有舵机扭力
     */
    static void torqueOnAll();

private:
    /**
     * @brief 执行单个动作步骤
     * @return true = 成功，false = 被急停中断
     */
    static bool executeStep(const ActionStep& step);

    static volatile bool emergencyFlag;  // 急停标志
};
