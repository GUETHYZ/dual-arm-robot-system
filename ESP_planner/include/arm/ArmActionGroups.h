/**
 * @file ArmActionGroups.h
 * @brief 机械臂预设动作组定义
 *
 * 所有动作组的 PWM 参数集中在此文件，便于后续现场调参。
 * 不需要逆运动学，不需要轨迹规划，仅使用预设动作组。
 *
 * PWM 参考值（示例）：
 *   0500 = 极限位置 A
 *   1500 = 中位（安全默认值）
 *   2500 = 极限位置 B
 *
 * 实际值需要连接舵机后现场标定！
 */

#pragma once

#include <Arduino.h>
#include "config/ArmConfig.h"

// ============================================================
// 动作组索引枚举
// ============================================================
enum class ArmAction : uint8_t {
    HOME = 0,          // 双臂复位
    LEFT_PICK,         // 左臂夹取
    LEFT_PLACE,        // 左臂放下
    RIGHT_PICK,        // 右臂夹取
    RIGHT_PLACE,       // 右臂放下
    BOTH_PICK,         // 双臂夹取
    BOTH_PLACE,        // 双臂放下
    STOP_ALL,          // 停止所有舵机
    EMERGENCY_STOP,    // 急停（先停止再释力）
    TORQUE_OFF_ALL,    // 释放所有扭力
    TORQUE_ON_ALL,     // 恢复所有扭力

    ACTION_COUNT       // 动作组总数（用于数组大小）
};

// ============================================================
// 动作组数据声明
// 定义在 src/arm/ArmActionGroups.cpp
// ============================================================
extern const ActionGroup actionGroups[];
