/**
 * @file ArmConfig.h
 * @brief 机械臂舵机 ID 配置和结构体定义
 *
 * 双机械臂共 8 个舵机，每个机械臂 4 个。
 * 左臂：底座旋转、肩关节、肘关节、夹爪
 * 右臂：底座旋转、肩关节、肘关节、夹爪
 *
 * 重要：以下舵机 ID 为占位值，后续需要根据实际舵机 ID 修改！
 * 修改方式：直接改下面 constexpr 的值即可，业务代码会自动同步。
 *
 * 众灵舵机 ID 范围：0 ~ 254
 * 广播 ID 255 仅用于特殊控制，不要将单个舵机设为 255
 */

#pragma once

#include <Arduino.h>

// ============================================================
// 左机械臂舵机 ID
// ============================================================
constexpr uint8_t LEFT_SHOULDER_ID = 0;   // Left arm shoulder
constexpr uint8_t LEFT_ELBOW_ID    = 1;   // Left arm elbow
constexpr uint8_t LEFT_GRIPPER_ID  = 2;   // Left arm gripper

// ============================================================
// 右机械臂舵机 ID
// ============================================================
constexpr uint8_t RIGHT_SHOULDER_ID = 3;   // Right arm shoulder
constexpr uint8_t RIGHT_ELBOW_ID    = 4;   // Right arm elbow
constexpr uint8_t RIGHT_GRIPPER_ID  = 5;   // Right arm gripper

// ============================================================
// 舵机参数限制
// ============================================================
constexpr uint16_t SERVO_PWM_UNUSED = 0;   // 动作组占位值：不向该 ID 发送位置命令
constexpr uint16_t SERVO_PWM_MIN = 500;    // PWM 最小值（众灵协议有效范围 0500~2500）
constexpr uint16_t SERVO_PWM_MAX = 2500;   // PWM 最大值
constexpr uint16_t SERVO_PWM_MID = 1500;   // PWM 中位值
constexpr uint16_t SERVO_TIME_MIN = 0;     // 运动时间最小值 (ms)
constexpr uint16_t SERVO_TIME_MAX = 9999;  // 运动时间最大值 (ms)
constexpr uint8_t  SERVO_ID_MIN  = 0;      // 舵机 ID 最小值
constexpr uint8_t  SERVO_ID_MAX  = 254;    // 舵机 ID 最大值
constexpr uint8_t  SERVO_BROADCAST_ID = 255; // 众灵舵机广播 ID（非系统协议广播地址）

// ============================================================
// 机械臂舵机总数
// ============================================================
constexpr uint8_t TOTAL_SERVO_COUNT = 6;

// ============================================================
// 舵机姿态结构体
// ============================================================
struct ServoPose {
    uint8_t  id;      // 舵机 ID (0~254)
    uint16_t pwm;     // 目标 PWM (500~2500)
    uint16_t timeMs;  // 运动时间 (ms, 0~9999)
};

// ============================================================
// 动作步骤结构体
// 一个动作步骤包含多个舵机同时运动的姿态 + 步骤完成后等待时间
// ============================================================
struct ActionStep {
    ServoPose poses[8];     // 本步骤涉及的舵机姿态（最多8个舵机同时运动）
    uint8_t   poseCount;    // 本步骤实际舵机数量
    uint16_t  waitMs;       // 本步骤完成后等待时间 (ms)
};

// ============================================================
// 动作组
// 一个动作组由多个步骤组成，按顺序执行
// ============================================================
constexpr uint8_t MAX_STEPS_PER_ACTION = 16;  // 每个动作组最大步骤数

struct ActionGroup {
    const char* name;                        // 动作组名称（用于调试）
    ActionStep  steps[MAX_STEPS_PER_ACTION]; // 动作步骤数组
    uint8_t     stepCount;                   // 实际步骤数
};
