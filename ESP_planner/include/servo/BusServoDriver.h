/**
 * @file BusServoDriver.h
 * @brief 众灵总线舵机驱动类
 *
 * 通过串口发送字符串指令控制众灵总线舵机。
 * 所有舵机通信必须通过此类，不直接在业务代码中拼接字符串。
 *
 * 协议格式：
 *   单舵机：#000P1500T1000!
 *   多舵机：{#000P1500T1000!#001P1600T1000!}
 *   读角度：#000PRAD!
 *   停止：  #000PDST!
 *   暂停：  #000PDPT!
 *   继续：  #000PDCT!
 *   释力：  #000PULK!
 *   恢复：  #000PULR!
 */

#pragma once

#include <Arduino.h>
#include "config/ArmConfig.h"
#include "config/TaskConfig.h"

class BusServoDriver {
public:
    /**
     * @brief 初始化舵机驱动（当前为预留接口）
     */
    static void begin();

    // ============================================================
    // 单舵机控制
    // ============================================================

    /**
     * @brief 单个舵机移动到目标位置
     * @param id     舵机 ID (0~254)
     * @param pwm    目标 PWM (500~2500)
     * @param timeMs 运动时间 (ms, 0~9999)
     * @return true = 指令已发送
     */
    static bool moveServo(uint8_t id, uint16_t pwm, uint16_t timeMs);

    /**
     * @brief 多个舵机同步运动（组合指令）
     * @param poses 舵机姿态数组
     * @param count 数组长度
     * @return true = 指令已发送
     */
    static bool moveServos(const ServoPose* poses, uint8_t count);

    // ============================================================
    // 特殊控制
    // ============================================================

    /**
     * @brief 停止单个舵机
     */
    static bool stopServo(uint8_t id);

    /**
     * @brief 停止多个舵机
     */
    static bool stopAll(const uint8_t* ids, uint8_t count);

    /**
     * @brief 暂停单个舵机
     */
    static bool pauseServo(uint8_t id);

    /**
     * @brief 继续单个舵机
     */
    static bool resumeServo(uint8_t id);

    /**
     * @brief 释放单个舵机扭力
     */
    static bool torqueOff(uint8_t id);

    /**
     * @brief 恢复单个舵机扭力
     */
    static bool torqueOn(uint8_t id);

    /**
     * @brief 释放所有舵机扭力
     */
    static bool torqueOffAll();

    /**
     * @brief 恢复所有舵机扭力
     */
    static bool torqueOnAll();

    // ============================================================
    // 读取接口（预留）
    // ============================================================

    /**
     * @brief 读取舵机角度（预留，当前仅发送指令，不解析返回值）
     * @param id 舵机 ID
     * @return true = 读取指令已发送
     */
    static bool readAngle(uint8_t id);

private:
    // ============================================================
    // 参数校验
    // ============================================================
    static bool checkId(uint8_t id);
    static bool checkPwm(uint16_t pwm);
    static bool checkTime(uint16_t timeMs);

    // ============================================================
    // 内部辅助
    // ============================================================

    /**
     * @brief 构建多舵机组合命令字符串
     * @param poses  舵机姿态数组
     * @param count  数量
     * @param buffer 输出缓冲区
     * @param bufSize 缓冲区大小
     * @return 构建的字符串长度，0 表示失败
     */
    static size_t buildMultiServoCmd(const ServoPose* poses, uint8_t count,
                                     char* buffer, size_t bufSize);

    /**
     * @brief 通过 SerialManager 发送舵机指令
     */
    static bool sendCommand(const char* cmd);

    // 命令字符串缓冲区，避免每次动态分配
    static char cmdBuffer[SERVO_CMD_BUF_SIZE];
};
