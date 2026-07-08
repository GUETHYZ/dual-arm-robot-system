/**
 * @file Crc16.h
 * @brief CRC16 校验函数（CRC-16/MODBUS）
 *
 * 多项式：0x8005
 * 初始值：0xFFFF
 * 输入/输出反转：是
 */

#pragma once

#include <Arduino.h>

/**
 * @brief 计算 CRC-16/MODBUS
 * @param data 数据指针
 * @param len  数据长度
 * @return 16 位 CRC 值
 */
uint16_t crc16Compute(const uint8_t* data, size_t len);

/**
 * @brief 验证 CRC
 * @param data 包含 CRC 的完整数据
 * @param len  数据总长度（含 CRC 两字节）
 * @return true = CRC 正确
 */
bool crc16Verify(const uint8_t* data, size_t len);
