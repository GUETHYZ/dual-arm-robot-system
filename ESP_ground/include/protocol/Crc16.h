/**
 * @file Crc16.h
 * @brief CRC-16/MODBUS 校验实现
 *
 * 算法参数：
 *   - 多项式反向形式：0xA001
 *   - 初始值：0xFFFF
 *   - 输入/输出不反转
 *   - 异或值：0x0000
 *
 * CRC 计算范围：
 *   从帧头 0xAA 开始，到 Data 区最后一个字节为止。
 *   不包括 CRC 自身，也不包括帧尾 0x0D 0x0A。
 */

#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief 计算 CRC-16/MODBUS 校验值
 *
 * @param data   数据缓冲区指针
 * @param length 数据长度（字节）
 * @return uint16_t CRC16 校验值
 */
uint16_t crc16Modbus(const uint8_t *data, size_t length);

/**
 * @brief 验证 CRC16 校验值
 *
 * @param data   数据缓冲区指针（含 CRC 之前的全部数据）
 * @param length 数据长度（字节，不含 CRC 两字节）
 * @param crcLow CRC 低字节
 * @param crcHigh CRC 高字节
 * @return true  校验通过
 * @return false 校验失败
 */
bool crc16Verify(const uint8_t *data, size_t length, uint8_t crcLow, uint8_t crcHigh);
