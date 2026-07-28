/**
 * @file module_referee_crc.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 裁判系统 CRC8 和 CRC16 校验计算实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 */

#include "module_referee_crc.h" // 包含 CRC 头文件
#include <stddef.h>             // 提供 NULL

/**
 * @brief 计算 CRC8 校验值
 * @param data 数据缓冲区指针
 * @param data_size 数据大小（字节）
 * @return CRC8 校验值
 * @note 初值 0xFF，多项式 0x8C（CRC-8/SAE-J1850 变体）
 *       算法：逐字节异或，每位右移，遇 1 则异或多项式
 */
uint8_t module_referee_crc8_calculate(const uint8_t *data, size_t data_size)
{
    uint8_t crc = 0xFFU; // 初始值 0xFF
    size_t data_index;   // 数据索引

    // 数据为空但长度非零 => 参数非法，返回 0
    if ((data == NULL) && (data_size != 0U))
    {
        return 0U;
    }
    // 逐字节处理
    for (data_index = 0U; data_index < data_size; ++data_index)
    {
        uint8_t bit_index;
        crc ^= data[data_index]; // 与当前字节异或
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            // 检查最低位，右移并根据情况异或多项式 0x8C
            crc = ((crc & 0x01U) != 0U) ? (uint8_t)((crc >> 1U) ^ 0x8CU) : (uint8_t)(crc >> 1U);
        }
    }
    return crc;
}

/**
 * @brief 计算 CRC16 校验值
 * @param data 数据缓冲区指针
 * @param data_size 数据大小（字节）
 * @return CRC16 校验值
 * @note 初值 0xFFFF，多项式 0x8408（CRC-16/IBM 变体，LSB-first）
 *       算法：逐字节异或，每位右移，遇 1 则异或多项式
 */
uint16_t module_referee_crc16_calculate(const uint8_t *data, size_t data_size)
{
    uint16_t crc = 0xFFFFU; // 初始值 0xFFFF
    size_t data_index;      // 数据索引

    // 数据为空但长度非零 => 参数非法，返回 0
    if ((data == NULL) && (data_size != 0U))
    {
        return 0U;
    }
    // 逐字节处理
    for (data_index = 0U; data_index < data_size; ++data_index)
    {
        uint8_t bit_index;
        crc ^= data[data_index]; // 与当前字节异或
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            // 检查最低位，右移并根据情况异或多项式 0x8408
            crc =
                ((crc & 0x0001U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

/**
 * @brief 验证 CRC8（数据末尾已包含 CRC8 值）
 * @param data 包含 CRC8 的数据缓冲区
 * @param data_size_with_crc 含 CRC8 的总大小
 * @return true=校验通过，false=校验失败
 * @note 对 data 前 (data_size_with_crc - 1) 字节计算 CRC8，
 *       与末尾字节比较
 */
bool module_referee_crc8_verify(const uint8_t *data, size_t data_size_with_crc)
{
    // 数据非空且至少包含 1 字节 CRC
    return (data != NULL) && (data_size_with_crc >= 1U) &&
           (module_referee_crc8_calculate(data, data_size_with_crc - 1U) ==
            data[data_size_with_crc - 1U]);
}

/**
 * @brief 验证 CRC16（数据末尾已包含 CRC16 值）
 * @param data 包含 CRC16 的数据缓冲区
 * @param data_size_with_crc 含 CRC16 的总大小
 * @return true=校验通过，false=校验失败
 * @note 对 data 前 (data_size_with_crc - 2) 字节计算 CRC16，
 *       与末尾 2 字节比较（小端序）
 */
bool module_referee_crc16_verify(const uint8_t *data, size_t data_size_with_crc)
{
    uint16_t expected_crc;

    // 数据非空且至少包含 2 字节 CRC
    if ((data == NULL) || (data_size_with_crc < 2U))
    {
        return false;
    }
    // 从末尾 2 字节读取期望的 CRC（小端序）
    expected_crc =
        (uint16_t)data[data_size_with_crc - 2U] | ((uint16_t)data[data_size_with_crc - 1U] << 8U);
    // 计算除 CRC 字段外数据的 CRC，与期望值比较
    return module_referee_crc16_calculate(data, data_size_with_crc - 2U) == expected_crc;
}

/**
 * @brief 计算并追加 CRC8 到数据末尾
 * @param data 数据缓冲区（末尾预留 1 字节给 CRC8）
 * @param data_size_with_crc 含 CRC8 预留位的总大小
 * @return true=操作成功，false=参数非法
 */
bool module_referee_crc8_append(uint8_t *data, size_t data_size_with_crc)
{
    // 数据非空且至少预留 1 字节
    if ((data == NULL) || (data_size_with_crc < 1U))
    {
        return false;
    }
    // 计算 CRC8 并写入末尾预留位置
    data[data_size_with_crc - 1U] = module_referee_crc8_calculate(data, data_size_with_crc - 1U);
    return true;
}

/**
 * @brief 计算并追加 CRC16 到数据末尾
 * @param data 数据缓冲区（末尾预留 2 字节给 CRC16）
 * @param data_size_with_crc 含 CRC16 预留位的总大小
 * @return true=操作成功，false=参数非法
 * @note CRC 以小端序写入末尾 2 字节
 */
bool module_referee_crc16_append(uint8_t *data, size_t data_size_with_crc)
{
    uint16_t crc;

    // 数据非空且至少预留 2 字节
    if ((data == NULL) || (data_size_with_crc < 2U))
    {
        return false;
    }
    // 计算 CRC16
    crc = module_referee_crc16_calculate(data, data_size_with_crc - 2U);
    // 小端序写入末尾 2 字节
    data[data_size_with_crc - 2U] = (uint8_t)crc;
    data[data_size_with_crc - 1U] = (uint8_t)(crc >> 8U);
    return true;
}