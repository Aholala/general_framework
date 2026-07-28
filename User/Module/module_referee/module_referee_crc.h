/**
 * @file module_referee_crc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 裁判系统 CRC8 和 CRC16 校验计算头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供 CRC8（初值 0xFF，多项式 0x8C）和 CRC16（初值 0xFFFF，多项式 0x8408）
 *       的计算、验证和追加接口。CRC8 用于帧头校验，CRC16 用于整帧校验。
 */

#ifndef MODULE_REFEREE_CRC_H
#define MODULE_REFEREE_CRC_H

#include <stdbool.h> // 布尔类型
#include <stddef.h>  // size_t
#include <stdint.h>  // uint8_t, uint16_t

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 计算 CRC8 校验值
     * @param data 数据缓冲区指针
     * @param data_size 数据大小（字节）
     * @return CRC8 校验值
     * @note 初值 0xFF，多项式 0x8C（CRC-8/SAE-J1850 变体）
     */
    uint8_t module_referee_crc8_calculate(const uint8_t *data, size_t data_size);

    /**
     * @brief 计算 CRC16 校验值
     * @param data 数据缓冲区指针
     * @param data_size 数据大小（字节）
     * @return CRC16 校验值
     * @note 初值 0xFFFF，多项式 0x8408（CRC-16/IBM 变体，LSB-first）
     */
    uint16_t module_referee_crc16_calculate(const uint8_t *data, size_t data_size);

    /**
     * @brief 验证 CRC8（数据末尾已包含 CRC8 值）
     * @param data 包含 CRC8 的数据缓冲区
     * @param data_size_with_crc 含 CRC8 的总大小
     * @return true=校验通过，false=校验失败
     */
    bool module_referee_crc8_verify(const uint8_t *data, size_t data_size_with_crc);

    /**
     * @brief 验证 CRC16（数据末尾已包含 CRC16 值）
     * @param data 包含 CRC16 的数据缓冲区
     * @param data_size_with_crc 含 CRC16 的总大小
     * @return true=校验通过，false=校验失败
     */
    bool module_referee_crc16_verify(const uint8_t *data, size_t data_size_with_crc);

    /**
     * @brief 计算并追加 CRC8 到数据末尾
     * @param data 数据缓冲区（末尾预留 1 字节给 CRC8）
     * @param data_size_with_crc 含 CRC8 预留位的总大小
     * @return true=操作成功，false=参数非法
     * @note 调用前 data 末尾字节应为占位值，调用后写入 CRC 值
     */
    bool module_referee_crc8_append(uint8_t *data, size_t data_size_with_crc);

    /**
     * @brief 计算并追加 CRC16 到数据末尾
     * @param data 数据缓冲区（末尾预留 2 字节给 CRC16）
     * @param data_size_with_crc 含 CRC16 预留位的总大小
     * @return true=操作成功，false=参数非法
     */
    bool module_referee_crc16_append(uint8_t *data, size_t data_size_with_crc);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_REFEREE_CRC_H */