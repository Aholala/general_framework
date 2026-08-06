/**
 * @file alg_crc.h
 * @brief 与硬件无关的通用 CRC 算法。
 */
#ifndef ALG_CRC_H
#define ALG_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_CRC_BIT_ORDER_MSB_FIRST = 0,
        ALG_CRC_BIT_ORDER_LSB_FIRST
    } alg_crc_bit_order_t;

    /**
     * @brief CRC 参数。
     * @note LSB-first 时 polynomial 应填写反射后的多项式，例如 0x8408。
     */
    typedef struct
    {
        uint8_t width;                    // 8、16 或 32
        uint32_t polynomial;
        uint32_t initial_value;
        uint32_t output_xor;
        alg_crc_bit_order_t bit_order;
    } alg_crc_config_t;

    extern const alg_crc_config_t alg_crc8_0x8c_ff_config;
    extern const alg_crc_config_t alg_crc16_ccitt_false_config;
    extern const alg_crc_config_t alg_crc16_0x8408_ff_config;

    /** @brief 使用给定参数统一计算 CRC。 */
    bool alg_crc_calculate(const alg_crc_config_t *config,
                           const uint8_t *data, size_t data_size,
                           uint32_t *result);

#ifdef __cplusplus
}
#endif
#endif /* ALG_CRC_H */
