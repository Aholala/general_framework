#include "alg_crc.h"

const alg_crc_config_t alg_crc8_0x8c_ff_config = {
    .width = 8U,
    .polynomial = 0x8CU,
    .initial_value = 0xFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_LSB_FIRST,
};

const alg_crc_config_t alg_crc16_ccitt_false_config = {
    .width = 16U,
    .polynomial = 0x1021U,
    .initial_value = 0xFFFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_MSB_FIRST,
};

const alg_crc_config_t alg_crc16_0x8408_ff_config = {
    .width = 16U,
    .polynomial = 0x8408U,
    .initial_value = 0xFFFFU,
    .output_xor = 0U,
    .bit_order = ALG_CRC_BIT_ORDER_LSB_FIRST,
};

bool alg_crc_calculate(const alg_crc_config_t *config,
                       const uint8_t *data,
                       size_t data_size,
                       uint32_t *result)
{
    uint32_t crc;
    uint32_t mask;
    size_t byte_index;

    if ((config == NULL) || (result == NULL) || ((data == NULL) && (data_size > 0U)) ||
        ((config->width != 8U) && (config->width != 16U) && (config->width != 32U)) ||
        (config->bit_order > ALG_CRC_BIT_ORDER_LSB_FIRST))
    {
        return false;
    }

    mask = (config->width == 32U) ? UINT32_MAX
                                  : ((1UL << config->width) - 1UL);
    crc = config->initial_value & mask;
    for (byte_index = 0U; byte_index < data_size; ++byte_index)
    {
        uint8_t bit_index;
        if (config->bit_order == ALG_CRC_BIT_ORDER_LSB_FIRST)
        {
            crc ^= data[byte_index];
            for (bit_index = 0U; bit_index < 8U; ++bit_index)
            {
                crc = ((crc & 1U) != 0U) ? ((crc >> 1U) ^ config->polynomial)
                                         : (crc >> 1U);
            }
        }
        else
        {
            const uint32_t top_bit = 1UL << (config->width - 1U);
            crc ^= (uint32_t)data[byte_index] << (config->width - 8U);
            for (bit_index = 0U; bit_index < 8U; ++bit_index)
            {
                crc = ((crc & top_bit) != 0U) ? ((crc << 1U) ^ config->polynomial)
                                              : (crc << 1U);
            }
        }
        crc &= mask;
    }
    *result = (crc ^ config->output_xor) & mask;
    return true;
}
