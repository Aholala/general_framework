#include "module_referee_crc.h"

#include <stddef.h>

uint8_t module_referee_crc8_calculate(const uint8_t *data, size_t data_size)
{
    uint8_t crc = 0xFFU;
    size_t data_index;

    if ((data == NULL) && (data_size != 0U))
    {
        return 0U;
    }
    for (data_index = 0U; data_index < data_size; ++data_index)
    {
        uint8_t bit_index;
        crc ^= data[data_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            crc = ((crc & 0x01U) != 0U) ? (uint8_t)((crc >> 1U) ^ 0x8CU) : (uint8_t)(crc >> 1U);
        }
    }
    return crc;
}

uint16_t module_referee_crc16_calculate(const uint8_t *data, size_t data_size)
{
    uint16_t crc = 0xFFFFU;
    size_t data_index;

    if ((data == NULL) && (data_size != 0U))
    {
        return 0U;
    }
    for (data_index = 0U; data_index < data_size; ++data_index)
    {
        uint8_t bit_index;
        crc ^= data[data_index];
        for (bit_index = 0U; bit_index < 8U; ++bit_index)
        {
            crc =
                ((crc & 0x0001U) != 0U) ? (uint16_t)((crc >> 1U) ^ 0x8408U) : (uint16_t)(crc >> 1U);
        }
    }
    return crc;
}

bool module_referee_crc8_verify(const uint8_t *data, size_t data_size_with_crc)
{
    return (data != NULL) && (data_size_with_crc >= 1U) &&
           (module_referee_crc8_calculate(data, data_size_with_crc - 1U) ==
            data[data_size_with_crc - 1U]);
}

bool module_referee_crc16_verify(const uint8_t *data, size_t data_size_with_crc)
{
    uint16_t expected_crc;

    if ((data == NULL) || (data_size_with_crc < 2U))
    {
        return false;
    }
    expected_crc =
        (uint16_t)data[data_size_with_crc - 2U] | ((uint16_t)data[data_size_with_crc - 1U] << 8U);
    return module_referee_crc16_calculate(data, data_size_with_crc - 2U) == expected_crc;
}

bool module_referee_crc8_append(uint8_t *data, size_t data_size_with_crc)
{
    if ((data == NULL) || (data_size_with_crc < 1U))
    {
        return false;
    }
    data[data_size_with_crc - 1U] = module_referee_crc8_calculate(data, data_size_with_crc - 1U);
    return true;
}

bool module_referee_crc16_append(uint8_t *data, size_t data_size_with_crc)
{
    uint16_t crc;

    if ((data == NULL) || (data_size_with_crc < 2U))
    {
        return false;
    }
    crc = module_referee_crc16_calculate(data, data_size_with_crc - 2U);
    data[data_size_with_crc - 2U] = (uint8_t)crc;
    data[data_size_with_crc - 1U] = (uint8_t)(crc >> 8U);
    return true;
}
