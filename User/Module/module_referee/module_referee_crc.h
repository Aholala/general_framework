#ifndef MODULE_REFEREE_CRC_H
#define MODULE_REFEREE_CRC_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    uint8_t module_referee_crc8_calculate(const uint8_t *data, size_t data_size);
    uint16_t module_referee_crc16_calculate(const uint8_t *data, size_t data_size);
    bool module_referee_crc8_verify(const uint8_t *data, size_t data_size_with_crc);
    bool module_referee_crc16_verify(const uint8_t *data, size_t data_size_with_crc);
    bool module_referee_crc8_append(uint8_t *data, size_t data_size_with_crc);
    bool module_referee_crc16_append(uint8_t *data, size_t data_size_with_crc);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_REFEREE_CRC_H */
