#ifndef BSP_CRC_H
#define BSP_CRC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_crc bsp_crc_t;
    typedef struct bsp_crc_device bsp_crc_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*calculate)(bsp_crc_t *me, const void *data, size_t size,
                                  uint32_t initial_value, uint32_t *result);
    } bsp_crc_ops_t;

    struct bsp_crc
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *handle);
        bsp_status_t (*deinit)(void *handle);
        bsp_status_t (*calculate)(void *handle, const void *data, size_t size,
                                  uint32_t initial_value, uint32_t *result);
    } bsp_crc_driver_ops_t;

    struct bsp_crc_device
    {
        bsp_crc_t super;
        const bsp_crc_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_crc_driver_ops_t *driver_ops;
    } bsp_crc_config_t;

    bsp_status_t bsp_crc_init(bsp_crc_device_t *me, const bsp_crc_config_t *config);
    bsp_crc_t *bsp_crc_as_base(bsp_crc_device_t *me);
    bsp_status_t bsp_crc_calculate(bsp_crc_t *me, const void *data, size_t size,
                                   uint32_t initial_value, uint32_t *result);

#ifdef __cplusplus
}
#endif

#endif
