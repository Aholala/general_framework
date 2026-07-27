#ifndef BSP_RNG_H
#define BSP_RNG_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_rng bsp_rng_t;
    typedef struct bsp_rng_device bsp_rng_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*get_uint32)(bsp_rng_t *me, uint32_t *value);
        bsp_status_t (*fill)(bsp_rng_t *me, void *data, size_t size);
    } bsp_rng_ops_t;

    struct bsp_rng
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *handle);
        bsp_status_t (*deinit)(void *handle);
        bsp_status_t (*get_uint32)(void *handle, uint32_t *value);
        bsp_status_t (*fill)(void *handle, void *data, size_t size);
    } bsp_rng_driver_ops_t;

    struct bsp_rng_device
    {
        bsp_rng_t super;
        const bsp_rng_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_rng_driver_ops_t *driver_ops;
    } bsp_rng_config_t;

    bsp_status_t bsp_rng_init(bsp_rng_device_t *me, const bsp_rng_config_t *config);
    bsp_rng_t *bsp_rng_as_base(bsp_rng_device_t *me);
    bsp_status_t bsp_rng_get_uint32(bsp_rng_t *me, uint32_t *value);
    bsp_status_t bsp_rng_fill(bsp_rng_t *me, void *data, size_t size);

#ifdef __cplusplus
}
#endif

#endif
