#ifndef BSP_ENCODER_H
#define BSP_ENCODER_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_encoder bsp_encoder_t;
    typedef struct bsp_encoder_device bsp_encoder_device_t;

    typedef enum
    {
        BSP_ENCODER_DIRECTION_STOPPED = 0,
        BSP_ENCODER_DIRECTION_FORWARD,
        BSP_ENCODER_DIRECTION_REVERSE
    } bsp_encoder_direction_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_encoder_t *const me);
        bsp_status_t (*stop)(bsp_encoder_t *const me);
        bsp_status_t (*set_count)(bsp_encoder_t *const me, int32_t count);
        bsp_status_t (*get_count)(const bsp_encoder_t *const me, int32_t *count);
        bsp_status_t (*get_direction)(const bsp_encoder_t *const me,
                                      bsp_encoder_direction_t *direction);
    } bsp_encoder_ops_t;

    struct bsp_encoder
    {
        bsp_device_t super;
        int32_t previous_count;
        uint32_t counter_modulus;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*start)(void *device_handle);
        bsp_status_t (*stop)(void *device_handle);
        bsp_status_t (*set_count)(void *device_handle, int32_t count);
        bsp_status_t (*get_count)(const void *device_handle, int32_t *count);
        bsp_status_t (*get_direction)(const void *device_handle,
                                      bsp_encoder_direction_t *direction);
    } bsp_encoder_driver_ops_t;

    struct bsp_encoder_device
    {
        bsp_encoder_t super;
        const bsp_encoder_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_encoder_driver_ops_t *driver_ops;
        uint32_t counter_modulus;
    } bsp_encoder_config_t;

    bsp_status_t bsp_encoder_init(bsp_encoder_device_t *const me,
                                  const bsp_encoder_config_t *const config);
    bsp_encoder_t *bsp_encoder_as_base(bsp_encoder_device_t *const me);
    bsp_status_t bsp_encoder_start(bsp_encoder_t *const me);
    bsp_status_t bsp_encoder_stop(bsp_encoder_t *const me);
    bsp_status_t bsp_encoder_reset(bsp_encoder_t *const me);
    bsp_status_t bsp_encoder_set_count(bsp_encoder_t *const me, int32_t count);
    bsp_status_t bsp_encoder_get_count(const bsp_encoder_t *const me, int32_t *count);
    bsp_status_t bsp_encoder_get_delta(bsp_encoder_t *const me, int32_t *count_delta);
    bsp_status_t bsp_encoder_get_direction(const bsp_encoder_t *const me,
                                           bsp_encoder_direction_t *direction);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ENCODER_H */
