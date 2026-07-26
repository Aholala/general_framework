#ifndef BSP_TIMEBASE_H
#define BSP_TIMEBASE_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_timebase bsp_timebase_t;
    typedef struct bsp_timebase_device bsp_timebase_device_t;

    typedef struct
    {
        uint32_t cycle_count;
    } bsp_timebase_time_point_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*reset)(bsp_timebase_t *const me);
        bsp_status_t (*get_cycle_count)(const bsp_timebase_t *const me, uint32_t *cycle_count);
        bsp_status_t (*get_frequency)(const bsp_timebase_t *const me, uint32_t *frequency_hz);
    } bsp_timebase_ops_t;

    struct bsp_timebase
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*reset)(void *device_handle);
        bsp_status_t (*get_cycle_count)(const void *device_handle, uint32_t *cycle_count);
        bsp_status_t (*get_frequency)(const void *device_handle, uint32_t *frequency_hz);
    } bsp_timebase_driver_ops_t;

    struct bsp_timebase_device
    {
        bsp_timebase_t super;
        const bsp_timebase_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_timebase_driver_ops_t *driver_ops;
    } bsp_timebase_config_t;

    bsp_status_t bsp_timebase_init(bsp_timebase_device_t *const me,
                                   const bsp_timebase_config_t *const config);
    bsp_timebase_t *bsp_timebase_as_base(bsp_timebase_device_t *const me);
    bsp_status_t bsp_timebase_reset(bsp_timebase_t *const me);
    bsp_status_t bsp_timebase_get_cycle_count(const bsp_timebase_t *const me,
                                              uint32_t *cycle_count);
    bsp_status_t bsp_timebase_get_frequency(const bsp_timebase_t *const me, uint32_t *frequency_hz);
    bsp_status_t bsp_timebase_now(const bsp_timebase_t *const me,
                                  bsp_timebase_time_point_t *time_point);
    bsp_status_t bsp_timebase_elapsed_cycles(const bsp_timebase_t *const me,
                                             bsp_timebase_time_point_t start_time,
                                             uint32_t *elapsed_cycles);
    bsp_status_t bsp_timebase_cycles_to_us(const bsp_timebase_t *const me, uint32_t cycle_count,
                                           uint32_t *time_us);
    bsp_status_t bsp_timebase_us_to_cycles(const bsp_timebase_t *const me, uint32_t time_us,
                                           uint32_t *cycle_count);
    bsp_status_t bsp_timebase_delay_us(const bsp_timebase_t *const me, uint32_t delay_us);
    bsp_status_t bsp_timebase_has_elapsed_us(const bsp_timebase_t *const me,
                                             bsp_timebase_time_point_t start_time,
                                             uint32_t duration_us, bool *has_elapsed);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIMEBASE_H */
