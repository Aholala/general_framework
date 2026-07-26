#ifndef BSP_TIMER_H
#define BSP_TIMER_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_timer bsp_timer_t;
    typedef struct bsp_timer_device bsp_timer_device_t;
    typedef void (*bsp_timer_callback_t)(bsp_timer_t *const me, void *user_context);

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*start)(bsp_timer_t *const me);
        bsp_status_t (*stop)(bsp_timer_t *const me);
        bsp_status_t (*set_counter)(bsp_timer_t *const me, uint32_t counter_ticks);
        bsp_status_t (*get_counter)(const bsp_timer_t *const me, uint32_t *counter_ticks);
        bsp_status_t (*set_period)(bsp_timer_t *const me, uint32_t period_ticks);
        bsp_status_t (*get_period)(const bsp_timer_t *const me, uint32_t *period_ticks);
        bsp_status_t (*get_frequency)(const bsp_timer_t *const me, uint32_t *frequency_hz);
    } bsp_timer_ops_t;

    struct bsp_timer
    {
        bsp_device_t super;
        bsp_timer_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*start)(void *device_handle);
        bsp_status_t (*stop)(void *device_handle);
        bsp_status_t (*set_counter)(void *device_handle, uint32_t counter_ticks);
        bsp_status_t (*get_counter)(const void *device_handle, uint32_t *counter_ticks);
        bsp_status_t (*set_period)(void *device_handle, uint32_t period_ticks);
        bsp_status_t (*get_period)(const void *device_handle, uint32_t *period_ticks);
        bsp_status_t (*get_frequency)(const void *device_handle, uint32_t *frequency_hz);
    } bsp_timer_driver_ops_t;

    struct bsp_timer_device
    {
        bsp_timer_t super;
        const bsp_timer_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_timer_driver_ops_t *driver_ops;
        bsp_timer_callback_t callback;
        void *user_context;
    } bsp_timer_config_t;

    bsp_status_t bsp_timer_init(bsp_timer_device_t *const me,
                                const bsp_timer_config_t *const config);
    bsp_timer_t *bsp_timer_as_base(bsp_timer_device_t *const me);
    bsp_status_t bsp_timer_set_callback(bsp_timer_t *const me, bsp_timer_callback_t callback,
                                        void *user_context);
    bsp_status_t bsp_timer_start(bsp_timer_t *const me);
    bsp_status_t bsp_timer_stop(bsp_timer_t *const me);
    bsp_status_t bsp_timer_reset(bsp_timer_t *const me);
    bsp_status_t bsp_timer_set_counter(bsp_timer_t *const me, uint32_t counter_ticks);
    bsp_status_t bsp_timer_get_counter(const bsp_timer_t *const me, uint32_t *counter_ticks);
    bsp_status_t bsp_timer_set_period(bsp_timer_t *const me, uint32_t period_ticks);
    bsp_status_t bsp_timer_get_period(const bsp_timer_t *const me, uint32_t *period_ticks);
    bsp_status_t bsp_timer_get_frequency(const bsp_timer_t *const me, uint32_t *frequency_hz);
    void bsp_timer_notify_elapsed(bsp_timer_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_TIMER_H */
