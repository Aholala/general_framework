/**
 * @file bsp_watchdog.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 
 * @version 1.0
 * @date 2026-07-27
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#ifndef BSP_WATCHDOG_H
#define BSP_WATCHDOG_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_watchdog bsp_watchdog_t;
    typedef struct bsp_watchdog_device bsp_watchdog_device_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*refresh)(bsp_watchdog_t *const me);
        bsp_status_t (*get_timeout_ms)(const bsp_watchdog_t *const me, uint32_t *timeout_ms);
        bsp_status_t (*get_reset_detected)(const bsp_watchdog_t *const me, bool *reset_detected);
    } bsp_watchdog_ops_t;

    struct bsp_watchdog
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*refresh)(void *device_handle);
        bsp_status_t (*get_timeout_ms)(const void *device_handle, uint32_t *timeout_ms);
        bsp_status_t (*get_reset_detected)(const void *device_handle, bool *reset_detected);
    } bsp_watchdog_driver_ops_t;

    struct bsp_watchdog_device
    {
        bsp_watchdog_t super;
        const bsp_watchdog_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_watchdog_driver_ops_t *driver_ops;
    } bsp_watchdog_config_t;

    bsp_status_t bsp_watchdog_init(bsp_watchdog_device_t *const me,
                                   const bsp_watchdog_config_t *const config);
    bsp_watchdog_t *bsp_watchdog_as_base(bsp_watchdog_device_t *const me);
    bsp_status_t bsp_watchdog_refresh(bsp_watchdog_t *const me);
    bsp_status_t bsp_watchdog_get_timeout_ms(const bsp_watchdog_t *const me, uint32_t *timeout_ms);
    bsp_status_t bsp_watchdog_get_reset_detected(const bsp_watchdog_t *const me,
                                                 bool *reset_detected);

#ifdef __cplusplus
}
#endif

#endif /* BSP_WATCHDOG_H */
