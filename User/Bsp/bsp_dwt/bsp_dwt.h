/**
 * @file bsp_dwt.h
 * @brief 可跨 Cortex-M 平台复用的 DWT 周期计数接口。
 */

#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint32_t cycle_count;
    } bsp_dwt_time_point_t;

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*reset)(void *device_handle);
        bsp_status_t (*get_cycle_count)(const void *device_handle, uint32_t *cycle_count);
        bsp_status_t (*get_frequency_hz)(const void *device_handle, uint32_t *frequency_hz);
    } bsp_dwt_driver_ops_t;

    typedef struct
    {
        void *device_handle;
        const bsp_dwt_driver_ops_t *driver_ops;
    } bsp_dwt_config_t;

    typedef struct
    {
        void *device_handle;
        const bsp_dwt_driver_ops_t *driver_ops;
        bool is_initialized;
    } bsp_dwt_t;

    bsp_status_t bsp_dwt_init(bsp_dwt_t *me, const bsp_dwt_config_t *config);
    bool bsp_dwt_is_initialized(const bsp_dwt_t *me);
    bsp_status_t bsp_dwt_reset(bsp_dwt_t *me);
    bsp_status_t bsp_dwt_get_cycle_count(const bsp_dwt_t *me, uint32_t *cycle_count);
    bsp_status_t bsp_dwt_get_frequency_hz(const bsp_dwt_t *me, uint32_t *frequency_hz);
    bsp_status_t bsp_dwt_now(const bsp_dwt_t *me, bsp_dwt_time_point_t *time_point);
    bsp_status_t bsp_dwt_elapsed_cycles(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                        uint32_t *elapsed_cycles);
    bsp_status_t bsp_dwt_cycles_to_us(const bsp_dwt_t *me, uint32_t cycle_count, uint32_t *time_us);
    bsp_status_t bsp_dwt_us_to_cycles(const bsp_dwt_t *me, uint32_t time_us, uint32_t *cycle_count);
    bsp_status_t bsp_dwt_delay_us(const bsp_dwt_t *me, uint32_t delay_us);
    bsp_status_t bsp_dwt_has_elapsed_us(const bsp_dwt_t *me, bsp_dwt_time_point_t start_time,
                                        uint32_t duration_us, bool *has_elapsed);

#ifdef __cplusplus
}
#endif

#endif /* BSP_DWT_H */
