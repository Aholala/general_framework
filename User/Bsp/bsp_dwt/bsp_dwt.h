/**
 * @file bsp_dwt.h
 * @brief Cortex-M7 DWT cycle counter utility.
 *
 * DWT is a core singleton, so this module intentionally has no device object,
 * virtual table, configuration structure, or dynamic state.
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

    bsp_status_t bsp_dwt_init(void);
    bool bsp_dwt_is_initialized(void);
    bsp_status_t bsp_dwt_reset(void);
    bsp_status_t bsp_dwt_get_cycle_count(uint32_t *cycle_count);
    bsp_status_t bsp_dwt_get_frequency_hz(uint32_t *frequency_hz);
    bsp_status_t bsp_dwt_now(bsp_dwt_time_point_t *time_point);
    bsp_status_t bsp_dwt_elapsed_cycles(bsp_dwt_time_point_t start_time,
                                        uint32_t *elapsed_cycles);
    bsp_status_t bsp_dwt_cycles_to_us(uint32_t cycle_count, uint32_t *time_us);
    bsp_status_t bsp_dwt_us_to_cycles(uint32_t time_us, uint32_t *cycle_count);
    bsp_status_t bsp_dwt_delay_us(uint32_t delay_us);
    bsp_status_t bsp_dwt_has_elapsed_us(bsp_dwt_time_point_t start_time,
                                        uint32_t duration_us,
                                        bool *has_elapsed);

#ifdef __cplusplus
}
#endif

#endif
