/**
 * @file bsp_dwt.c
 * @brief Cortex-M7 DWT cycle counter utility implementation.
 */

#include "bsp_dwt.h"

#include "stm32h723xx.h"

#include <limits.h>

#define BSP_DWT_MICROSECONDS_PER_SECOND (1000000ULL)
#define BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES (UINT32_MAX / 2U)
#define BSP_DWT_LOCK_ACCESS_KEY (0xC5ACCE55UL)

static bsp_status_t bsp_dwt_validate(void)
{
    return bsp_dwt_is_initialized() ? BSP_STATUS_OK : BSP_STATUS_NOT_INITIALIZED;
}

bsp_status_t bsp_dwt_init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = BSP_DWT_LOCK_ACCESS_KEY;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    __DSB();
    __ISB();

    return bsp_dwt_is_initialized() ? BSP_STATUS_OK : BSP_STATUS_UNSUPPORTED;
}

bool bsp_dwt_is_initialized(void)
{
    return ((CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0U) &&
           ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U);
}

bsp_status_t bsp_dwt_reset(void)
{
    const bsp_status_t status = bsp_dwt_validate();

    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    DWT->CYCCNT = 0U;
    __DSB();
    __ISB();
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_get_cycle_count(uint32_t *cycle_count)
{
    const bsp_status_t status = bsp_dwt_validate();

    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    *cycle_count = DWT->CYCCNT;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_get_frequency_hz(uint32_t *frequency_hz)
{
    if (frequency_hz == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (SystemCoreClock == 0U)
    {
        return BSP_STATUS_IO_ERROR;
    }

    *frequency_hz = SystemCoreClock;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_now(bsp_dwt_time_point_t *time_point)
{
    if (time_point == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    return bsp_dwt_get_cycle_count(&time_point->cycle_count);
}

bsp_status_t bsp_dwt_elapsed_cycles(bsp_dwt_time_point_t start_time,
                                    uint32_t *elapsed_cycles)
{
    uint32_t current_cycle_count;
    bsp_status_t status;

    if (elapsed_cycles == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_dwt_get_cycle_count(&current_cycle_count);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    *elapsed_cycles = current_cycle_count - start_time.cycle_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_cycles_to_us(uint32_t cycle_count, uint32_t *time_us)
{
    uint32_t frequency_hz;
    uint64_t converted_time_us;
    bsp_status_t status;

    if (time_us == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_dwt_get_frequency_hz(&frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    converted_time_us = ((uint64_t)cycle_count * BSP_DWT_MICROSECONDS_PER_SECOND) /
                        (uint64_t)frequency_hz;
    if (converted_time_us > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }

    *time_us = (uint32_t)converted_time_us;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_us_to_cycles(uint32_t time_us, uint32_t *cycle_count)
{
    uint32_t frequency_hz;
    uint64_t converted_cycle_count;
    bsp_status_t status;

    if (cycle_count == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_dwt_get_frequency_hz(&frequency_hz);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    converted_cycle_count = ((uint64_t)time_us * (uint64_t)frequency_hz +
                             BSP_DWT_MICROSECONDS_PER_SECOND - 1ULL) /
                            BSP_DWT_MICROSECONDS_PER_SECOND;
    if (converted_cycle_count > UINT32_MAX)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }

    *cycle_count = (uint32_t)converted_cycle_count;
    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_delay_us(uint32_t delay_us)
{
    bsp_dwt_time_point_t start_time;
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    status = bsp_dwt_us_to_cycles(delay_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }

    status = bsp_dwt_now(&start_time);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    do
    {
        status = bsp_dwt_elapsed_cycles(start_time, &elapsed_cycles);
        if (status != BSP_STATUS_OK)
        {
            return status;
        }
    } while (elapsed_cycles < required_cycles);

    return BSP_STATUS_OK;
}

bsp_status_t bsp_dwt_has_elapsed_us(bsp_dwt_time_point_t start_time,
                                    uint32_t duration_us,
                                    bool *has_elapsed)
{
    uint32_t required_cycles;
    uint32_t elapsed_cycles;
    bsp_status_t status;

    if (has_elapsed == NULL)
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    status = bsp_dwt_us_to_cycles(duration_us, &required_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }
    if (required_cycles > BSP_DWT_MAXIMUM_SAFE_INTERVAL_CYCLES)
    {
        return BSP_STATUS_OUT_OF_RANGE;
    }

    status = bsp_dwt_elapsed_cycles(start_time, &elapsed_cycles);
    if (status != BSP_STATUS_OK)
    {
        return status;
    }

    *has_elapsed = elapsed_cycles >= required_cycles;
    return BSP_STATUS_OK;
}
