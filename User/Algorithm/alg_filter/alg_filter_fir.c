#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

alg_filter_status_t alg_filter_fir_init(alg_filter_fir_t *me, const float *coefficients,
                                        float *state_buffer, size_t tap_count)
{
    size_t tap_index;

    if ((me == NULL) || (coefficients == NULL) || (state_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (tap_count == 0U)
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->is_initialized = false;
    for (tap_index = 0U; tap_index < tap_count; ++tap_index)
    {
        if (!isfinite(coefficients[tap_index]))
        {
            return ALG_FILTER_STATUS_OUT_OF_RANGE;
        }
    }

    me->coefficients = coefficients;
    me->state_buffer = state_buffer;
    me->tap_count = tap_count;
    me->is_initialized = true;
    return alg_filter_fir_reset(me);
}

alg_filter_status_t alg_filter_fir_reset(alg_filter_fir_t *me)
{
    size_t tap_index;

    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->tap_count == 0U) || (me->coefficients == NULL) ||
        (me->state_buffer == NULL))
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    for (tap_index = 0U; tap_index < me->tap_count; ++tap_index)
    {
        me->state_buffer[tap_index] = 0.0F;
    }
    me->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_fir_update(alg_filter_fir_t *me, float input, float *output)
{
    size_t coefficient_index;
    size_t state_index;
    float accumulator = 0.0F;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized || (me->tap_count == 0U) || (me->coefficients == NULL) ||
        (me->state_buffer == NULL) || (me->write_index >= me->tap_count))
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->state_buffer[me->write_index] = input;
    state_index = me->write_index;
    for (coefficient_index = 0U; coefficient_index < me->tap_count; ++coefficient_index)
    {
        accumulator += me->coefficients[coefficient_index] * me->state_buffer[state_index];
        state_index = (state_index == 0U) ? (me->tap_count - 1U) : (state_index - 1U);
    }

    me->write_index = (me->write_index + 1U) % me->tap_count;
    if (!isfinite(accumulator))
    {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
    }

    *output = accumulator;
    return ALG_FILTER_STATUS_OK;
}
