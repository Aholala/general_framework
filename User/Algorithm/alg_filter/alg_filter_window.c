#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

static void alg_filter_window_clear(float *buffer, size_t capacity)
{
    size_t index;

    for (index = 0U; index < capacity; ++index)
    {
        buffer[index] = 0.0F;
    }
}

alg_filter_status_t alg_filter_moving_average_init(alg_filter_moving_average_t *me,
                                                   float *sample_buffer, size_t capacity)
{
    if ((me == NULL) || (sample_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (capacity == 0U)
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->sample_buffer = sample_buffer;
    me->capacity = capacity;
    me->is_initialized = true;
    return alg_filter_moving_average_reset(me);
}

alg_filter_status_t alg_filter_moving_average_reset(alg_filter_moving_average_t *me)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    alg_filter_window_clear(me->sample_buffer, me->capacity);
    me->sample_count = 0U;
    me->write_index = 0U;
    me->sum = 0.0F;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_moving_average_update(alg_filter_moving_average_t *me, float input,
                                                     float *output)
{
    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (me->sample_count == me->capacity)
    {
        me->sum -= me->sample_buffer[me->write_index];
    }
    else
    {
        ++me->sample_count;
    }

    me->sample_buffer[me->write_index] = input;
    me->sum += input;
    me->write_index = (me->write_index + 1U) % me->capacity;
    *output = me->sum / (float)me->sample_count;

    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

alg_filter_status_t alg_filter_median_init(alg_filter_median_t *me, float *sample_buffer,
                                           float *sort_buffer, size_t capacity)
{
    if ((me == NULL) || (sample_buffer == NULL) || (sort_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if ((capacity == 0U) || (sample_buffer == sort_buffer))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->sample_buffer = sample_buffer;
    me->sort_buffer = sort_buffer;
    me->capacity = capacity;
    me->is_initialized = true;
    return alg_filter_median_reset(me);
}

alg_filter_status_t alg_filter_median_reset(alg_filter_median_t *me)
{
    if (me == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    alg_filter_window_clear(me->sample_buffer, me->capacity);
    alg_filter_window_clear(me->sort_buffer, me->capacity);
    me->sample_count = 0U;
    me->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

alg_filter_status_t alg_filter_median_update(alg_filter_median_t *me, float input, float *output)
{
    size_t source_index;
    size_t insertion_index;
    float value;

    if ((me == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->sample_buffer[me->write_index] = input;
    me->write_index = (me->write_index + 1U) % me->capacity;
    if (me->sample_count < me->capacity)
    {
        ++me->sample_count;
    }

    for (source_index = 0U; source_index < me->sample_count; ++source_index)
    {
        me->sort_buffer[source_index] = me->sample_buffer[source_index];
    }

    for (source_index = 1U; source_index < me->sample_count; ++source_index)
    {
        value = me->sort_buffer[source_index];
        insertion_index = source_index;
        while ((insertion_index > 0U) && (me->sort_buffer[insertion_index - 1U] > value))
        {
            me->sort_buffer[insertion_index] = me->sort_buffer[insertion_index - 1U];
            --insertion_index;
        }
        me->sort_buffer[insertion_index] = value;
    }

    if ((me->sample_count % 2U) == 0U)
    {
        const size_t upper_index = me->sample_count / 2U;
        *output = 0.5F * (me->sort_buffer[upper_index - 1U] + me->sort_buffer[upper_index]);
    }
    else
    {
        *output = me->sort_buffer[me->sample_count / 2U];
    }

    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}
