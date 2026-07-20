#include "alg_filter.h"

#include <math.h>
#include <stddef.h>

static void AlgFilterWindow_Clear(float *buffer, size_t capacity)
{
    size_t index;

    for (index = 0U; index < capacity; ++index)
    {
        buffer[index] = 0.0F;
    }
}

AlgFilterStatus_t AlgFilterMovingAverage_Init(AlgFilterMovingAverage_t *self,
                                              float *sample_buffer,
                                              size_t capacity)
{
    if ((self == NULL) || (sample_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (capacity == 0U)
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->sample_buffer = sample_buffer;
    self->capacity = capacity;
    self->is_initialized = true;
    return AlgFilterMovingAverage_Reset(self);
}

AlgFilterStatus_t AlgFilterMovingAverage_Reset(AlgFilterMovingAverage_t *self)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    AlgFilterWindow_Clear(self->sample_buffer, self->capacity);
    self->sample_count = 0U;
    self->write_index = 0U;
    self->sum = 0.0F;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterMovingAverage_Update(AlgFilterMovingAverage_t *self,
                                                float input,
                                                float *output)
{
    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    if (self->sample_count == self->capacity)
    {
        self->sum -= self->sample_buffer[self->write_index];
    }
    else
    {
        ++self->sample_count;
    }

    self->sample_buffer[self->write_index] = input;
    self->sum += input;
    self->write_index = (self->write_index + 1U) % self->capacity;
    *output = self->sum / (float)self->sample_count;

    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

AlgFilterStatus_t AlgFilterMedian_Init(AlgFilterMedian_t *self,
                                       float *sample_buffer,
                                       float *sort_buffer,
                                       size_t capacity)
{
    if ((self == NULL) || (sample_buffer == NULL) || (sort_buffer == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if ((capacity == 0U) || (sample_buffer == sort_buffer))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->sample_buffer = sample_buffer;
    self->sort_buffer = sort_buffer;
    self->capacity = capacity;
    self->is_initialized = true;
    return AlgFilterMedian_Reset(self);
}

AlgFilterStatus_t AlgFilterMedian_Reset(AlgFilterMedian_t *self)
{
    if (self == NULL)
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }

    AlgFilterWindow_Clear(self->sample_buffer, self->capacity);
    AlgFilterWindow_Clear(self->sort_buffer, self->capacity);
    self->sample_count = 0U;
    self->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

AlgFilterStatus_t AlgFilterMedian_Update(AlgFilterMedian_t *self,
                                         float input,
                                         float *output)
{
    size_t source_index;
    size_t insertion_index;
    float value;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input))
    {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    self->sample_buffer[self->write_index] = input;
    self->write_index = (self->write_index + 1U) % self->capacity;
    if (self->sample_count < self->capacity)
    {
        ++self->sample_count;
    }

    for (source_index = 0U; source_index < self->sample_count; ++source_index)
    {
        self->sort_buffer[source_index] = self->sample_buffer[source_index];
    }

    for (source_index = 1U; source_index < self->sample_count; ++source_index)
    {
        value = self->sort_buffer[source_index];
        insertion_index = source_index;
        while ((insertion_index > 0U) &&
               (self->sort_buffer[insertion_index - 1U] > value))
        {
            self->sort_buffer[insertion_index] = self->sort_buffer[insertion_index - 1U];
            --insertion_index;
        }
        self->sort_buffer[insertion_index] = value;
    }

    if ((self->sample_count % 2U) == 0U)
    {
        const size_t upper_index = self->sample_count / 2U;
        *output = 0.5F * (self->sort_buffer[upper_index - 1U] +
                          self->sort_buffer[upper_index]);
    }
    else
    {
        *output = self->sort_buffer[self->sample_count / 2U];
    }

    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}
