#include "alg_math.h"

#include <float.h>
#include <math.h>

alg_math_status_t alg_math_statistics_init(alg_math_statistics_t *me)
{
    if (me == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    me->sample_count = 0U;
    me->mean = 0.0F;
    me->sum_of_squared_deviations = 0.0F;
    me->minimum = FLT_MAX;
    me->maximum = -FLT_MAX;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_statistics_update(alg_math_statistics_t *me, float sample)
{
    float delta;
    float updated_mean;

    if (me == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(sample) || (me->sample_count == UINT32_MAX))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    ++me->sample_count;
    delta = sample - me->mean;
    updated_mean = me->mean + (delta / (float)me->sample_count);
    me->sum_of_squared_deviations += delta * (sample - updated_mean);
    me->mean = updated_mean;
    me->minimum = fminf(me->minimum, sample);
    me->maximum = fmaxf(me->maximum, sample);
    return (isfinite(me->mean) && isfinite(me->sum_of_squared_deviations))
               ? ALG_MATH_STATUS_OK
               : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_statistics_get_population_variance(const alg_math_statistics_t *me,
                                                              float *variance)
{
    if ((me == NULL) || (variance == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (me->sample_count == 0U)
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *variance = me->sum_of_squared_deviations / (float)me->sample_count;
    *variance = fmaxf(*variance, 0.0F);
    return isfinite(*variance) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_statistics_get_sample_variance(const alg_math_statistics_t *me,
                                                          float *variance)
{
    if ((me == NULL) || (variance == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (me->sample_count < 2U)
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *variance = me->sum_of_squared_deviations / (float)(me->sample_count - 1U);
    *variance = fmaxf(*variance, 0.0F);
    return isfinite(*variance) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_statistics_get_standard_deviation(const alg_math_statistics_t *me,
                                                             bool sample_standard_deviation,
                                                             float *standard_deviation)
{
    float variance;
    alg_math_status_t status;

    if (standard_deviation == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    status = sample_standard_deviation ? alg_math_statistics_get_sample_variance(me, &variance)
                                       : alg_math_statistics_get_population_variance(me, &variance);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    *standard_deviation = sqrtf(variance);
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_array_mean(const float *values, size_t value_count, float *mean)
{
    size_t index;
    float accumulator = 0.0F;

    if ((values == NULL) || (mean == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((value_count == 0U) || !alg_math_is_finite_array(values, value_count))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 0U; index < value_count; ++index)
    {
        accumulator += values[index];
    }
    *mean = accumulator / (float)value_count;
    return isfinite(*mean) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_array_rms(const float *values, size_t value_count, float *rms)
{
    size_t index;
    float accumulator = 0.0F;

    if ((values == NULL) || (rms == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((value_count == 0U) || !alg_math_is_finite_array(values, value_count))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 0U; index < value_count; ++index)
    {
        accumulator += values[index] * values[index];
    }
    *rms = sqrtf(accumulator / (float)value_count);
    return isfinite(*rms) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_interpolate_linear1_d(const float *x_values, const float *y_values,
                                                 size_t point_count, float input,
                                                 bool clamp_to_table, float *output)
{
    size_t index;
    float ratio;

    if ((x_values == NULL) || (y_values == NULL) || (output == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((point_count < 2U) || !isfinite(input) ||
        !alg_math_is_finite_array(x_values, point_count) ||
        !alg_math_is_finite_array(y_values, point_count))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 1U; index < point_count; ++index)
    {
        if (x_values[index] <= x_values[index - 1U])
        {
            return ALG_MATH_STATUS_OUT_OF_RANGE;
        }
    }
    if (input < x_values[0])
    {
        if (!clamp_to_table)
        {
            return ALG_MATH_STATUS_OUT_OF_RANGE;
        }
        *output = y_values[0];
        return ALG_MATH_STATUS_OK;
    }
    if (input > x_values[point_count - 1U])
    {
        if (!clamp_to_table)
        {
            return ALG_MATH_STATUS_OUT_OF_RANGE;
        }
        *output = y_values[point_count - 1U];
        return ALG_MATH_STATUS_OK;
    }
    if (input == x_values[point_count - 1U])
    {
        *output = y_values[point_count - 1U];
        return ALG_MATH_STATUS_OK;
    }
    for (index = 1U; index < point_count; ++index)
    {
        if (input <= x_values[index])
        {
            ratio = (input - x_values[index - 1U]) / (x_values[index] - x_values[index - 1U]);
            *output = y_values[index - 1U] + (ratio * (y_values[index] - y_values[index - 1U]));
            return isfinite(*output) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
        }
    }
    return ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_interpolate_bilinear(float x_ratio, float y_ratio, float value_00,
                                                float value_10, float value_01, float value_11,
                                                float *output)
{
    float lower;
    float upper;

    if (output == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(x_ratio) || !isfinite(y_ratio) || !isfinite(value_00) || !isfinite(value_10) ||
        !isfinite(value_01) || !isfinite(value_11) || (x_ratio < 0.0F) || (x_ratio > 1.0F) ||
        (y_ratio < 0.0F) || (y_ratio > 1.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    lower = value_00 + (x_ratio * (value_10 - value_00));
    upper = value_01 + (x_ratio * (value_11 - value_01));
    *output = lower + (y_ratio * (upper - lower));
    return isfinite(*output) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}
