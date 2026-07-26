#include "alg_math.h"

#include <float.h>
#include <math.h>

bool alg_math_is_finite_array(const float *values, size_t value_count)
{
    size_t index;

    if ((values == NULL) && (value_count > 0U))
    {
        return false;
    }
    for (index = 0U; index < value_count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

alg_math_status_t alg_math_clamp(float value, float lower_limit, float upper_limit, float *result)
{
    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(value) || !isfinite(lower_limit) || !isfinite(upper_limit) ||
        (lower_limit > upper_limit))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *result = fminf(fmaxf(value, lower_limit), upper_limit);
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_lerp(float start, float end, float ratio, float *result)
{
    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(start) || !isfinite(end) || !isfinite(ratio))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *result = start + (ratio * (end - start));
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_map_range(float value, float input_minimum, float input_maximum,
                                     float output_minimum, float output_maximum, bool clamp_output,
                                     float *result)
{
    float ratio;

    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(value) || !isfinite(input_minimum) || !isfinite(input_maximum) ||
        !isfinite(output_minimum) || !isfinite(output_maximum) || (input_minimum >= input_maximum))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    ratio = (value - input_minimum) / (input_maximum - input_minimum);
    if (clamp_output)
    {
        ratio = fminf(fmaxf(ratio, 0.0F), 1.0F);
    }
    *result = output_minimum + (ratio * (output_maximum - output_minimum));
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_apply_deadband(float value, float deadband, bool rescale_output,
                                          float *result)
{
    float magnitude;

    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(value) || !isfinite(deadband) || (deadband < 0.0F) || (deadband >= 1.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    magnitude = fabsf(value);
    if (magnitude <= deadband)
    {
        *result = 0.0F;
    }
    else if (rescale_output)
    {
        *result = copysignf((magnitude - deadband) / (1.0F - deadband), value);
    }
    else
    {
        *result = value;
    }
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_wrap(float value, float lower_bound, float upper_bound, float *result)
{
    float width;
    float offset;

    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(value) || !isfinite(lower_bound) || !isfinite(upper_bound) ||
        (lower_bound >= upper_bound))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    width = upper_bound - lower_bound;
    offset = fmodf(value - lower_bound, width);
    if (offset < 0.0F)
    {
        offset += width;
    }
    *result = lower_bound + offset;
    if (*result >= upper_bound)
    {
        *result = lower_bound;
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_wrap_angle_pi(float angle_rad, float *result_rad)
{
    return alg_math_wrap(angle_rad, -ALG_MATH_PI_F, ALG_MATH_PI_F, result_rad);
}

alg_math_status_t alg_math_angle_difference(float target_rad, float current_rad,
                                            float *difference_rad)
{
    if (!isfinite(target_rad) || !isfinite(current_rad))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    return alg_math_wrap_angle_pi(target_rad - current_rad, difference_rad);
}

float alg_math_degrees_to_radians(float angle_deg)
{
    return angle_deg * ALG_MATH_DEG_TO_RAD_F;
}

float alg_math_radians_to_degrees(float angle_rad)
{
    return angle_rad * ALG_MATH_RAD_TO_DEG_F;
}

alg_math_status_t alg_math_safe_sqrt(float value, float *result)
{
    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(value) || (value < 0.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *result = sqrtf(value);
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_safe_divide(float numerator, float denominator,
                                       float minimum_denominator, float *result)
{
    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(numerator) || !isfinite(denominator) || !isfinite(minimum_denominator) ||
        (minimum_denominator <= 0.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    if (fabsf(denominator) < minimum_denominator)
    {
        return ALG_MATH_STATUS_SINGULAR;
    }
    *result = numerator / denominator;
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}
