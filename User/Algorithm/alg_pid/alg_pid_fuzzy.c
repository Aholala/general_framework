#include "alg_pid_internal.h"

#include <math.h>
#include <stddef.h>

static bool alg_pid_fuzzy_is_finite_table(const float *table, size_t element_count)
{
    size_t element_index;

    if (table == NULL)
    {
        return false;
    }
    for (element_index = 0U; element_index < element_count; ++element_index)
    {
        if (!isfinite(table[element_index]))
        {
            return false;
        }
    }
    return true;
}

static float alg_pid_fuzzy_interpolate_table(const float *table, size_t axis_point_count,
                                             float normalized_error, float normalized_error_rate)
{
    const float maximum_index = (float)(axis_point_count - 1U);
    const float error_coordinate = 0.5F * (normalized_error + 1.0F) * maximum_index;
    const float rate_coordinate = 0.5F * (normalized_error_rate + 1.0F) * maximum_index;
    size_t error_lower = (size_t)floorf(error_coordinate);
    size_t rate_lower = (size_t)floorf(rate_coordinate);
    size_t error_upper;
    size_t rate_upper;
    float error_fraction;
    float rate_fraction;
    float lower_value;
    float upper_value;

    if (error_lower >= (axis_point_count - 1U))
    {
        error_lower = axis_point_count - 1U;
    }
    if (rate_lower >= (axis_point_count - 1U))
    {
        rate_lower = axis_point_count - 1U;
    }
    error_upper = (error_lower + 1U < axis_point_count) ? error_lower + 1U : error_lower;
    rate_upper = (rate_lower + 1U < axis_point_count) ? rate_lower + 1U : rate_lower;
    error_fraction = error_coordinate - (float)error_lower;
    rate_fraction = rate_coordinate - (float)rate_lower;

    lower_value = table[(error_lower * axis_point_count) + rate_lower] +
                  rate_fraction * (table[(error_lower * axis_point_count) + rate_upper] -
                                   table[(error_lower * axis_point_count) + rate_lower]);
    upper_value = table[(error_upper * axis_point_count) + rate_lower] +
                  rate_fraction * (table[(error_upper * axis_point_count) + rate_upper] -
                                   table[(error_upper * axis_point_count) + rate_lower]);
    return lower_value + error_fraction * (upper_value - lower_value);
}

alg_pid_status_t alg_pid_fuzzy_init(alg_pid_fuzzy_t *me, const alg_pid_fuzzy_config_t *config)
{
    size_t table_element_count;
    alg_pid_status_t status;

    if ((me == NULL) || (config == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    if ((config->axis_point_count < 2U) || !isfinite(config->error_normalization) ||
        !isfinite(config->error_rate_normalization) || (config->error_normalization <= 0.0F) ||
        (config->error_rate_normalization <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    if (config->axis_point_count > (SIZE_MAX / config->axis_point_count))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    table_element_count = config->axis_point_count * config->axis_point_count;
    if (!alg_pid_fuzzy_is_finite_table(config->proportional_adjustment_table,
                                       table_element_count) ||
        !alg_pid_fuzzy_is_finite_table(config->integral_adjustment_table, table_element_count) ||
        !alg_pid_fuzzy_is_finite_table(config->derivative_adjustment_table, table_element_count))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    status = alg_pid_init(&me->controller, &config->base_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    me->config = *config;
    me->previous_error = 0.0F;
    me->has_previous_sample = false;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_fuzzy_reset(alg_pid_fuzzy_t *me, float measurement, float initial_output)
{
    alg_pid_status_t status;

    if (me == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }

    status = alg_pid_reset(&me->controller, measurement, initial_output);
    if (status == ALG_PID_STATUS_OK)
    {
        me->previous_error = 0.0F;
        me->has_previous_sample = false;
    }
    return status;
}

alg_pid_status_t alg_pid_fuzzy_update(alg_pid_fuzzy_t *me, const alg_pid_input_t *input,
                                      float *output)
{
    float error;
    float error_rate;
    float normalized_error;
    float normalized_error_rate;
    alg_pid_status_t status;

    if ((me == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->delta_time_s) || (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = input->setpoint - input->measurement;
    error_rate =
        me->has_previous_sample ? (error - me->previous_error) / input->delta_time_s : 0.0F;
    normalized_error = alg_pid_internal_clamp(error / me->config.error_normalization, -1.0F, 1.0F);
    normalized_error_rate =
        alg_pid_internal_clamp(error_rate / me->config.error_rate_normalization, -1.0F, 1.0F);

    me->controller.config.proportional_gain =
        me->config.base_config.proportional_gain +
        alg_pid_fuzzy_interpolate_table(me->config.proportional_adjustment_table,
                                        me->config.axis_point_count, normalized_error,
                                        normalized_error_rate);
    me->controller.config.integral_gain =
        me->config.base_config.integral_gain +
        alg_pid_fuzzy_interpolate_table(me->config.integral_adjustment_table,
                                        me->config.axis_point_count, normalized_error,
                                        normalized_error_rate);
    me->controller.config.derivative_gain =
        me->config.base_config.derivative_gain +
        alg_pid_fuzzy_interpolate_table(me->config.derivative_adjustment_table,
                                        me->config.axis_point_count, normalized_error,
                                        normalized_error_rate);

    status = alg_pid_update_advanced(&me->controller, input, output);
    if (status == ALG_PID_STATUS_OK)
    {
        me->previous_error = error;
        me->has_previous_sample = true;
    }
    return status;
}
