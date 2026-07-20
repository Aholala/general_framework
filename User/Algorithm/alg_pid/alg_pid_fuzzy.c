#include "alg_pid_internal.h"

#include <math.h>
#include <stddef.h>

static bool AlgPidFuzzy_IsFiniteTable(const float *table, size_t element_count)
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

static float AlgPidFuzzy_InterpolateTable(const float *table,
                                          size_t axis_point_count,
                                          float normalized_error,
                                          float normalized_error_rate)
{
    const float maximum_index = (float)(axis_point_count - 1U);
    const float error_coordinate =
        0.5F * (normalized_error + 1.0F) * maximum_index;
    const float rate_coordinate =
        0.5F * (normalized_error_rate + 1.0F) * maximum_index;
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
    error_upper = (error_lower + 1U < axis_point_count)
                      ? error_lower + 1U
                      : error_lower;
    rate_upper = (rate_lower + 1U < axis_point_count)
                     ? rate_lower + 1U
                     : rate_lower;
    error_fraction = error_coordinate - (float)error_lower;
    rate_fraction = rate_coordinate - (float)rate_lower;

    lower_value = table[(error_lower * axis_point_count) + rate_lower] +
                  rate_fraction *
                      (table[(error_lower * axis_point_count) + rate_upper] -
                       table[(error_lower * axis_point_count) + rate_lower]);
    upper_value = table[(error_upper * axis_point_count) + rate_lower] +
                  rate_fraction *
                      (table[(error_upper * axis_point_count) + rate_upper] -
                       table[(error_upper * axis_point_count) + rate_lower]);
    return lower_value + error_fraction * (upper_value - lower_value);
}

AlgPidStatus_t AlgPidFuzzy_Init(AlgPidFuzzy_t *self,
                                const AlgPidFuzzyConfig_t *config)
{
    size_t table_element_count;
    AlgPidStatus_t status;

    if ((self == NULL) || (config == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if ((config->axis_point_count < 2U) ||
        !isfinite(config->error_normalization) ||
        !isfinite(config->error_rate_normalization) ||
        (config->error_normalization <= 0.0F) ||
        (config->error_rate_normalization <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    if (config->axis_point_count > (SIZE_MAX / config->axis_point_count))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    table_element_count = config->axis_point_count * config->axis_point_count;
    if (!AlgPidFuzzy_IsFiniteTable(config->proportional_adjustment_table,
                                   table_element_count) ||
        !AlgPidFuzzy_IsFiniteTable(config->integral_adjustment_table,
                                   table_element_count) ||
        !AlgPidFuzzy_IsFiniteTable(config->derivative_adjustment_table,
                                   table_element_count))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    status = AlgPid_Init(&self->controller, &config->base_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    self->previous_error = 0.0F;
    self->has_previous_sample = false;
    self->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidFuzzy_Reset(AlgPidFuzzy_t *self,
                                 float measurement,
                                 float initial_output)
{
    AlgPidStatus_t status;

    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }

    status = AlgPid_Reset(&self->controller, measurement, initial_output);
    if (status == ALG_PID_STATUS_OK)
    {
        self->previous_error = 0.0F;
        self->has_previous_sample = false;
    }
    return status;
}

AlgPidStatus_t AlgPidFuzzy_Update(AlgPidFuzzy_t *self,
                                  const AlgPidInput_t *input,
                                  float *output)
{
    float error;
    float error_rate;
    float normalized_error;
    float normalized_error_rate;
    AlgPidStatus_t status;

    if ((self == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->delta_time_s) || (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = input->setpoint - input->measurement;
    error_rate = self->has_previous_sample
                     ? (error - self->previous_error) / input->delta_time_s
                     : 0.0F;
    normalized_error = AlgPidInternal_Clamp(
        error / self->config.error_normalization,
        -1.0F,
        1.0F);
    normalized_error_rate = AlgPidInternal_Clamp(
        error_rate / self->config.error_rate_normalization,
        -1.0F,
        1.0F);

    self->controller.config.proportional_gain =
        self->config.base_config.proportional_gain +
        AlgPidFuzzy_InterpolateTable(
            self->config.proportional_adjustment_table,
            self->config.axis_point_count,
            normalized_error,
            normalized_error_rate);
    self->controller.config.integral_gain =
        self->config.base_config.integral_gain +
        AlgPidFuzzy_InterpolateTable(
            self->config.integral_adjustment_table,
            self->config.axis_point_count,
            normalized_error,
            normalized_error_rate);
    self->controller.config.derivative_gain =
        self->config.base_config.derivative_gain +
        AlgPidFuzzy_InterpolateTable(
            self->config.derivative_adjustment_table,
            self->config.axis_point_count,
            normalized_error,
            normalized_error_rate);

    status = AlgPid_UpdateAdvanced(&self->controller, input, output);
    if (status == ALG_PID_STATUS_OK)
    {
        self->previous_error = error;
        self->has_previous_sample = true;
    }
    return status;
}
