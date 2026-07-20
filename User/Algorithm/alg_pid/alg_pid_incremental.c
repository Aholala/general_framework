#include "alg_pid_internal.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define ALG_PID_TWO_PI_F (6.28318530717958647692F)

static AlgPidStatus_t AlgPidIncremental_ValidateConfig(
    const AlgPidIncrementalConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->proportional_gain) ||
        !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) ||
        !isfinite(config->derivative_filter_cutoff_hz) ||
        !isfinite(config->error_deadband) ||
        !isfinite(config->delta_output_min) ||
        !isfinite(config->delta_output_max) ||
        !isfinite(config->output_min) || !isfinite(config->output_max))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    if ((config->derivative_filter_cutoff_hz < 0.0F) ||
        (config->error_deadband < 0.0F) ||
        (config->delta_output_min > config->delta_output_max) ||
        (config->output_min >= config->output_max))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidIncrementalConfig_Init(
    AlgPidIncrementalConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    *config = (AlgPidIncrementalConfig_t){
        .proportional_gain = 0.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .error_deadband = 0.0F,
        .delta_output_min = -FLT_MAX,
        .delta_output_max = FLT_MAX,
        .output_min = -FLT_MAX,
        .output_max = FLT_MAX};
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidIncremental_Init(
    AlgPidIncremental_t *self,
    const AlgPidIncrementalConfig_t *config)
{
    AlgPidStatus_t status;

    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgPidIncremental_ValidateConfig(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    self->terms = (AlgPidTerms_t){0};
    self->previous_error = 0.0F;
    self->second_previous_error = 0.0F;
    self->filtered_derivative_delta = 0.0F;
    self->has_previous_sample = false;
    self->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidIncremental_SetConfig(
    AlgPidIncremental_t *self,
    const AlgPidIncrementalConfig_t *config)
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
    status = AlgPidIncremental_ValidateConfig(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    self->terms.output = AlgPidInternal_Clamp(self->terms.output,
                                              config->output_min,
                                              config->output_max);
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidIncremental_Reset(AlgPidIncremental_t *self,
                                       float initial_output)
{
    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(initial_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    self->terms = (AlgPidTerms_t){0};
    self->terms.output = AlgPidInternal_Clamp(initial_output,
                                              self->config.output_min,
                                              self->config.output_max);
    self->terms.unsaturated_output = self->terms.output;
    self->previous_error = 0.0F;
    self->second_previous_error = 0.0F;
    self->filtered_derivative_delta = 0.0F;
    self->has_previous_sample = false;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidIncremental_Update(AlgPidIncremental_t *self,
                                        float setpoint,
                                        float measurement,
                                        float feedforward_delta,
                                        float delta_time_s,
                                        float *output)
{
    float error;
    float proportional_delta;
    float integral_delta;
    float derivative_delta;
    float filtered_derivative_delta;
    float total_delta;
    float unsaturated_output;
    float saturated_output;
    float time_constant_s;
    float smoothing_factor;

    if ((self == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(setpoint) || !isfinite(measurement) ||
        !isfinite(feedforward_delta) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = AlgPidInternal_ApplyDeadband(setpoint - measurement,
                                         self->config.error_deadband);
    proportional_delta = self->config.proportional_gain *
                         (error - self->previous_error);
    integral_delta = self->config.integral_gain * error * delta_time_s;
    derivative_delta = 0.0F;
    if (self->has_previous_sample)
    {
        derivative_delta = self->config.derivative_gain *
                           (error - (2.0F * self->previous_error) +
                            self->second_previous_error) /
                           delta_time_s;
    }

    filtered_derivative_delta = derivative_delta;
    if (self->has_previous_sample &&
        (self->config.derivative_filter_cutoff_hz > 0.0F))
    {
        time_constant_s = 1.0F /
                          (ALG_PID_TWO_PI_F *
                           self->config.derivative_filter_cutoff_hz);
        smoothing_factor = delta_time_s / (time_constant_s + delta_time_s);
        filtered_derivative_delta = self->filtered_derivative_delta +
                                    smoothing_factor *
                                        (derivative_delta -
                                         self->filtered_derivative_delta);
    }

    total_delta = proportional_delta + integral_delta +
                  filtered_derivative_delta + feedforward_delta;
    total_delta = AlgPidInternal_Clamp(total_delta,
                                       self->config.delta_output_min,
                                       self->config.delta_output_max);
    unsaturated_output = self->terms.output + total_delta;
    saturated_output = AlgPidInternal_Clamp(unsaturated_output,
                                            self->config.output_min,
                                            self->config.output_max);

    if (!isfinite(total_delta) || !isfinite(saturated_output))
    {
        return ALG_PID_STATUS_NUMERICAL_ERROR;
    }

    self->terms.proportional = proportional_delta;
    self->terms.integral = integral_delta;
    self->terms.derivative = filtered_derivative_delta;
    self->terms.feedforward = feedforward_delta;
    self->terms.unsaturated_output = unsaturated_output;
    self->terms.output = saturated_output;
    self->second_previous_error = self->has_previous_sample
                                      ? self->previous_error
                                      : error;
    self->previous_error = error;
    self->filtered_derivative_delta = filtered_derivative_delta;
    self->has_previous_sample = true;
    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

const AlgPidTerms_t *AlgPidIncremental_GetTerms(
    const AlgPidIncremental_t *self)
{
    return ((self != NULL) && self->is_initialized) ? &self->terms : NULL;
}
