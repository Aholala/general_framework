#include "alg_pid_internal.h"

#include <float.h>
#include <math.h>
#include <stddef.h>

#define ALG_PID_TWO_PI_F (6.28318530717958647692F)

bool AlgPidInternal_IsFinite(float value)
{
    return isfinite(value);
}

float AlgPidInternal_Clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

float AlgPidInternal_ApplyDeadband(float value, float deadband)
{
    return (fabsf(value) <= deadband) ? 0.0F : value;
}

AlgPidStatus_t AlgPidInternal_ValidateConfig(const AlgPidConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(config->proportional_gain) ||
        !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) ||
        !isfinite(config->setpoint_weight) ||
        !isfinite(config->derivative_setpoint_weight) ||
        !isfinite(config->velocity_feedforward_gain) ||
        !isfinite(config->acceleration_feedforward_gain) ||
        !isfinite(config->derivative_filter_cutoff_hz) ||
        !isfinite(config->error_deadband) ||
        !isfinite(config->integral_separation_threshold) ||
        !isfinite(config->integral_min) || !isfinite(config->integral_max) ||
        !isfinite(config->output_min) || !isfinite(config->output_max) ||
        !isfinite(config->back_calculation_gain))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    if ((config->setpoint_weight < 0.0F) || (config->setpoint_weight > 1.0F) ||
        (config->derivative_setpoint_weight < 0.0F) ||
        (config->derivative_setpoint_weight > 1.0F) ||
        (config->derivative_filter_cutoff_hz < 0.0F) ||
        (config->error_deadband < 0.0F) ||
        (config->integral_separation_threshold < 0.0F) ||
        (config->integral_min > config->integral_max) ||
        (config->output_min >= config->output_max) ||
        (config->back_calculation_gain < 0.0F) ||
        (config->anti_windup_mode > ALG_PID_ANTI_WINDUP_BACK_CALCULATION) ||
        (config->derivative_mode > ALG_PID_DERIVATIVE_ON_MEASUREMENT))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidConfig_Init(AlgPidConfig_t *config)
{
    if (config == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    *config = (AlgPidConfig_t){
        .proportional_gain = 0.0F,
        .integral_gain = 0.0F,
        .derivative_gain = 0.0F,
        .setpoint_weight = 1.0F,
        .derivative_setpoint_weight = 0.0F,
        .velocity_feedforward_gain = 0.0F,
        .acceleration_feedforward_gain = 0.0F,
        .derivative_filter_cutoff_hz = 0.0F,
        .error_deadband = 0.0F,
        .integral_separation_threshold = 0.0F,
        .integral_min = -FLT_MAX,
        .integral_max = FLT_MAX,
        .output_min = -FLT_MAX,
        .output_max = FLT_MAX,
        .back_calculation_gain = 0.0F,
        .anti_windup_mode = ALG_PID_ANTI_WINDUP_CLAMPING,
        .derivative_mode = ALG_PID_DERIVATIVE_ON_MEASUREMENT};
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPid_Init(AlgPid_t *self, const AlgPidConfig_t *config)
{
    AlgPidStatus_t status;

    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgPidInternal_ValidateConfig(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    self->terms = (AlgPidTerms_t){0};
    self->previous_error = 0.0F;
    self->previous_setpoint = 0.0F;
    self->previous_measurement = 0.0F;
    self->filtered_derivative = 0.0F;
    self->has_previous_sample = false;
    self->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidPosition_Init(AlgPidPosition_t *self,
                                  const AlgPidConfig_t *config)
{
    return AlgPid_Init(self, config);
}

AlgPidStatus_t AlgPidVelocity_Init(AlgPidVelocity_t *self,
                                  const AlgPidConfig_t *config)
{
    return AlgPid_Init(self, config);
}

AlgPidStatus_t AlgPid_SetConfig(AlgPid_t *self,
                               const AlgPidConfig_t *config)
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

    status = AlgPidInternal_ValidateConfig(config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    self->terms.integral = AlgPidInternal_Clamp(self->terms.integral,
                                                config->integral_min,
                                                config->integral_max);
    self->terms.output = AlgPidInternal_Clamp(self->terms.output,
                                              config->output_min,
                                              config->output_max);
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPid_Reset(AlgPid_t *self,
                            float measurement,
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
    if (!isfinite(measurement) || !isfinite(initial_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    self->terms = (AlgPidTerms_t){0};
    self->terms.integral = AlgPidInternal_Clamp(initial_output,
                                                self->config.integral_min,
                                                self->config.integral_max);
    self->terms.unsaturated_output = self->terms.integral;
    self->terms.output = AlgPidInternal_Clamp(initial_output,
                                              self->config.output_min,
                                              self->config.output_max);
    self->previous_error = 0.0F;
    self->previous_setpoint = measurement;
    self->previous_measurement = measurement;
    self->filtered_derivative = 0.0F;
    self->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPid_TrackOutput(AlgPid_t *self,
                                  float setpoint,
                                  float measurement,
                                  float feedforward,
                                  float tracked_output)
{
    float error;
    float proportional;
    float integral;

    if (self == NULL)
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(setpoint) || !isfinite(measurement) ||
        !isfinite(feedforward) || !isfinite(tracked_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = AlgPidInternal_ApplyDeadband(setpoint - measurement,
                                         self->config.error_deadband);
    proportional = (error == 0.0F)
                       ? 0.0F
                       : self->config.proportional_gain *
                             ((self->config.setpoint_weight * setpoint) - measurement);
    integral = tracked_output - proportional - feedforward;
    integral = AlgPidInternal_Clamp(integral,
                                    self->config.integral_min,
                                    self->config.integral_max);

    self->terms.proportional = proportional;
    self->terms.integral = integral;
    self->terms.derivative = 0.0F;
    self->terms.feedforward = feedforward;
    self->terms.unsaturated_output = proportional + integral + feedforward;
    self->terms.output = AlgPidInternal_Clamp(self->terms.unsaturated_output,
                                              self->config.output_min,
                                              self->config.output_max);
    self->previous_error = error;
    self->previous_setpoint = setpoint;
    self->previous_measurement = measurement;
    self->filtered_derivative = 0.0F;
    self->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPid_Update(AlgPid_t *self,
                             float setpoint,
                             float measurement,
                             float delta_time_s,
                             float *output)
{
    const AlgPidInput_t input = {
        .setpoint = setpoint,
        .measurement = measurement,
        .setpoint_rate_per_s = 0.0F,
        .setpoint_acceleration_per_s2 = 0.0F,
        .additional_feedforward = 0.0F,
        .delta_time_s = delta_time_s};

    return AlgPid_UpdateAdvanced(self, &input, output);
}

AlgPidStatus_t AlgPid_UpdateAdvanced(AlgPid_t *self,
                                     const AlgPidInput_t *input,
                                     float *output)
{
    float error;
    float control_error;
    float proportional;
    float integral_candidate;
    float derivative_signal;
    float filtered_derivative;
    float derivative;
    float feedforward;
    float unsaturated_output;
    float saturated_output;
    float time_constant_s;
    float smoothing_factor;
    bool integration_enabled;
    bool saturation_pushes_with_error;

    if ((self == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->setpoint_rate_per_s) ||
        !isfinite(input->setpoint_acceleration_per_s2) ||
        !isfinite(input->additional_feedforward) ||
        !isfinite(input->delta_time_s) || (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    error = input->setpoint - input->measurement;
    control_error = AlgPidInternal_ApplyDeadband(error,
                                                 self->config.error_deadband);
    proportional = (control_error == 0.0F)
                       ? 0.0F
                       : self->config.proportional_gain *
                             ((self->config.setpoint_weight * input->setpoint) -
                              input->measurement);

    integration_enabled =
        (self->config.integral_separation_threshold <= 0.0F) ||
        (fabsf(error) <= self->config.integral_separation_threshold);
    integral_candidate = self->terms.integral;
    if (integration_enabled)
    {
        integral_candidate += self->config.integral_gain * control_error *
                              input->delta_time_s;
    }
    integral_candidate = AlgPidInternal_Clamp(integral_candidate,
                                               self->config.integral_min,
                                               self->config.integral_max);

    derivative_signal = 0.0F;
    if (self->has_previous_sample)
    {
        if (self->config.derivative_mode == ALG_PID_DERIVATIVE_ON_MEASUREMENT)
        {
            derivative_signal = -(input->measurement - self->previous_measurement) /
                                input->delta_time_s;
        }
        else
        {
            derivative_signal =
                (((self->config.derivative_setpoint_weight * input->setpoint) -
                  input->measurement) -
                 ((self->config.derivative_setpoint_weight * self->previous_setpoint) -
                  self->previous_measurement)) /
                input->delta_time_s;
        }
    }

    filtered_derivative = derivative_signal;
    if (self->has_previous_sample &&
        (self->config.derivative_filter_cutoff_hz > 0.0F))
    {
        time_constant_s = 1.0F /
                          (ALG_PID_TWO_PI_F *
                           self->config.derivative_filter_cutoff_hz);
        smoothing_factor = input->delta_time_s /
                           (time_constant_s + input->delta_time_s);
        filtered_derivative = self->filtered_derivative +
                              smoothing_factor *
                                  (derivative_signal - self->filtered_derivative);
    }
    derivative = self->config.derivative_gain * filtered_derivative;
    feedforward = (self->config.velocity_feedforward_gain *
                   input->setpoint_rate_per_s) +
                  (self->config.acceleration_feedforward_gain *
                   input->setpoint_acceleration_per_s2) +
                  input->additional_feedforward;

    unsaturated_output = proportional + integral_candidate + derivative + feedforward;
    saturated_output = AlgPidInternal_Clamp(unsaturated_output,
                                            self->config.output_min,
                                            self->config.output_max);

    if (self->config.anti_windup_mode == ALG_PID_ANTI_WINDUP_CLAMPING)
    {
        saturation_pushes_with_error =
            ((unsaturated_output > self->config.output_max) &&
             (control_error > 0.0F)) ||
            ((unsaturated_output < self->config.output_min) &&
             (control_error < 0.0F));
        if (saturation_pushes_with_error)
        {
            integral_candidate = self->terms.integral;
            unsaturated_output = proportional + integral_candidate + derivative +
                                 feedforward;
            saturated_output = AlgPidInternal_Clamp(unsaturated_output,
                                                    self->config.output_min,
                                                    self->config.output_max);
        }
    }
    else if (self->config.anti_windup_mode ==
             ALG_PID_ANTI_WINDUP_BACK_CALCULATION)
    {
        integral_candidate += self->config.back_calculation_gain *
                              (saturated_output - unsaturated_output) *
                              input->delta_time_s;
        integral_candidate = AlgPidInternal_Clamp(integral_candidate,
                                                   self->config.integral_min,
                                                   self->config.integral_max);
        unsaturated_output = proportional + integral_candidate + derivative +
                             feedforward;
        saturated_output = AlgPidInternal_Clamp(unsaturated_output,
                                                self->config.output_min,
                                                self->config.output_max);
    }

    if (!isfinite(proportional) || !isfinite(integral_candidate) ||
        !isfinite(derivative) || !isfinite(feedforward) ||
        !isfinite(unsaturated_output) || !isfinite(saturated_output))
    {
        return ALG_PID_STATUS_NUMERICAL_ERROR;
    }

    self->terms.proportional = proportional;
    self->terms.integral = integral_candidate;
    self->terms.derivative = derivative;
    self->terms.feedforward = feedforward;
    self->terms.unsaturated_output = unsaturated_output;
    self->terms.output = saturated_output;
    self->previous_error = control_error;
    self->previous_setpoint = input->setpoint;
    self->previous_measurement = input->measurement;
    self->filtered_derivative = filtered_derivative;
    self->has_previous_sample = true;
    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

const AlgPidTerms_t *AlgPid_GetTerms(const AlgPid_t *self)
{
    return ((self != NULL) && self->is_initialized) ? &self->terms : NULL;
}
