#include "alg_pid_internal.h"

#include <math.h>
#include <stddef.h>

AlgPidStatus_t AlgPidCascade_Init(AlgPidCascade_t *self,
                                  const AlgPidCascadeConfig_t *config)
{
    AlgPidStatus_t status;

    if ((self == NULL) || (config == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    if ((config->position_loop_divider == 0U) ||
        !isfinite(config->velocity_setpoint_min) ||
        !isfinite(config->velocity_setpoint_max) ||
        (config->velocity_setpoint_min >= config->velocity_setpoint_max))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    status = AlgPidPosition_Init(&self->position_controller,
                                 &config->position_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }
    status = AlgPidVelocity_Init(&self->velocity_controller,
                                 &config->velocity_config);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->position_loop_divider = config->position_loop_divider;
    self->position_loop_counter = config->position_loop_divider - 1U;
    self->position_elapsed_time_s = 0.0F;
    self->velocity_setpoint_min = config->velocity_setpoint_min;
    self->velocity_setpoint_max = config->velocity_setpoint_max;
    self->velocity_setpoint = 0.0F;
    self->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidCascade_Reset(AlgPidCascade_t *self,
                                   float position_measurement,
                                   float velocity_measurement,
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
    if (!isfinite(position_measurement) || !isfinite(velocity_measurement) ||
        !isfinite(initial_output))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    status = AlgPid_Reset(&self->position_controller,
                          position_measurement,
                          0.0F);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }
    status = AlgPid_Reset(&self->velocity_controller,
                          velocity_measurement,
                          initial_output);
    if (status != ALG_PID_STATUS_OK)
    {
        return status;
    }

    self->position_loop_counter = self->position_loop_divider - 1U;
    self->position_elapsed_time_s = 0.0F;
    self->velocity_setpoint = 0.0F;
    return ALG_PID_STATUS_OK;
}

AlgPidStatus_t AlgPidCascade_Update(AlgPidCascade_t *self,
                                    const AlgPidCascadeInput_t *input,
                                    float *output)
{
    AlgPidInput_t position_input;
    AlgPidInput_t velocity_input;
    AlgPidStatus_t status;
    float position_output;

    if ((self == NULL) || (input == NULL) || (output == NULL))
    {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_PID_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(input->position_setpoint) ||
        !isfinite(input->position_measurement) ||
        !isfinite(input->velocity_measurement) ||
        !isfinite(input->velocity_feedforward) ||
        !isfinite(input->actuator_feedforward) ||
        !isfinite(input->delta_time_s) || (input->delta_time_s <= 0.0F))
    {
        return ALG_PID_STATUS_OUT_OF_RANGE;
    }

    self->position_elapsed_time_s += input->delta_time_s;
    ++self->position_loop_counter;
    if (self->position_loop_counter >= self->position_loop_divider)
    {
        position_input = (AlgPidInput_t){
            .setpoint = input->position_setpoint,
            .measurement = input->position_measurement,
            .setpoint_rate_per_s = 0.0F,
            .setpoint_acceleration_per_s2 = 0.0F,
            .additional_feedforward = input->velocity_feedforward,
            .delta_time_s = self->position_elapsed_time_s};
        status = AlgPid_UpdateAdvanced(&self->position_controller,
                                       &position_input,
                                       &position_output);
        if (status != ALG_PID_STATUS_OK)
        {
            return status;
        }
        self->velocity_setpoint = AlgPidInternal_Clamp(
            position_output,
            self->velocity_setpoint_min,
            self->velocity_setpoint_max);
        self->position_loop_counter = 0U;
        self->position_elapsed_time_s = 0.0F;
    }

    velocity_input = (AlgPidInput_t){
        .setpoint = self->velocity_setpoint,
        .measurement = input->velocity_measurement,
        .setpoint_rate_per_s = 0.0F,
        .setpoint_acceleration_per_s2 = 0.0F,
        .additional_feedforward = input->actuator_feedforward,
        .delta_time_s = input->delta_time_s};
    return AlgPid_UpdateAdvanced(&self->velocity_controller,
                                 &velocity_input,
                                 output);
}

float AlgPidCascade_GetVelocitySetpoint(const AlgPidCascade_t *self)
{
    return ((self != NULL) && self->is_initialized)
               ? self->velocity_setpoint
               : 0.0F;
}
