#include "alg_kalman_internal.h"

#include <math.h>
#include <stddef.h>

static AlgKalmanStatus_t AlgKalmanExtended_ValidateConfig(
    const AlgKalmanExtendedConfig_t *config)
{
    size_t required_workspace;
    size_t state_square;
    size_t measurement_square;

    if ((config == NULL) || (config->state == NULL) ||
        (config->covariance == NULL) || (config->process_noise == NULL) ||
        (config->measurement_noise == NULL) || (config->workspace == NULL) ||
        (config->state_function == NULL) ||
        (config->state_jacobian_function == NULL) ||
        (config->measurement_function == NULL) ||
        (config->measurement_jacobian_function == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if ((config->state_dimension == 0U) || (config->measurement_dimension == 0U))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    required_workspace = ALG_KALMAN_WORKSPACE_SIZE(config->state_dimension,
                                                    config->measurement_dimension);
    if (config->workspace_size < required_workspace)
    {
        return ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE;
    }

    state_square = config->state_dimension * config->state_dimension;
    measurement_square = config->measurement_dimension * config->measurement_dimension;
    if (!AlgKalmanInternal_IsFiniteArray(config->state, config->state_dimension) ||
        !AlgKalmanInternal_IsFiniteArray(config->covariance, state_square) ||
        !AlgKalmanInternal_IsFiniteArray(config->process_noise, state_square) ||
        !AlgKalmanInternal_IsFiniteArray(config->measurement_noise,
                                         measurement_square) ||
        !AlgKalmanInternal_HasNonnegativeDiagonal(config->covariance,
                                                  config->state_dimension) ||
        !AlgKalmanInternal_HasNonnegativeDiagonal(config->process_noise,
                                                  config->state_dimension) ||
        !AlgKalmanInternal_HasNonnegativeDiagonal(config->measurement_noise,
                                                  config->measurement_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanExtended_Init(AlgKalmanExtended_t *self,
                                         const AlgKalmanExtendedConfig_t *config)
{
    AlgKalmanStatus_t status;

    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgKalmanExtended_ValidateConfig(config);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }

    self->config = *config;
    AlgKalmanInternal_Symmetrize(self->config.covariance,
                                 self->config.state_dimension);
    self->is_initialized = true;
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanExtended_Reset(AlgKalmanExtended_t *self,
                                          const float *initial_state,
                                          const float *initial_covariance)
{
    size_t state_square;

    if ((self == NULL) || (initial_state == NULL) || (initial_covariance == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }

    state_square = self->config.state_dimension * self->config.state_dimension;
    if (!AlgKalmanInternal_IsFiniteArray(initial_state,
                                         self->config.state_dimension) ||
        !AlgKalmanInternal_IsFiniteArray(initial_covariance, state_square) ||
        !AlgKalmanInternal_HasNonnegativeDiagonal(initial_covariance,
                                                  self->config.state_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    AlgKalmanInternal_Copy(self->config.state,
                           initial_state,
                           self->config.state_dimension);
    AlgKalmanInternal_Copy(self->config.covariance,
                           initial_covariance,
                           state_square);
    AlgKalmanInternal_Symmetrize(self->config.covariance,
                                 self->config.state_dimension);
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanExtended_Predict(AlgKalmanExtended_t *self,
                                            const float *control_input,
                                            float delta_time_s)
{
    const AlgKalmanExtendedConfig_t *config;
    size_t state_square;
    size_t index;
    float *predicted_state;
    float *state_jacobian;
    float *temporary_covariance;
    float *predicted_covariance;
    AlgKalmanStatus_t status;

    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    config = &self->config;
    if ((config->control_dimension > 0U) && (control_input == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if ((config->control_dimension > 0U) &&
        !AlgKalmanInternal_IsFiniteArray(control_input, config->control_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    state_square = config->state_dimension * config->state_dimension;
    predicted_state = config->workspace;
    state_jacobian = predicted_state + config->state_dimension;
    temporary_covariance = state_jacobian + state_square;
    predicted_covariance = temporary_covariance + state_square;

    status = config->state_function(config->state,
                                    config->state_dimension,
                                    control_input,
                                    config->control_dimension,
                                    delta_time_s,
                                    predicted_state,
                                    config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    status = config->state_jacobian_function(config->state,
                                             config->state_dimension,
                                             control_input,
                                             config->control_dimension,
                                             delta_time_s,
                                             state_jacobian,
                                             config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    if (!AlgKalmanInternal_IsFiniteArray(predicted_state,
                                         config->state_dimension) ||
        !AlgKalmanInternal_IsFiniteArray(state_jacobian, state_square))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    AlgKalmanInternal_Multiply(state_jacobian,
                               config->state_dimension,
                               config->state_dimension,
                               config->covariance,
                               config->state_dimension,
                               temporary_covariance);
    AlgKalmanInternal_MultiplyRightTranspose(temporary_covariance,
                                             config->state_dimension,
                                             config->state_dimension,
                                             state_jacobian,
                                             config->state_dimension,
                                             predicted_covariance);
    for (index = 0U; index < state_square; ++index)
    {
        predicted_covariance[index] += config->process_noise[index];
    }
    AlgKalmanInternal_Symmetrize(predicted_covariance, config->state_dimension);

    if (!AlgKalmanInternal_IsFiniteArray(predicted_covariance, state_square))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    AlgKalmanInternal_Copy(config->state,
                           predicted_state,
                           config->state_dimension);
    AlgKalmanInternal_Copy(config->covariance,
                           predicted_covariance,
                           state_square);
    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanExtended_Correct(AlgKalmanExtended_t *self,
                                            const float *measurement)
{
    const AlgKalmanExtendedConfig_t *config;
    size_t cross_size;
    float *predicted_measurement;
    float *measurement_jacobian;
    float *correction_workspace;
    size_t correction_workspace_size;
    AlgKalmanStatus_t status;

    if ((self == NULL) || (measurement == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }

    config = &self->config;
    if (!AlgKalmanInternal_IsFiniteArray(measurement,
                                         config->measurement_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    cross_size = config->state_dimension * config->measurement_dimension;
    predicted_measurement = config->workspace;
    measurement_jacobian = predicted_measurement + config->measurement_dimension;
    correction_workspace = measurement_jacobian + cross_size;
    correction_workspace_size = config->workspace_size -
                                config->measurement_dimension - cross_size;

    status = config->measurement_function(config->state,
                                          config->state_dimension,
                                          config->measurement_dimension,
                                          predicted_measurement,
                                          config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    status = config->measurement_jacobian_function(config->state,
                                                   config->state_dimension,
                                                   config->measurement_dimension,
                                                   measurement_jacobian,
                                                   config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    if (!AlgKalmanInternal_IsFiniteArray(predicted_measurement,
                                         config->measurement_dimension) ||
        !AlgKalmanInternal_IsFiniteArray(measurement_jacobian, cross_size))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    return AlgKalmanInternal_Correct(config->state,
                                     config->covariance,
                                     config->state_dimension,
                                     measurement_jacobian,
                                     config->measurement_noise,
                                     measurement,
                                     predicted_measurement,
                                     config->measurement_dimension,
                                     correction_workspace,
                                     correction_workspace_size);
}

const float *AlgKalmanExtended_GetState(const AlgKalmanExtended_t *self)
{
    return ((self != NULL) && self->is_initialized) ? self->config.state : NULL;
}

const float *AlgKalmanExtended_GetCovariance(const AlgKalmanExtended_t *self)
{
    return ((self != NULL) && self->is_initialized) ? self->config.covariance : NULL;
}
