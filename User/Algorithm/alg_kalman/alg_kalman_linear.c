#include "alg_kalman_internal.h"

#include <stddef.h>

static AlgKalmanStatus_t AlgKalmanLinear_ValidateConfig(
    const AlgKalmanLinearConfig_t *config)
{
    size_t required_workspace;
    size_t state_square;
    size_t measurement_square;

    if ((config == NULL) || (config->state == NULL) ||
        (config->covariance == NULL) || (config->transition_matrix == NULL) ||
        (config->process_noise == NULL) || (config->measurement_matrix == NULL) ||
        (config->measurement_noise == NULL) || (config->workspace == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if ((config->state_dimension == 0U) || (config->measurement_dimension == 0U))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }
    if ((config->control_dimension > 0U) && (config->control_matrix == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
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
        !AlgKalmanInternal_IsFiniteArray(config->transition_matrix, state_square) ||
        !AlgKalmanInternal_IsFiniteArray(config->process_noise, state_square) ||
        !AlgKalmanInternal_IsFiniteArray(config->measurement_matrix,
                                         config->measurement_dimension *
                                             config->state_dimension) ||
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
    if ((config->control_dimension > 0U) &&
        !AlgKalmanInternal_IsFiniteArray(config->control_matrix,
                                         config->state_dimension *
                                             config->control_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    return ALG_KALMAN_STATUS_OK;
}

AlgKalmanStatus_t AlgKalmanLinear_Init(AlgKalmanLinear_t *self,
                                       const AlgKalmanLinearConfig_t *config)
{
    AlgKalmanStatus_t status;

    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }

    self->is_initialized = false;
    status = AlgKalmanLinear_ValidateConfig(config);
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

AlgKalmanStatus_t AlgKalmanLinear_Reset(AlgKalmanLinear_t *self,
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

AlgKalmanStatus_t AlgKalmanLinear_Predict(AlgKalmanLinear_t *self,
                                          const float *control_input)
{
    const AlgKalmanLinearConfig_t *config;
    size_t state_square;
    size_t state_index;
    size_t control_index;
    float *predicted_state;
    float *temporary_covariance;
    float *predicted_covariance;

    if (self == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!self->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
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
    temporary_covariance = predicted_state + config->state_dimension;
    predicted_covariance = temporary_covariance + state_square;

    AlgKalmanInternal_Multiply(config->transition_matrix,
                               config->state_dimension,
                               config->state_dimension,
                               config->state,
                               1U,
                               predicted_state);

    if (config->control_dimension > 0U)
    {
        for (state_index = 0U; state_index < config->state_dimension; ++state_index)
        {
            for (control_index = 0U; control_index < config->control_dimension;
                 ++control_index)
            {
                predicted_state[state_index] +=
                    config->control_matrix[
                        (state_index * config->control_dimension) + control_index] *
                    control_input[control_index];
            }
        }
    }

    AlgKalmanInternal_Multiply(config->transition_matrix,
                               config->state_dimension,
                               config->state_dimension,
                               config->covariance,
                               config->state_dimension,
                               temporary_covariance);
    AlgKalmanInternal_MultiplyRightTranspose(temporary_covariance,
                                             config->state_dimension,
                                             config->state_dimension,
                                             config->transition_matrix,
                                             config->state_dimension,
                                             predicted_covariance);
    for (state_index = 0U; state_index < state_square; ++state_index)
    {
        predicted_covariance[state_index] += config->process_noise[state_index];
    }
    AlgKalmanInternal_Symmetrize(predicted_covariance, config->state_dimension);

    if (!AlgKalmanInternal_IsFiniteArray(predicted_state, config->state_dimension) ||
        !AlgKalmanInternal_IsFiniteArray(predicted_covariance, state_square))
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

AlgKalmanStatus_t AlgKalmanLinear_Correct(AlgKalmanLinear_t *self,
                                          const float *measurement)
{
    const AlgKalmanLinearConfig_t *config;
    float *predicted_measurement;

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

    predicted_measurement = config->workspace;
    AlgKalmanInternal_Multiply(config->measurement_matrix,
                               config->measurement_dimension,
                               config->state_dimension,
                               config->state,
                               1U,
                               predicted_measurement);

    return AlgKalmanInternal_Correct(
        config->state,
        config->covariance,
        config->state_dimension,
        config->measurement_matrix,
        config->measurement_noise,
        measurement,
        predicted_measurement,
        config->measurement_dimension,
        config->workspace + config->measurement_dimension,
        config->workspace_size - config->measurement_dimension);
}

const float *AlgKalmanLinear_GetState(const AlgKalmanLinear_t *self)
{
    return ((self != NULL) && self->is_initialized) ? self->config.state : NULL;
}

const float *AlgKalmanLinear_GetCovariance(const AlgKalmanLinear_t *self)
{
    return ((self != NULL) && self->is_initialized) ? self->config.covariance : NULL;
}
