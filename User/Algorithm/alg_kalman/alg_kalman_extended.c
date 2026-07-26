#include "alg_kalman_internal.h"

#include <math.h>
#include <stddef.h>

static alg_kalman_status_t
alg_kalman_extended_validate_config(const alg_kalman_extended_config_t *config)
{
    size_t required_workspace;
    size_t state_square;
    size_t measurement_square;

    if ((config == NULL) || (config->state == NULL) || (config->covariance == NULL) ||
        (config->process_noise == NULL) || (config->measurement_noise == NULL) ||
        (config->workspace == NULL) || (config->state_function == NULL) ||
        (config->state_jacobian_function == NULL) || (config->measurement_function == NULL) ||
        (config->measurement_jacobian_function == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if ((config->state_dimension == 0U) || (config->measurement_dimension == 0U))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    required_workspace =
        ALG_KALMAN_WORKSPACE_SIZE(config->state_dimension, config->measurement_dimension);
    if (config->workspace_size < required_workspace)
    {
        return ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE;
    }

    state_square = config->state_dimension * config->state_dimension;
    measurement_square = config->measurement_dimension * config->measurement_dimension;
    if (!alg_kalman_internal_is_finite_array(config->state, config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(config->covariance, state_square) ||
        !alg_kalman_internal_is_finite_array(config->process_noise, state_square) ||
        !alg_kalman_internal_is_finite_array(config->measurement_noise, measurement_square) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->covariance,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->process_noise,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->measurement_noise,
                                                      config->measurement_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_extended_init(alg_kalman_extended_t *me,
                                             const alg_kalman_extended_config_t *config)
{
    alg_kalman_status_t status;

    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    status = alg_kalman_extended_validate_config(config);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }

    me->config = *config;
    alg_kalman_internal_symmetrize(me->config.covariance, me->config.state_dimension);
    me->is_initialized = true;
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_extended_reset(alg_kalman_extended_t *me, const float *initial_state,
                                              const float *initial_covariance)
{
    size_t state_square;

    if ((me == NULL) || (initial_state == NULL) || (initial_covariance == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }

    state_square = me->config.state_dimension * me->config.state_dimension;
    if (!alg_kalman_internal_is_finite_array(initial_state, me->config.state_dimension) ||
        !alg_kalman_internal_is_finite_array(initial_covariance, state_square) ||
        !alg_kalman_internal_has_nonnegative_diagonal(initial_covariance,
                                                      me->config.state_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    alg_kalman_internal_copy(me->config.state, initial_state, me->config.state_dimension);
    alg_kalman_internal_copy(me->config.covariance, initial_covariance, state_square);
    alg_kalman_internal_symmetrize(me->config.covariance, me->config.state_dimension);
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_extended_predict(alg_kalman_extended_t *me,
                                                const float *control_input, float delta_time_s)
{
    const alg_kalman_extended_config_t *config;
    size_t state_square;
    size_t index;
    float *predicted_state;
    float *state_jacobian;
    float *temporary_covariance;
    float *predicted_covariance;
    alg_kalman_status_t status;

    if (me == NULL)
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }
    if (!isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    config = &me->config;
    if ((config->control_dimension > 0U) && (control_input == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if ((config->control_dimension > 0U) &&
        !alg_kalman_internal_is_finite_array(control_input, config->control_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    state_square = config->state_dimension * config->state_dimension;
    predicted_state = config->workspace;
    state_jacobian = predicted_state + config->state_dimension;
    temporary_covariance = state_jacobian + state_square;
    predicted_covariance = temporary_covariance + state_square;

    status = config->state_function(config->state, config->state_dimension, control_input,
                                    config->control_dimension, delta_time_s, predicted_state,
                                    config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    status = config->state_jacobian_function(config->state, config->state_dimension, control_input,
                                             config->control_dimension, delta_time_s,
                                             state_jacobian, config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    if (!alg_kalman_internal_is_finite_array(predicted_state, config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(state_jacobian, state_square))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    alg_kalman_internal_multiply(state_jacobian, config->state_dimension, config->state_dimension,
                                 config->covariance, config->state_dimension, temporary_covariance);
    alg_kalman_internal_multiply_right_transpose(temporary_covariance, config->state_dimension,
                                                 config->state_dimension, state_jacobian,
                                                 config->state_dimension, predicted_covariance);
    for (index = 0U; index < state_square; ++index)
    {
        predicted_covariance[index] += config->process_noise[index];
    }
    alg_kalman_internal_symmetrize(predicted_covariance, config->state_dimension);

    if (!alg_kalman_internal_is_finite_array(predicted_covariance, state_square))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    alg_kalman_internal_copy(config->state, predicted_state, config->state_dimension);
    alg_kalman_internal_copy(config->covariance, predicted_covariance, state_square);
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_status_t alg_kalman_extended_correct(alg_kalman_extended_t *me, const float *measurement)
{
    const alg_kalman_extended_config_t *config;
    size_t cross_size;
    float *predicted_measurement;
    float *measurement_jacobian;
    float *correction_workspace;
    size_t correction_workspace_size;
    alg_kalman_status_t status;

    if ((me == NULL) || (measurement == NULL))
    {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    }

    config = &me->config;
    if (!alg_kalman_internal_is_finite_array(measurement, config->measurement_dimension))
    {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
    }

    cross_size = config->state_dimension * config->measurement_dimension;
    predicted_measurement = config->workspace;
    measurement_jacobian = predicted_measurement + config->measurement_dimension;
    correction_workspace = measurement_jacobian + cross_size;
    correction_workspace_size = config->workspace_size - config->measurement_dimension - cross_size;

    status = config->measurement_function(config->state, config->state_dimension,
                                          config->measurement_dimension, predicted_measurement,
                                          config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    status = config->measurement_jacobian_function(config->state, config->state_dimension,
                                                   config->measurement_dimension,
                                                   measurement_jacobian, config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }
    if (!alg_kalman_internal_is_finite_array(predicted_measurement,
                                             config->measurement_dimension) ||
        !alg_kalman_internal_is_finite_array(measurement_jacobian, cross_size))
    {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
    }

    return alg_kalman_internal_correct(config->state, config->covariance, config->state_dimension,
                                       measurement_jacobian, config->measurement_noise, measurement,
                                       predicted_measurement, config->measurement_dimension,
                                       correction_workspace, correction_workspace_size);
}

const float *alg_kalman_extended_get_state(const alg_kalman_extended_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.state : NULL;
}

const float *alg_kalman_extended_get_covariance(const alg_kalman_extended_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.covariance : NULL;
}
