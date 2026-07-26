#include "alg_lqr_internal.h"

#include <math.h>
#include <stddef.h>

alg_lqr_status_t alg_lqr_internal_riccati_step(
    size_t state_dimension, size_t control_dimension, const float *state_matrix,
    const float *control_matrix, const float *state_weight, const float *control_weight,
    const float *cross_weight, const float *next_riccati, float *current_riccati,
    float *gain_matrix, float *workspace, size_t workspace_size)
{
    const size_t state_square = state_dimension * state_dimension;
    const size_t control_square = control_dimension * control_dimension;
    const size_t cross_size = state_dimension * control_dimension;
    const size_t required_size = (3U * state_square) + (3U * cross_size) + (2U * control_square);
    float *riccati_control;
    float *gain_denominator;
    float *gain_denominator_inverse;
    float *riccati_state;
    float *gain_numerator;
    float *state_transpose_riccati_state;
    float *state_transpose_riccati_control;
    float *feedback_cost;
    size_t row;
    size_t column;
    size_t index;
    alg_lqr_status_t status;

    if (workspace_size < required_size)
    {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
    }

    riccati_control = workspace;
    gain_denominator = riccati_control + cross_size;
    gain_denominator_inverse = gain_denominator + control_square;
    riccati_state = gain_denominator_inverse + control_square;
    gain_numerator = riccati_state + state_square;
    state_transpose_riccati_state = gain_numerator + cross_size;
    state_transpose_riccati_control = state_transpose_riccati_state + state_square;
    feedback_cost = state_transpose_riccati_control + cross_size;

    alg_lqr_internal_multiply(next_riccati, state_dimension, state_dimension, control_matrix,
                              control_dimension, riccati_control);
    alg_lqr_internal_multiply_left_transpose(control_matrix, state_dimension, control_dimension,
                                             riccati_control, control_dimension, gain_denominator);
    for (index = 0U; index < control_square; ++index)
    {
        gain_denominator[index] += control_weight[index];
    }

    alg_lqr_internal_multiply(next_riccati, state_dimension, state_dimension, state_matrix,
                              state_dimension, riccati_state);
    alg_lqr_internal_multiply_left_transpose(control_matrix, state_dimension, control_dimension,
                                             riccati_state, state_dimension, gain_numerator);
    if (cross_weight != NULL)
    {
        for (row = 0U; row < control_dimension; ++row)
        {
            for (column = 0U; column < state_dimension; ++column)
            {
                gain_numerator[(row * state_dimension) + column] +=
                    cross_weight[(column * control_dimension) + row];
            }
        }
    }

    status = alg_lqr_internal_invert(gain_denominator, gain_denominator_inverse, control_dimension);
    if (status != ALG_LQR_STATUS_OK)
    {
        return status;
    }
    alg_lqr_internal_multiply(gain_denominator_inverse, control_dimension, control_dimension,
                              gain_numerator, state_dimension, gain_matrix);

    alg_lqr_internal_multiply_left_transpose(state_matrix, state_dimension, state_dimension,
                                             riccati_state, state_dimension,
                                             state_transpose_riccati_state);
    alg_lqr_internal_multiply_left_transpose(state_matrix, state_dimension, state_dimension,
                                             riccati_control, control_dimension,
                                             state_transpose_riccati_control);
    if (cross_weight != NULL)
    {
        for (index = 0U; index < cross_size; ++index)
        {
            state_transpose_riccati_control[index] += cross_weight[index];
        }
    }
    alg_lqr_internal_multiply(state_transpose_riccati_control, state_dimension, control_dimension,
                              gain_matrix, state_dimension, feedback_cost);

    for (index = 0U; index < state_square; ++index)
    {
        current_riccati[index] =
            state_weight[index] + state_transpose_riccati_state[index] - feedback_cost[index];
    }
    alg_lqr_internal_symmetrize(current_riccati, state_dimension);

    return (alg_lqr_internal_is_finite_array(current_riccati, state_square) &&
            alg_lqr_internal_is_finite_array(gain_matrix, cross_size))
               ? ALG_LQR_STATUS_OK
               : ALG_LQR_STATUS_NUMERICAL_ERROR;
}

static alg_lqr_status_t alg_lqr_dare_validate(const alg_lqr_dare_config_t *config)
{
    size_t state_square;
    size_t control_square;
    size_t cross_size;

    if ((config == NULL) || (config->state_matrix == NULL) || (config->control_matrix == NULL) ||
        (config->state_weight == NULL) || (config->control_weight == NULL) ||
        (config->workspace == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        !isfinite(config->tolerance) || (config->tolerance <= 0.0F) ||
        (config->maximum_iterations == 0U))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (config->workspace_size <
        ALG_LQR_RICCATI_WORKSPACE_SIZE(config->state_dimension, config->control_dimension))
    {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
    }

    state_square = config->state_dimension * config->state_dimension;
    control_square = config->control_dimension * config->control_dimension;
    cross_size = config->state_dimension * config->control_dimension;
    if (!alg_lqr_internal_is_finite_array(config->state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_matrix, cross_size) ||
        !alg_lqr_internal_is_finite_array(config->state_weight, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_weight, control_square) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->state_weight, config->state_dimension) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->control_weight,
                                                   config->control_dimension) ||
        ((config->cross_weight != NULL) &&
         !alg_lqr_internal_is_finite_array(config->cross_weight, cross_size)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    return ALG_LQR_STATUS_OK;
}

alg_lqr_status_t alg_lqr_dare_solve(const alg_lqr_dare_config_t *config, float *riccati_solution,
                                    float *gain_matrix, size_t *completed_iterations)
{
    size_t state_square;
    size_t iteration;
    size_t index;
    float maximum_difference;
    float *candidate_riccati;
    float *step_workspace;
    size_t step_workspace_size;
    alg_lqr_status_t status;

    if ((riccati_solution == NULL) || (gain_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    status = alg_lqr_dare_validate(config);
    if (status != ALG_LQR_STATUS_OK)
    {
        return status;
    }

    state_square = config->state_dimension * config->state_dimension;
    candidate_riccati = config->workspace;
    step_workspace = candidate_riccati + state_square;
    step_workspace_size = config->workspace_size - state_square;
    alg_lqr_internal_copy(riccati_solution, config->state_weight, state_square);

    for (iteration = 1U; iteration <= config->maximum_iterations; ++iteration)
    {
        status = alg_lqr_internal_riccati_step(
            config->state_dimension, config->control_dimension, config->state_matrix,
            config->control_matrix, config->state_weight, config->control_weight,
            config->cross_weight, riccati_solution, candidate_riccati, gain_matrix, step_workspace,
            step_workspace_size);
        if (status != ALG_LQR_STATUS_OK)
        {
            return status;
        }

        maximum_difference = 0.0F;
        for (index = 0U; index < state_square; ++index)
        {
            const float difference = fabsf(candidate_riccati[index] - riccati_solution[index]);
            if (difference > maximum_difference)
            {
                maximum_difference = difference;
            }
        }
        alg_lqr_internal_copy(riccati_solution, candidate_riccati, state_square);
        if (maximum_difference <= config->tolerance)
        {
            status = alg_lqr_internal_riccati_step(
                config->state_dimension, config->control_dimension, config->state_matrix,
                config->control_matrix, config->state_weight, config->control_weight,
                config->cross_weight, riccati_solution, candidate_riccati, gain_matrix,
                step_workspace, step_workspace_size);
            if (status != ALG_LQR_STATUS_OK)
            {
                return status;
            }
            if (completed_iterations != NULL)
            {
                *completed_iterations = iteration;
            }
            return ALG_LQR_STATUS_OK;
        }
    }

    if (completed_iterations != NULL)
    {
        *completed_iterations = config->maximum_iterations;
    }
    return ALG_LQR_STATUS_NOT_CONVERGED;
}

alg_lqr_status_t alg_lqr_finite_solve(const alg_lqr_finite_config_t *config, float *gain_sequence,
                                      float *initial_riccati_solution)
{
    size_t state_square;
    size_t control_square;
    size_t cross_size;
    size_t step;
    float *next_riccati;
    float *step_workspace;
    size_t step_workspace_size;
    alg_lqr_status_t status;

    if ((config == NULL) || (gain_sequence == NULL) || (initial_riccati_solution == NULL) ||
        (config->state_matrix == NULL) || (config->control_matrix == NULL) ||
        (config->state_weight == NULL) || (config->control_weight == NULL) ||
        (config->terminal_state_weight == NULL) || (config->workspace == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        (config->horizon_length == 0U))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (config->workspace_size <
        ALG_LQR_FINITE_WORKSPACE_SIZE(config->state_dimension, config->control_dimension))
    {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
    }

    state_square = config->state_dimension * config->state_dimension;
    control_square = config->control_dimension * config->control_dimension;
    cross_size = config->state_dimension * config->control_dimension;
    if (!alg_lqr_internal_is_finite_array(config->state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_matrix, cross_size) ||
        !alg_lqr_internal_is_finite_array(config->state_weight, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_weight, control_square) ||
        !alg_lqr_internal_is_finite_array(config->terminal_state_weight, state_square) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->state_weight, config->state_dimension) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->control_weight,
                                                   config->control_dimension) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->terminal_state_weight,
                                                   config->state_dimension) ||
        ((config->cross_weight != NULL) &&
         !alg_lqr_internal_is_finite_array(config->cross_weight, cross_size)))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    next_riccati = config->workspace;
    step_workspace = next_riccati + state_square;
    step_workspace_size = config->workspace_size - state_square;
    alg_lqr_internal_copy(next_riccati, config->terminal_state_weight, state_square);
    alg_lqr_internal_symmetrize(next_riccati, config->state_dimension);

    for (step = config->horizon_length; step > 0U; --step)
    {
        status = alg_lqr_internal_riccati_step(
            config->state_dimension, config->control_dimension, config->state_matrix,
            config->control_matrix, config->state_weight, config->control_weight,
            config->cross_weight, next_riccati, initial_riccati_solution,
            &gain_sequence[(step - 1U) * config->control_dimension * config->state_dimension],
            step_workspace, step_workspace_size);
        if (status != ALG_LQR_STATUS_OK)
        {
            return status;
        }
        alg_lqr_internal_copy(next_riccati, initial_riccati_solution, state_square);
    }
    return ALG_LQR_STATUS_OK;
}
