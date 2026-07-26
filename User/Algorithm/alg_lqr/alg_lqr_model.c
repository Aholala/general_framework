#include "alg_lqr_internal.h"

#include <math.h>
#include <stddef.h>

alg_lqr_status_t alg_lqr_discretize_tustin(const float *continuous_state_matrix,
                                           const float *continuous_control_matrix,
                                           size_t state_dimension, size_t control_dimension,
                                           float delta_time_s, float *discrete_state_matrix,
                                           float *discrete_control_matrix, float *workspace,
                                           size_t workspace_size)
{
    size_t state_square;
    size_t cross_size;
    size_t row;
    size_t column;
    float *left_matrix;
    float *left_inverse;
    float *right_matrix;
    float *scaled_control;
    alg_lqr_status_t status;

    if ((continuous_state_matrix == NULL) || (continuous_control_matrix == NULL) ||
        (discrete_state_matrix == NULL) || (discrete_control_matrix == NULL) || (workspace == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if ((state_dimension == 0U) || (control_dimension == 0U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (workspace_size < ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(state_dimension, control_dimension))
    {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
    }

    state_square = state_dimension * state_dimension;
    cross_size = state_dimension * control_dimension;
    if (!alg_lqr_internal_is_finite_array(continuous_state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(continuous_control_matrix, cross_size))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    left_matrix = workspace;
    left_inverse = left_matrix + state_square;
    right_matrix = left_inverse + state_square;
    scaled_control = right_matrix + state_square;

    for (row = 0U; row < state_dimension; ++row)
    {
        for (column = 0U; column < state_dimension; ++column)
        {
            const size_t index = (row * state_dimension) + column;
            const float identity = (row == column) ? 1.0F : 0.0F;
            const float scaled_state = 0.5F * delta_time_s * continuous_state_matrix[index];
            left_matrix[index] = identity - scaled_state;
            right_matrix[index] = identity + scaled_state;
        }
    }
    for (row = 0U; row < cross_size; ++row)
    {
        scaled_control[row] = delta_time_s * continuous_control_matrix[row];
    }

    status = alg_lqr_internal_invert(left_matrix, left_inverse, state_dimension);
    if (status != ALG_LQR_STATUS_OK)
    {
        return status;
    }
    alg_lqr_internal_multiply(left_inverse, state_dimension, state_dimension, right_matrix,
                              state_dimension, discrete_state_matrix);
    alg_lqr_internal_multiply(left_inverse, state_dimension, state_dimension, scaled_control,
                              control_dimension, discrete_control_matrix);

    return (alg_lqr_internal_is_finite_array(discrete_state_matrix, state_square) &&
            alg_lqr_internal_is_finite_array(discrete_control_matrix, cross_size))
               ? ALG_LQR_STATUS_OK
               : ALG_LQR_STATUS_NUMERICAL_ERROR;
}

alg_lqr_status_t alg_lqr_lqi_build_augmented_model(
    const float *state_matrix, const float *control_matrix, const float *output_matrix,
    size_t state_dimension, size_t control_dimension, size_t integral_dimension, float delta_time_s,
    float *augmented_state_matrix, float *augmented_control_matrix)
{
    size_t augmented_dimension;
    size_t row;
    size_t column;

    if ((state_matrix == NULL) || (control_matrix == NULL) || (output_matrix == NULL) ||
        (augmented_state_matrix == NULL) || (augmented_control_matrix == NULL))
    {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
    }
    if ((state_dimension == 0U) || (control_dimension == 0U) || (integral_dimension == 0U) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }
    if (!alg_lqr_internal_is_finite_array(state_matrix, state_dimension * state_dimension) ||
        !alg_lqr_internal_is_finite_array(control_matrix, state_dimension * control_dimension) ||
        !alg_lqr_internal_is_finite_array(output_matrix, integral_dimension * state_dimension))
    {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
    }

    augmented_dimension = state_dimension + integral_dimension;
    for (row = 0U; row < augmented_dimension; ++row)
    {
        for (column = 0U; column < augmented_dimension; ++column)
        {
            augmented_state_matrix[(row * augmented_dimension) + column] = 0.0F;
        }
        for (column = 0U; column < control_dimension; ++column)
        {
            augmented_control_matrix[(row * control_dimension) + column] = 0.0F;
        }
    }

    for (row = 0U; row < state_dimension; ++row)
    {
        for (column = 0U; column < state_dimension; ++column)
        {
            augmented_state_matrix[(row * augmented_dimension) + column] =
                state_matrix[(row * state_dimension) + column];
        }
        for (column = 0U; column < control_dimension; ++column)
        {
            augmented_control_matrix[(row * control_dimension) + column] =
                control_matrix[(row * control_dimension) + column];
        }
    }

    for (row = 0U; row < integral_dimension; ++row)
    {
        for (column = 0U; column < state_dimension; ++column)
        {
            augmented_state_matrix[((state_dimension + row) * augmented_dimension) + column] =
                -delta_time_s * output_matrix[(row * state_dimension) + column];
        }
        augmented_state_matrix[((state_dimension + row) * augmented_dimension) + state_dimension +
                               row] = 1.0F;
    }
    return ALG_LQR_STATUS_OK;
}
