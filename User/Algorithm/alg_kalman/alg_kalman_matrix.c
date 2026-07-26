#include "alg_kalman_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_KALMAN_SINGULAR_THRESHOLD (1.0e-12F)

static alg_kalman_status_t alg_kalman_internal_invert(float *matrix, float *inverse,
                                                      size_t dimension)
{
    size_t row;
    size_t column;
    size_t pivot_row;
    size_t element;
    float pivot_magnitude;
    float candidate_magnitude;
    float pivot_value;
    float scale;
    float temporary;

    for (row = 0U; row < dimension; ++row)
    {
        for (column = 0U; column < dimension; ++column)
        {
            inverse[(row * dimension) + column] = (row == column) ? 1.0F : 0.0F;
        }
    }

    for (column = 0U; column < dimension; ++column)
    {
        pivot_row = column;
        pivot_magnitude = fabsf(matrix[(column * dimension) + column]);
        for (row = column + 1U; row < dimension; ++row)
        {
            candidate_magnitude = fabsf(matrix[(row * dimension) + column]);
            if (candidate_magnitude > pivot_magnitude)
            {
                pivot_magnitude = candidate_magnitude;
                pivot_row = row;
            }
        }

        if (!isfinite(pivot_magnitude) || (pivot_magnitude <= ALG_KALMAN_SINGULAR_THRESHOLD))
        {
            return ALG_KALMAN_STATUS_SINGULAR_MATRIX;
        }

        if (pivot_row != column)
        {
            for (element = 0U; element < dimension; ++element)
            {
                temporary = matrix[(column * dimension) + element];
                matrix[(column * dimension) + element] = matrix[(pivot_row * dimension) + element];
                matrix[(pivot_row * dimension) + element] = temporary;

                temporary = inverse[(column * dimension) + element];
                inverse[(column * dimension) + element] =
                    inverse[(pivot_row * dimension) + element];
                inverse[(pivot_row * dimension) + element] = temporary;
            }
        }

        pivot_value = matrix[(column * dimension) + column];
        for (element = 0U; element < dimension; ++element)
        {
            matrix[(column * dimension) + element] /= pivot_value;
            inverse[(column * dimension) + element] /= pivot_value;
        }

        for (row = 0U; row < dimension; ++row)
        {
            if (row == column)
            {
                continue;
            }

            scale = matrix[(row * dimension) + column];
            for (element = 0U; element < dimension; ++element)
            {
                matrix[(row * dimension) + element] -=
                    scale * matrix[(column * dimension) + element];
                inverse[(row * dimension) + element] -=
                    scale * inverse[(column * dimension) + element];
            }
        }
    }

    return alg_kalman_internal_is_finite_array(inverse, dimension * dimension)
               ? ALG_KALMAN_STATUS_OK
               : ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

bool alg_kalman_internal_is_finite_array(const float *values, size_t value_count)
{
    size_t index;

    if (values == NULL)
    {
        return false;
    }

    for (index = 0U; index < value_count; ++index)
    {
        if (!isfinite(values[index]))
        {
            return false;
        }
    }
    return true;
}

bool alg_kalman_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension)
{
    size_t index;

    if (matrix == NULL)
    {
        return false;
    }
    for (index = 0U; index < dimension; ++index)
    {
        if (!isfinite(matrix[(index * dimension) + index]) ||
            (matrix[(index * dimension) + index] < 0.0F))
        {
            return false;
        }
    }
    return true;
}

void alg_kalman_internal_copy(float *destination, const float *source, size_t value_count)
{
    size_t index;

    for (index = 0U; index < value_count; ++index)
    {
        destination[index] = source[index];
    }
}

void alg_kalman_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                                  const float *right, size_t right_columns, float *output)
{
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    for (row = 0U; row < left_rows; ++row)
    {
        for (column = 0U; column < right_columns; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < shared_dimension; ++shared_index)
            {
                accumulator += left[(row * shared_dimension) + shared_index] *
                               right[(shared_index * right_columns) + column];
            }
            output[(row * right_columns) + column] = accumulator;
        }
    }
}

void alg_kalman_internal_multiply_right_transpose(const float *left, size_t left_rows,
                                                  size_t shared_dimension, const float *right,
                                                  size_t right_rows, float *output)
{
    size_t left_row;
    size_t right_row;
    size_t shared_index;
    float accumulator;

    for (left_row = 0U; left_row < left_rows; ++left_row)
    {
        for (right_row = 0U; right_row < right_rows; ++right_row)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < shared_dimension; ++shared_index)
            {
                accumulator += left[(left_row * shared_dimension) + shared_index] *
                               right[(right_row * shared_dimension) + shared_index];
            }
            output[(left_row * right_rows) + right_row] = accumulator;
        }
    }
}

void alg_kalman_internal_symmetrize(float *matrix, size_t dimension)
{
    size_t row;
    size_t column;
    float average;

    for (row = 0U; row < dimension; ++row)
    {
        for (column = row + 1U; column < dimension; ++column)
        {
            average =
                0.5F * (matrix[(row * dimension) + column] + matrix[(column * dimension) + row]);
            matrix[(row * dimension) + column] = average;
            matrix[(column * dimension) + row] = average;
        }
    }
}

alg_kalman_status_t
alg_kalman_internal_correct(float *state, float *covariance, size_t state_dimension,
                            const float *measurement_matrix, const float *measurement_noise,
                            const float *measurement, const float *predicted_measurement,
                            size_t measurement_dimension, float *workspace, size_t workspace_size)
{
    const size_t state_square = state_dimension * state_dimension;
    const size_t measurement_square = measurement_dimension * measurement_dimension;
    const size_t cross_size = state_dimension * measurement_dimension;
    const size_t required_size = state_dimension + (3U * state_square) + (3U * cross_size) +
                                 measurement_dimension + (2U * measurement_square);
    float *new_state;
    float *innovation;
    float *measurement_covariance_product;
    float *innovation_covariance;
    float *innovation_covariance_inverse;
    float *covariance_measurement_transpose;
    float *gain;
    float *identity_minus_gain_measurement;
    float *temporary_state_square;
    float *new_covariance;
    size_t state_index;
    size_t measurement_index;
    size_t index;
    alg_kalman_status_t status;

    if (workspace_size < required_size)
    {
        return ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE;
    }

    new_state = workspace;
    innovation = new_state + state_dimension;
    measurement_covariance_product = innovation + measurement_dimension;
    innovation_covariance = measurement_covariance_product + cross_size;
    innovation_covariance_inverse = innovation_covariance + measurement_square;
    covariance_measurement_transpose = innovation_covariance_inverse + measurement_square;
    gain = covariance_measurement_transpose + cross_size;
    identity_minus_gain_measurement = gain + cross_size;
    temporary_state_square = identity_minus_gain_measurement + state_square;
    new_covariance = temporary_state_square + state_square;

    for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index)
    {
        innovation[measurement_index] =
            measurement[measurement_index] - predicted_measurement[measurement_index];
    }

    alg_kalman_internal_multiply(measurement_matrix, measurement_dimension, state_dimension,
                                 covariance, state_dimension, measurement_covariance_product);
    alg_kalman_internal_multiply_right_transpose(
        measurement_covariance_product, measurement_dimension, state_dimension, measurement_matrix,
        measurement_dimension, innovation_covariance);
    for (index = 0U; index < measurement_square; ++index)
    {
        innovation_covariance[index] += measurement_noise[index];
    }

    alg_kalman_internal_copy(new_state, state, state_dimension);
    for (state_index = 0U; state_index < state_dimension; ++state_index)
    {
        for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index)
        {
            covariance_measurement_transpose[(state_index * measurement_dimension) +
                                             measurement_index] =
                measurement_covariance_product[(measurement_index * state_dimension) + state_index];
        }
    }

    status = alg_kalman_internal_invert(innovation_covariance, innovation_covariance_inverse,
                                        measurement_dimension);
    if (status != ALG_KALMAN_STATUS_OK)
    {
        return status;
    }

    alg_kalman_internal_multiply(covariance_measurement_transpose, state_dimension,
                                 measurement_dimension, innovation_covariance_inverse,
                                 measurement_dimension, gain);

    for (state_index = 0U; state_index < state_dimension; ++state_index)
    {
        for (measurement_index = 0U; measurement_index < measurement_dimension; ++measurement_index)
        {
            new_state[state_index] +=
                gain[(state_index * measurement_dimension) + measurement_index] *
                innovation[measurement_index];
        }
    }

    alg_kalman_internal_multiply(gain, state_dimension, measurement_dimension, measurement_matrix,
                                 state_dimension, identity_minus_gain_measurement);
    for (index = 0U; index < state_square; ++index)
    {
        identity_minus_gain_measurement[index] = -identity_minus_gain_measurement[index];
    }
    for (state_index = 0U; state_index < state_dimension; ++state_index)
    {
        identity_minus_gain_measurement[(state_index * state_dimension) + state_index] += 1.0F;
    }

    alg_kalman_internal_multiply(identity_minus_gain_measurement, state_dimension, state_dimension,
                                 covariance, state_dimension, temporary_state_square);
    alg_kalman_internal_multiply_right_transpose(temporary_state_square, state_dimension,
                                                 state_dimension, identity_minus_gain_measurement,
                                                 state_dimension, new_covariance);

    alg_kalman_internal_multiply(gain, state_dimension, measurement_dimension, measurement_noise,
                                 measurement_dimension, covariance_measurement_transpose);
    alg_kalman_internal_multiply_right_transpose(covariance_measurement_transpose, state_dimension,
                                                 measurement_dimension, gain, state_dimension,
                                                 temporary_state_square);
    for (index = 0U; index < state_square; ++index)
    {
        new_covariance[index] += temporary_state_square[index];
    }

    alg_kalman_internal_symmetrize(new_covariance, state_dimension);
    if (!alg_kalman_internal_is_finite_array(new_state, state_dimension) ||
        !alg_kalman_internal_is_finite_array(new_covariance, state_square))
    {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
    }

    alg_kalman_internal_copy(state, new_state, state_dimension);
    alg_kalman_internal_copy(covariance, new_covariance, state_square);
    return ALG_KALMAN_STATUS_OK;
}
