#include "alg_math.h"

#include <math.h>

#define ALG_MATH_MATRIX_PIVOT_THRESHOLD (1.0e-12F)

static bool alg_math_matrix_is_valid(const alg_math_matrix_t *matrix)
{
    return (matrix != NULL) && (matrix->data != NULL) && (matrix->rows > 0U) &&
           (matrix->columns > 0U) && (matrix->rows <= (SIZE_MAX / matrix->columns));
}

static bool alg_math_matrix_has_same_size(const alg_math_matrix_t *left,
                                          const alg_math_matrix_t *right)
{
    return (left->rows == right->rows) && (left->columns == right->columns);
}

alg_math_status_t alg_math_matrix_init(alg_math_matrix_t *matrix, float *data, size_t rows,
                                       size_t columns)
{
    if ((matrix == NULL) || (data == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((rows == 0U) || (columns == 0U) || (rows > (SIZE_MAX / columns)))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    matrix->rows = rows;
    matrix->columns = columns;
    matrix->data = data;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_zero(alg_math_matrix_t *matrix)
{
    size_t index;

    if (!alg_math_matrix_is_valid(matrix))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    for (index = 0U; index < (matrix->rows * matrix->columns); ++index)
    {
        matrix->data[index] = 0.0F;
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_identity(alg_math_matrix_t *matrix)
{
    size_t index;

    if (!alg_math_matrix_is_valid(matrix))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (matrix->rows != matrix->columns)
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    (void)alg_math_matrix_zero(matrix);
    for (index = 0U; index < matrix->rows; ++index)
    {
        matrix->data[(index * matrix->columns) + index] = 1.0F;
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_copy(const alg_math_matrix_t *source,
                                       alg_math_matrix_t *destination)
{
    size_t index;

    if (!alg_math_matrix_is_valid(source) || !alg_math_matrix_is_valid(destination))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_matrix_has_same_size(source, destination))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!alg_math_is_finite_array(source->data, source->rows * source->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 0U; index < (source->rows * source->columns); ++index)
    {
        destination->data[index] = source->data[index];
    }
    return ALG_MATH_STATUS_OK;
}

static alg_math_status_t alg_math_matrix_elementwise(const alg_math_matrix_t *left,
                                                     const alg_math_matrix_t *right,
                                                     float right_scale, alg_math_matrix_t *result)
{
    size_t index;
    float value;

    if (!alg_math_matrix_is_valid(left) || !alg_math_matrix_is_valid(right) ||
        !alg_math_matrix_is_valid(result))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_matrix_has_same_size(left, right) || !alg_math_matrix_has_same_size(left, result))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!alg_math_is_finite_array(left->data, left->rows * left->columns) ||
        !alg_math_is_finite_array(right->data, right->rows * right->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 0U; index < (left->rows * left->columns); ++index)
    {
        value = left->data[index] + (right_scale * right->data[index]);
        if (!isfinite(value))
        {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
        }
        result->data[index] = value;
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_add(const alg_math_matrix_t *left, const alg_math_matrix_t *right,
                                      alg_math_matrix_t *result)
{
    return alg_math_matrix_elementwise(left, right, 1.0F, result);
}

alg_math_status_t alg_math_matrix_subtract(const alg_math_matrix_t *left,
                                           const alg_math_matrix_t *right,
                                           alg_math_matrix_t *result)
{
    return alg_math_matrix_elementwise(left, right, -1.0F, result);
}

alg_math_status_t alg_math_matrix_scale(const alg_math_matrix_t *input, float scale,
                                        alg_math_matrix_t *result)
{
    size_t index;
    float value;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(result))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_matrix_has_same_size(input, result))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!isfinite(scale) || !alg_math_is_finite_array(input->data, input->rows * input->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (index = 0U; index < (input->rows * input->columns); ++index)
    {
        value = input->data[index] * scale;
        if (!isfinite(value))
        {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
        }
        result->data[index] = value;
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_multiply(const alg_math_matrix_t *left,
                                           const alg_math_matrix_t *right,
                                           alg_math_matrix_t *result)
{
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    if (!alg_math_matrix_is_valid(left) || !alg_math_matrix_is_valid(right) ||
        !alg_math_matrix_is_valid(result))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((left->columns != right->rows) || (result->rows != left->rows) ||
        (result->columns != right->columns))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if ((result->data == left->data) || (result->data == right->data))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_is_finite_array(left->data, left->rows * left->columns) ||
        !alg_math_is_finite_array(right->data, right->rows * right->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (row = 0U; row < result->rows; ++row)
    {
        for (column = 0U; column < result->columns; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < left->columns; ++shared_index)
            {
                accumulator += left->data[(row * left->columns) + shared_index] *
                               right->data[(shared_index * right->columns) + column];
            }
            if (!isfinite(accumulator))
            {
                return ALG_MATH_STATUS_NUMERICAL_ERROR;
            }
            result->data[(row * result->columns) + column] = accumulator;
        }
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_transpose(const alg_math_matrix_t *input,
                                            alg_math_matrix_t *result)
{
    size_t row;
    size_t column;
    float temporary;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(result))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((result->rows != input->columns) || (result->columns != input->rows))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!alg_math_is_finite_array(input->data, input->rows * input->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    if (input->data == result->data)
    {
        if (input->rows != input->columns)
        {
            return ALG_MATH_STATUS_INVALID_ARGUMENT;
        }
        for (row = 0U; row < input->rows; ++row)
        {
            for (column = row + 1U; column < input->columns; ++column)
            {
                temporary = result->data[(row * result->columns) + column];
                result->data[(row * result->columns) + column] =
                    result->data[(column * result->columns) + row];
                result->data[(column * result->columns) + row] = temporary;
            }
        }
        return ALG_MATH_STATUS_OK;
    }
    for (row = 0U; row < input->rows; ++row)
    {
        for (column = 0U; column < input->columns; ++column)
        {
            result->data[(column * result->columns) + row] =
                input->data[(row * input->columns) + column];
        }
    }
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_matrix_multiply_vector(const alg_math_matrix_t *matrix,
                                                  const float *vector, size_t vector_length,
                                                  float *result, size_t result_length)
{
    size_t row;
    size_t column;
    float accumulator;

    if (!alg_math_matrix_is_valid(matrix) || (vector == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((vector_length != matrix->columns) || (result_length != matrix->rows))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (vector == result)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_is_finite_array(matrix->data, matrix->rows * matrix->columns) ||
        !alg_math_is_finite_array(vector, vector_length))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (row = 0U; row < matrix->rows; ++row)
    {
        accumulator = 0.0F;
        for (column = 0U; column < matrix->columns; ++column)
        {
            accumulator += matrix->data[(row * matrix->columns) + column] * vector[column];
        }
        if (!isfinite(accumulator))
        {
            return ALG_MATH_STATUS_NUMERICAL_ERROR;
        }
        result[row] = accumulator;
    }
    return ALG_MATH_STATUS_OK;
}

static void alg_math_matrix_swap_rows(float *matrix, size_t column_count, size_t first_row,
                                      size_t second_row)
{
    size_t column;
    float temporary;

    for (column = 0U; column < column_count; ++column)
    {
        temporary = matrix[(first_row * column_count) + column];
        matrix[(first_row * column_count) + column] = matrix[(second_row * column_count) + column];
        matrix[(second_row * column_count) + column] = temporary;
    }
}

alg_math_status_t alg_math_matrix_invert(const alg_math_matrix_t *input, alg_math_matrix_t *inverse,
                                         float *workspace, size_t workspace_size)
{
    size_t order;
    size_t augmented_columns;
    size_t row;
    size_t column;
    size_t pivot_row;
    float pivot_magnitude;
    float candidate_magnitude;
    float pivot;
    float factor;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(inverse) ||
        (workspace == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((input->rows != input->columns) || !alg_math_matrix_has_same_size(input, inverse))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    order = input->rows;
    if (workspace_size < ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(order))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!alg_math_is_finite_array(input->data, order * order))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    augmented_columns = 2U * order;
    for (row = 0U; row < order; ++row)
    {
        for (column = 0U; column < order; ++column)
        {
            workspace[(row * augmented_columns) + column] = input->data[(row * order) + column];
            workspace[(row * augmented_columns) + order + column] = (row == column) ? 1.0F : 0.0F;
        }
    }
    for (column = 0U; column < order; ++column)
    {
        pivot_row = column;
        pivot_magnitude = fabsf(workspace[(column * augmented_columns) + column]);
        for (row = column + 1U; row < order; ++row)
        {
            candidate_magnitude = fabsf(workspace[(row * augmented_columns) + column]);
            if (candidate_magnitude > pivot_magnitude)
            {
                pivot_magnitude = candidate_magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude <= ALG_MATH_MATRIX_PIVOT_THRESHOLD)
        {
            return ALG_MATH_STATUS_SINGULAR;
        }
        if (pivot_row != column)
        {
            alg_math_matrix_swap_rows(workspace, augmented_columns, column, pivot_row);
        }
        pivot = workspace[(column * augmented_columns) + column];
        for (size_t index = 0U; index < augmented_columns; ++index)
        {
            workspace[(column * augmented_columns) + index] /= pivot;
        }
        for (row = 0U; row < order; ++row)
        {
            if (row == column)
            {
                continue;
            }
            factor = workspace[(row * augmented_columns) + column];
            for (size_t index = 0U; index < augmented_columns; ++index)
            {
                workspace[(row * augmented_columns) + index] -=
                    factor * workspace[(column * augmented_columns) + index];
            }
        }
    }
    for (row = 0U; row < order; ++row)
    {
        for (column = 0U; column < order; ++column)
        {
            inverse->data[(row * order) + column] =
                workspace[(row * augmented_columns) + order + column];
        }
    }
    return alg_math_is_finite_array(inverse->data, order * order) ? ALG_MATH_STATUS_OK
                                                                  : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_matrix_solve(const alg_math_matrix_t *coefficients,
                                        const float *right_hand_side, float *solution,
                                        float *workspace, size_t workspace_size)
{
    size_t order;
    size_t augmented_columns;
    size_t row;
    size_t column;
    size_t pivot_row;
    float pivot_magnitude;
    float candidate_magnitude;
    float factor;
    float accumulator;

    if (!alg_math_matrix_is_valid(coefficients) || (right_hand_side == NULL) ||
        (solution == NULL) || (workspace == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (coefficients->rows != coefficients->columns)
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    order = coefficients->rows;
    if (workspace_size < ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(order))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (!alg_math_is_finite_array(coefficients->data, order * order) ||
        !alg_math_is_finite_array(right_hand_side, order))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    augmented_columns = order + 1U;
    for (row = 0U; row < order; ++row)
    {
        for (column = 0U; column < order; ++column)
        {
            workspace[(row * augmented_columns) + column] =
                coefficients->data[(row * order) + column];
        }
        workspace[(row * augmented_columns) + order] = right_hand_side[row];
    }
    for (column = 0U; column < order; ++column)
    {
        pivot_row = column;
        pivot_magnitude = fabsf(workspace[(column * augmented_columns) + column]);
        for (row = column + 1U; row < order; ++row)
        {
            candidate_magnitude = fabsf(workspace[(row * augmented_columns) + column]);
            if (candidate_magnitude > pivot_magnitude)
            {
                pivot_magnitude = candidate_magnitude;
                pivot_row = row;
            }
        }
        if (pivot_magnitude <= ALG_MATH_MATRIX_PIVOT_THRESHOLD)
        {
            return ALG_MATH_STATUS_SINGULAR;
        }
        if (pivot_row != column)
        {
            alg_math_matrix_swap_rows(workspace, augmented_columns, column, pivot_row);
        }
        for (row = column + 1U; row < order; ++row)
        {
            factor = workspace[(row * augmented_columns) + column] /
                     workspace[(column * augmented_columns) + column];
            workspace[(row * augmented_columns) + column] = 0.0F;
            for (size_t index = column + 1U; index < augmented_columns; ++index)
            {
                workspace[(row * augmented_columns) + index] -=
                    factor * workspace[(column * augmented_columns) + index];
            }
        }
    }
    for (row = order; row-- > 0U;)
    {
        accumulator = workspace[(row * augmented_columns) + order];
        for (column = row + 1U; column < order; ++column)
        {
            accumulator -= workspace[(row * augmented_columns) + column] * solution[column];
        }
        if (fabsf(workspace[(row * augmented_columns) + row]) <= ALG_MATH_MATRIX_PIVOT_THRESHOLD)
        {
            return ALG_MATH_STATUS_SINGULAR;
        }
        solution[row] = accumulator / workspace[(row * augmented_columns) + row];
    }
    return alg_math_is_finite_array(solution, order) ? ALG_MATH_STATUS_OK
                                                     : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_matrix_cholesky(const alg_math_matrix_t *input,
                                           alg_math_matrix_t *lower_triangular)
{
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    if (!alg_math_matrix_is_valid(input) || !alg_math_matrix_is_valid(lower_triangular))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if ((input->rows != input->columns) || !alg_math_matrix_has_same_size(input, lower_triangular))
    {
        return ALG_MATH_STATUS_SIZE_MISMATCH;
    }
    if (input->data == lower_triangular->data)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_is_finite_array(input->data, input->rows * input->columns))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    for (row = 0U; row < input->rows; ++row)
    {
        for (column = row + 1U; column < input->columns; ++column)
        {
            const float upper = input->data[(row * input->columns) + column];
            const float lower = input->data[(column * input->columns) + row];
            const float tolerance = 1.0e-5F * (1.0F + fmaxf(fabsf(upper), fabsf(lower)));

            if (fabsf(upper - lower) > tolerance)
            {
                return ALG_MATH_STATUS_OUT_OF_RANGE;
            }
        }
    }
    (void)alg_math_matrix_zero(lower_triangular);
    for (row = 0U; row < input->rows; ++row)
    {
        for (column = 0U; column <= row; ++column)
        {
            accumulator = input->data[(row * input->columns) + column];
            for (shared_index = 0U; shared_index < column; ++shared_index)
            {
                accumulator -=
                    lower_triangular->data[(row * lower_triangular->columns) + shared_index] *
                    lower_triangular->data[(column * lower_triangular->columns) + shared_index];
            }
            if (row == column)
            {
                if (accumulator <= ALG_MATH_MATRIX_PIVOT_THRESHOLD)
                {
                    return ALG_MATH_STATUS_SINGULAR;
                }
                lower_triangular->data[(row * lower_triangular->columns) + column] =
                    sqrtf(accumulator);
            }
            else
            {
                lower_triangular->data[(row * lower_triangular->columns) + column] =
                    accumulator /
                    lower_triangular->data[(column * lower_triangular->columns) + column];
            }
        }
    }
    return alg_math_is_finite_array(lower_triangular->data,
                                    lower_triangular->rows * lower_triangular->columns)
               ? ALG_MATH_STATUS_OK
               : ALG_MATH_STATUS_NUMERICAL_ERROR;
}
