#include "alg_lqr_internal.h"

#include <math.h>
#include <stddef.h>

#define ALG_LQR_SINGULAR_THRESHOLD (1.0e-12F)

bool AlgLqrInternal_IsFiniteArray(const float *values, size_t value_count)
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

bool AlgLqrInternal_HasNonnegativeDiagonal(const float *matrix,
                                           size_t dimension)
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

void AlgLqrInternal_Copy(float *destination,
                         const float *source,
                         size_t value_count)
{
    size_t index;

    for (index = 0U; index < value_count; ++index)
    {
        destination[index] = source[index];
    }
}

void AlgLqrInternal_Multiply(const float *left,
                             size_t left_rows,
                             size_t shared_dimension,
                             const float *right,
                             size_t right_columns,
                             float *output)
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

void AlgLqrInternal_MultiplyLeftTranspose(const float *left,
                                          size_t left_rows,
                                          size_t left_columns,
                                          const float *right,
                                          size_t right_columns,
                                          float *output)
{
    size_t output_row;
    size_t output_column;
    size_t shared_index;
    float accumulator;

    for (output_row = 0U; output_row < left_columns; ++output_row)
    {
        for (output_column = 0U; output_column < right_columns; ++output_column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < left_rows; ++shared_index)
            {
                accumulator += left[(shared_index * left_columns) + output_row] *
                               right[(shared_index * right_columns) + output_column];
            }
            output[(output_row * right_columns) + output_column] = accumulator;
        }
    }
}

void AlgLqrInternal_Symmetrize(float *matrix, size_t dimension)
{
    size_t row;
    size_t column;
    float average;

    for (row = 0U; row < dimension; ++row)
    {
        for (column = row + 1U; column < dimension; ++column)
        {
            average = 0.5F * (matrix[(row * dimension) + column] +
                              matrix[(column * dimension) + row]);
            matrix[(row * dimension) + column] = average;
            matrix[(column * dimension) + row] = average;
        }
    }
}

AlgLqrStatus_t AlgLqrInternal_Invert(float *matrix,
                                     float *inverse,
                                     size_t dimension)
{
    size_t row;
    size_t column;
    size_t pivot_row;
    size_t element;
    float pivot_magnitude;
    float candidate_magnitude;
    float pivot_value;
    float factor;
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
        if (!isfinite(pivot_magnitude) ||
            (pivot_magnitude <= ALG_LQR_SINGULAR_THRESHOLD))
        {
            return ALG_LQR_STATUS_SINGULAR_MATRIX;
        }

        if (pivot_row != column)
        {
            for (element = 0U; element < dimension; ++element)
            {
                temporary = matrix[(column * dimension) + element];
                matrix[(column * dimension) + element] =
                    matrix[(pivot_row * dimension) + element];
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
            factor = matrix[(row * dimension) + column];
            for (element = 0U; element < dimension; ++element)
            {
                matrix[(row * dimension) + element] -=
                    factor * matrix[(column * dimension) + element];
                inverse[(row * dimension) + element] -=
                    factor * inverse[(column * dimension) + element];
            }
        }
    }

    return AlgLqrInternal_IsFiniteArray(inverse, dimension * dimension)
               ? ALG_LQR_STATUS_OK
               : ALG_LQR_STATUS_NUMERICAL_ERROR;
}
