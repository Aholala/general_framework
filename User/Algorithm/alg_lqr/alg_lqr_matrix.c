/**
 * @file alg_lqr_matrix.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 矩阵运算、求逆、检查与对称化工具
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供内部矩阵基础操作，无动态内存。
 *       奇异阈值设为 1e-12，用于检测近似奇异矩阵。
 */

#include "alg_lqr_internal.h"
#include <math.h>
#include <stddef.h>

/** @brief 奇异阈值，用于判断矩阵是否可逆 */
#define ALG_LQR_SINGULAR_THRESHOLD (1.0e-12F)

/**
 * @brief 检查数组是否全为有限数
 */
bool alg_lqr_internal_is_finite_array(const float *values, size_t value_count)
{
    size_t index;
    if (values == NULL) {
        return false;
}
    for (index = 0U; index < value_count; ++index) {
        if (!isfinite(values[index])) {
            return false;
}
}
    return true;
}

/**
 * @brief 检查方阵对角线是否全为非负有限数
 */
bool alg_lqr_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension)
{
    size_t index;
    if (matrix == NULL) {
        return false;
}
    for (index = 0U; index < dimension; ++index)
    {
        if (!isfinite(matrix[(index * dimension) + index]) ||
            (matrix[(index * dimension) + index] < 0.0F)) {
            return false;
}
    }
    return true;
}

/**
 * @brief 拷贝数组
 */
void alg_lqr_internal_copy(float *destination, const float *source, size_t value_count)
{
    size_t index;
    for (index = 0U; index < value_count; ++index) {
        destination[index] = source[index];
}
}

/**
 * @brief 矩阵乘法 C = A * B
 */
void alg_lqr_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                               const float *right, size_t right_columns, float *output)
{
    size_t row, column, shared_index;
    float accumulator;
    for (row = 0U; row < left_rows; ++row)
    {
        for (column = 0U; column < right_columns; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < shared_dimension; ++shared_index) {
                accumulator += left[(row * shared_dimension) + shared_index] *
                               right[(shared_index * right_columns) + column];
}
            output[(row * right_columns) + column] = accumulator;
        }
    }
}

/**
 * @brief 矩阵乘法 C = A^T * B
 */
void alg_lqr_internal_multiply_left_transpose(const float *left, size_t left_rows,
                                              size_t left_columns, const float *right,
                                              size_t right_columns, float *output)
{
    size_t output_row, output_column, shared_index;
    float accumulator;
    for (output_row = 0U; output_row < left_columns; ++output_row)
    {
        for (output_column = 0U; output_column < right_columns; ++output_column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < left_rows; ++shared_index) {
                accumulator += left[(shared_index * left_columns) + output_row] *
                               right[(shared_index * right_columns) + output_column];
}
            output[(output_row * right_columns) + output_column] = accumulator;
        }
    }
}

/**
 * @brief 对称化：P = (P + P^T)/2
 */
void alg_lqr_internal_symmetrize(float *matrix, size_t dimension)
{
    size_t row, column;
    float average;
    for (row = 0U; row < dimension; ++row) {
        for (column = row + 1U; column < dimension; ++column)
        {
            average =
                0.5F * (matrix[(row * dimension) + column] + matrix[(column * dimension) + row]);
            matrix[(row * dimension) + column] = average;
            matrix[(column * dimension) + row] = average;
        }
}
}

/**
 * @brief 矩阵求逆（部分主元 Gauss-Jordan）
 */
alg_lqr_status_t alg_lqr_internal_invert(float *matrix, float *inverse, size_t dimension)
{
    size_t row, column, pivot_row, element;
    float pivot_magnitude, candidate_magnitude, pivot_value, factor, temporary;

    // 初始化为单位矩阵
    for (row = 0U; row < dimension; ++row) {
        for (column = 0U; column < dimension; ++column) {
            inverse[(row * dimension) + column] = (row == column) ? 1.0F : 0.0F;
}
}

    // 主循环
    for (column = 0U; column < dimension; ++column)
    {
        // 选择主元
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
        if (!isfinite(pivot_magnitude) || (pivot_magnitude <= ALG_LQR_SINGULAR_THRESHOLD)) {
            return ALG_LQR_STATUS_SINGULAR_MATRIX;
}

        // 交换行
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

        // 归一化主元行
        pivot_value = matrix[(column * dimension) + column];
        for (element = 0U; element < dimension; ++element)
        {
            matrix[(column * dimension) + element] /= pivot_value;
            inverse[(column * dimension) + element] /= pivot_value;
        }

        // 消去其他行
        for (row = 0U; row < dimension; ++row)
        {
            if (row == column) {
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

    return alg_lqr_internal_is_finite_array(inverse, dimension * dimension)
               ? ALG_LQR_STATUS_OK
               : ALG_LQR_STATUS_NUMERICAL_ERROR;
}