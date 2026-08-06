/**
 * @file alg_lqr_model.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模型变换：Tustin 离散化、LQI 增广
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供连续→离散转换和积分增广功能。
 *       Tustin 双线性变换在较大采样周期下稳定性优于前向欧拉。
 */

#include "alg_lqr_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief Tustin 双线性离散化
 */
alg_lqr_status_t alg_lqr_discretize_tustin(const float *continuous_state_matrix,
                                           const float *continuous_control_matrix,
                                           size_t state_dimension, size_t control_dimension,
                                           float delta_time_s, float *discrete_state_matrix,
                                           float *discrete_control_matrix, float *workspace,
                                           size_t workspace_size)
{
    size_t state_square = state_dimension * state_dimension;
    size_t cross_size = state_dimension * control_dimension;
    size_t row, column;
    float *left_matrix = workspace;
    float *left_inverse = left_matrix + state_square;
    float *right_matrix = left_inverse + state_square;
    float *scaled_control = right_matrix + state_square;
    alg_lqr_status_t status;

    // ---- 参数检查 ----
    if ((continuous_state_matrix == NULL) || (continuous_control_matrix == NULL) ||
        (discrete_state_matrix == NULL) || (discrete_control_matrix == NULL) || (workspace == NULL)) {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
}
    if ((state_dimension == 0U) || (control_dimension == 0U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}
    if (workspace_size < ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(state_dimension, control_dimension)) {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
}
    if (!alg_lqr_internal_is_finite_array(continuous_state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(continuous_control_matrix, cross_size)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}

    // ---- 构造 (I - A*dt/2) 和 (I + A*dt/2) ----
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
    // B 缩放
    for (row = 0U; row < cross_size; ++row) {
        scaled_control[row] = delta_time_s * continuous_control_matrix[row];
}

    // ---- 求逆并计算 A_d = L^{-1} * R, B_d = L^{-1} * (B*dt) ----
    status = alg_lqr_internal_invert(left_matrix, left_inverse, state_dimension);
    if (status != ALG_LQR_STATUS_OK) {
        return status;
}
    alg_lqr_internal_multiply(left_inverse, state_dimension, state_dimension, right_matrix,
                              state_dimension, discrete_state_matrix);
    alg_lqr_internal_multiply(left_inverse, state_dimension, state_dimension, scaled_control,
                              control_dimension, discrete_control_matrix);

    // 结果检查
    if (!alg_lqr_internal_is_finite_array(discrete_state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(discrete_control_matrix, cross_size)) {
        return ALG_LQR_STATUS_NUMERICAL_ERROR;
}
    return ALG_LQR_STATUS_OK;
}

/**
 * @brief LQI 增广模型构建
 */
alg_lqr_status_t alg_lqr_lqi_build_augmented_model(
    const float *state_matrix, const float *control_matrix, const float *output_matrix,
    size_t state_dimension, size_t control_dimension, size_t integral_dimension, float delta_time_s,
    float *augmented_state_matrix, float *augmented_control_matrix)
{
    size_t augmented_dimension = state_dimension + integral_dimension;
    size_t row, column;

    // ---- 参数检查 ----
    if ((state_matrix == NULL) || (control_matrix == NULL) || (output_matrix == NULL) ||
        (augmented_state_matrix == NULL) || (augmented_control_matrix == NULL)) {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
}
    if ((state_dimension == 0U) || (control_dimension == 0U) || (integral_dimension == 0U) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}
    if (!alg_lqr_internal_is_finite_array(state_matrix, state_dimension * state_dimension) ||
        !alg_lqr_internal_is_finite_array(control_matrix, state_dimension * control_dimension) ||
        !alg_lqr_internal_is_finite_array(output_matrix, integral_dimension * state_dimension)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}

    // ---- 初始化零矩阵 ----
    for (row = 0U; row < augmented_dimension; ++row)
    {
        for (column = 0U; column < augmented_dimension; ++column) {
            augmented_state_matrix[(row * augmented_dimension) + column] = 0.0F;
}
        for (column = 0U; column < control_dimension; ++column) {
            augmented_control_matrix[(row * control_dimension) + column] = 0.0F;
}
    }

    // ---- 填充左上块 A ----
    for (row = 0U; row < state_dimension; ++row)
    {
        for (column = 0U; column < state_dimension; ++column) {
            augmented_state_matrix[(row * augmented_dimension) + column] =
                state_matrix[(row * state_dimension) + column];
}
        for (column = 0U; column < control_dimension; ++column) {
            augmented_control_matrix[(row * control_dimension) + column] =
                control_matrix[(row * control_dimension) + column];
}
    }

    // ---- 填充积分块 [-dt*C, I] ----
    for (row = 0U; row < integral_dimension; ++row)
    {
        for (column = 0U; column < state_dimension; ++column) {
            augmented_state_matrix[((state_dimension + row) * augmented_dimension) + column] =
                -delta_time_s * output_matrix[(row * state_dimension) + column];
}
        augmented_state_matrix[((state_dimension + row) * augmented_dimension) + state_dimension +
                               row] = 1.0F;
    }
    return ALG_LQR_STATUS_OK;
}