/**
 * @file alg_lqr_riccati.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief Riccati 迭代与 DARE/有限时域求解器
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现一步 Riccati 迭代、DARE 迭代求解和有限时域反向递推。
 *       矩阵求逆采用部分主元 Gauss-Jordan，检测奇异。
 *       DARE 使用最大元素变化量判断收敛。
 */

#include "alg_lqr_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 一步 Riccati 迭代
 */
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
    alg_lqr_status_t status;

    // ---- 工作区分配 ----
    float *riccati_control = workspace;                     // P*B
    float *gain_denominator = riccati_control + cross_size; // BᵀPB + R
    float *gain_denominator_inverse = gain_denominator + control_square;
    float *riccati_state = gain_denominator_inverse + control_square;   // P*A
    float *gain_numerator = riccati_state + state_square;               // BᵀPA + Nᵀ
    float *state_transpose_riccati_state = gain_numerator + cross_size; // AᵀPA
    float *state_transpose_riccati_control =
        state_transpose_riccati_state + state_square;                    // AᵀPB + N
    float *feedback_cost = state_transpose_riccati_control + cross_size; // (AᵀPB+N)K

    if (workspace_size < required_size) {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
}

    // ---- 计算增益分母：S = BᵀPB + R ----
    alg_lqr_internal_multiply(next_riccati, state_dimension, state_dimension, control_matrix,
                              control_dimension, riccati_control);
    alg_lqr_internal_multiply_left_transpose(control_matrix, state_dimension, control_dimension,
                                             riccati_control, control_dimension, gain_denominator);
    for (size_t i = 0U; i < control_square; ++i) {
        gain_denominator[i] += control_weight[i];
}

    // ---- 计算增益分子：BᵀPA + Nᵀ ----
    alg_lqr_internal_multiply(next_riccati, state_dimension, state_dimension, state_matrix,
                              state_dimension, riccati_state);
    alg_lqr_internal_multiply_left_transpose(control_matrix, state_dimension, control_dimension,
                                             riccati_state, state_dimension, gain_numerator);
    if (cross_weight != NULL)
    {
        for (size_t row = 0U; row < control_dimension; ++row) {
            for (size_t col = 0U; col < state_dimension; ++col) {
                gain_numerator[(row * state_dimension) + col] +=
                    cross_weight[(col * control_dimension) + row];
}
}
    }

    // ---- 求逆并计算增益 K = S⁻¹ * (BᵀPA + Nᵀ) ----
    status = alg_lqr_internal_invert(gain_denominator, gain_denominator_inverse, control_dimension);
    if (status != ALG_LQR_STATUS_OK) {
        return status;
}
    alg_lqr_internal_multiply(gain_denominator_inverse, control_dimension, control_dimension,
                              gain_numerator, state_dimension, gain_matrix);

    // ---- 计算当前 P = Q + AᵀPA - (AᵀPB + N)K ----
    alg_lqr_internal_multiply_left_transpose(state_matrix, state_dimension, state_dimension,
                                             riccati_state, state_dimension,
                                             state_transpose_riccati_state);
    alg_lqr_internal_multiply_left_transpose(state_matrix, state_dimension, state_dimension,
                                             riccati_control, control_dimension,
                                             state_transpose_riccati_control);
    if (cross_weight != NULL)
    {
        for (size_t i = 0U; i < cross_size; ++i) {
            state_transpose_riccati_control[i] += cross_weight[i];
}
    }
    alg_lqr_internal_multiply(state_transpose_riccati_control, state_dimension, control_dimension,
                              gain_matrix, state_dimension, feedback_cost);

    for (size_t i = 0U; i < state_square; ++i) {
        current_riccati[i] = state_weight[i] + state_transpose_riccati_state[i] - feedback_cost[i];
}
    alg_lqr_internal_symmetrize(current_riccati, state_dimension);

    // ---- 结果检查 ----
    if (!alg_lqr_internal_is_finite_array(current_riccati, state_square) ||
        !alg_lqr_internal_is_finite_array(gain_matrix, cross_size)) {
        return ALG_LQR_STATUS_NUMERICAL_ERROR;
}
    return ALG_LQR_STATUS_OK;
}

/**
 * @brief 验证 DARE 配置
 */
static alg_lqr_status_t alg_lqr_dare_validate(const alg_lqr_dare_config_t *config)
{
    size_t state_square = config->state_dimension * config->state_dimension;
    size_t control_square = config->control_dimension * config->control_dimension;
    size_t cross_size = config->state_dimension * config->control_dimension;

    if ((config == NULL) || (config->state_matrix == NULL) || (config->control_matrix == NULL) ||
        (config->state_weight == NULL) || (config->control_weight == NULL) ||
        (config->workspace == NULL)) {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
}
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        !isfinite(config->tolerance) || (config->tolerance <= 0.0F) ||
        (config->maximum_iterations == 0U)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}
    if (config->workspace_size <
        ALG_LQR_RICCATI_WORKSPACE_SIZE(config->state_dimension, config->control_dimension)) {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
}

    if (!alg_lqr_internal_is_finite_array(config->state_matrix, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_matrix, cross_size) ||
        !alg_lqr_internal_is_finite_array(config->state_weight, state_square) ||
        !alg_lqr_internal_is_finite_array(config->control_weight, control_square) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->state_weight, config->state_dimension) ||
        !alg_lqr_internal_has_nonnegative_diagonal(config->control_weight,
                                                   config->control_dimension) ||
        ((config->cross_weight != NULL) &&
         !alg_lqr_internal_is_finite_array(config->cross_weight, cross_size))) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}
    return ALG_LQR_STATUS_OK;
}

/**
 * @brief 无限时域 DARE 求解
 */
alg_lqr_status_t alg_lqr_dare_solve(const alg_lqr_dare_config_t *config, float *riccati_solution,
                                    float *gain_matrix, size_t *completed_iterations)
{
    if ((riccati_solution == NULL) || (gain_matrix == NULL)) {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
}

    alg_lqr_status_t status = alg_lqr_dare_validate(config);
    if (status != ALG_LQR_STATUS_OK) {
        return status;
}

    size_t state_square = config->state_dimension * config->state_dimension;
    float *candidate_riccati = config->workspace;
    float *step_workspace = candidate_riccati + state_square;
    size_t step_workspace_size = config->workspace_size - state_square;

    // 初始 P = Q
    alg_lqr_internal_copy(riccati_solution, config->state_weight, state_square);

    // ---- 迭代 ----
    for (size_t iteration = 1U; iteration <= config->maximum_iterations; ++iteration)
    {
        status = alg_lqr_internal_riccati_step(
            config->state_dimension, config->control_dimension, config->state_matrix,
            config->control_matrix, config->state_weight, config->control_weight,
            config->cross_weight, riccati_solution, candidate_riccati, gain_matrix, step_workspace,
            step_workspace_size);
        if (status != ALG_LQR_STATUS_OK) {
            return status;
}

        // 计算最大变化量
        float max_diff = 0.0F;
        for (size_t i = 0U; i < state_square; ++i)
        {
            float diff = fabsf(candidate_riccati[i] - riccati_solution[i]);
            if (diff > max_diff) {
                max_diff = diff;
}
        }
        alg_lqr_internal_copy(riccati_solution, candidate_riccati, state_square);

        // 收敛判断
        if (max_diff <= config->tolerance)
        {
            // 额外一步以更新增益（可选）
            status = alg_lqr_internal_riccati_step(
                config->state_dimension, config->control_dimension, config->state_matrix,
                config->control_matrix, config->state_weight, config->control_weight,
                config->cross_weight, riccati_solution, candidate_riccati, gain_matrix,
                step_workspace, step_workspace_size);
            if (status != ALG_LQR_STATUS_OK) {
                return status;
}
            if (completed_iterations != NULL) {
                *completed_iterations = iteration;
}
            return ALG_LQR_STATUS_OK;
        }
    }

    if (completed_iterations != NULL) {
        *completed_iterations = config->maximum_iterations;
}
    return ALG_LQR_STATUS_NOT_CONVERGED;
}

/**
 * @brief 有限时域 LQR 求解
 */
alg_lqr_status_t alg_lqr_finite_solve(const alg_lqr_finite_config_t *config, float *gain_sequence,
                                      float *initial_riccati_solution)
{
    size_t state_square = config->state_dimension * config->state_dimension;
    size_t control_square = config->control_dimension * config->control_dimension;
    size_t cross_size = config->state_dimension * config->control_dimension;

    // ---- 参数检查 ----
    if ((config == NULL) || (gain_sequence == NULL) || (initial_riccati_solution == NULL) ||
        (config->state_matrix == NULL) || (config->control_matrix == NULL) ||
        (config->state_weight == NULL) || (config->control_weight == NULL) ||
        (config->terminal_state_weight == NULL) || (config->workspace == NULL)) {
        return ALG_LQR_STATUS_INVALID_ARGUMENT;
}
    if ((config->state_dimension == 0U) || (config->control_dimension == 0U) ||
        (config->horizon_length == 0U)) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}
    if (config->workspace_size <
        ALG_LQR_FINITE_WORKSPACE_SIZE(config->state_dimension, config->control_dimension)) {
        return ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE;
}

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
         !alg_lqr_internal_is_finite_array(config->cross_weight, cross_size))) {
        return ALG_LQR_STATUS_OUT_OF_RANGE;
}

    // ---- 反向递推 ----
    float *next_riccati = config->workspace;
    float *step_workspace = next_riccati + state_square;
    size_t step_workspace_size = config->workspace_size - state_square;
    alg_lqr_internal_copy(next_riccati, config->terminal_state_weight, state_square);
    alg_lqr_internal_symmetrize(next_riccati, config->state_dimension);

    for (size_t step = config->horizon_length; step > 0U; --step)
    {
        alg_lqr_status_t status = alg_lqr_internal_riccati_step(
            config->state_dimension, config->control_dimension, config->state_matrix,
            config->control_matrix, config->state_weight, config->control_weight,
            config->cross_weight, next_riccati, initial_riccati_solution,
            &gain_sequence[(step - 1U) * config->control_dimension * config->state_dimension],
            step_workspace, step_workspace_size);
        if (status != ALG_LQR_STATUS_OK) {
            return status;
}
        alg_lqr_internal_copy(next_riccati, initial_riccati_solution, state_square);
    }
    return ALG_LQR_STATUS_OK;
}