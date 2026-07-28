/**
 * @file alg_kalman_linear.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 线性卡尔曼滤波器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持任意维度的线性卡尔曼滤波。
 *       模型：x = F*x + B*u，z = H*x + v
 *       所有矩阵由调用者提供，滤波器不复制矩阵。
 *       支持时变模型（预测前可修改 F、B、Q、H、R）。
 */

#include "alg_kalman_internal.h"

#include <stddef.h> // NULL

/**
 * @brief 验证线性卡尔曼配置的有效性
 * @param config 配置指针
 * @return 执行状态
 */
static alg_kalman_status_t
alg_kalman_linear_validate_config(const alg_kalman_linear_config_t *config)
{
    size_t required_workspace;
    size_t state_square;
    size_t measurement_square;

    // ---- 基本指针检查 ----
    if ((config == NULL) || (config->state == NULL) || (config->covariance == NULL) ||
        (config->transition_matrix == NULL) || (config->process_noise == NULL) ||
        (config->measurement_matrix == NULL) || (config->measurement_noise == NULL) ||
        (config->workspace == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;

    // ---- 维度检查 ----
    if ((config->state_dimension == 0U) || (config->measurement_dimension == 0U))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    // ---- 控制矩阵检查 ----
    if ((config->control_dimension > 0U) && (config->control_matrix == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;

    // ---- 工作区大小检查 ----
    required_workspace =
        ALG_KALMAN_WORKSPACE_SIZE(config->state_dimension, config->measurement_dimension);
    if (config->workspace_size < required_workspace)
        return ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE;

    // ---- 矩阵内容检查 ----
    state_square = config->state_dimension * config->state_dimension;
    measurement_square = config->measurement_dimension * config->measurement_dimension;

    if (!alg_kalman_internal_is_finite_array(config->state, config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(config->covariance, state_square) ||
        !alg_kalman_internal_is_finite_array(config->transition_matrix, state_square) ||
        !alg_kalman_internal_is_finite_array(config->process_noise, state_square) ||
        !alg_kalman_internal_is_finite_array(
            config->measurement_matrix, config->measurement_dimension * config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(config->measurement_noise, measurement_square) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->covariance,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->process_noise,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->measurement_noise,
                                                      config->measurement_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    if ((config->control_dimension > 0U) &&
        !alg_kalman_internal_is_finite_array(config->control_matrix,
                                             config->state_dimension * config->control_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 初始化线性卡尔曼滤波器
 * @param me 滤波器对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_linear_init(alg_kalman_linear_t *me,
                                           const alg_kalman_linear_config_t *config)
{
    alg_kalman_status_t status;

    if (me == NULL)
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;

    status = alg_kalman_linear_validate_config(config);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    me->config = *config;
    alg_kalman_internal_symmetrize(me->config.covariance, me->config.state_dimension);
    me->is_initialized = true;

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 重置线性卡尔曼滤波器
 * @param me 滤波器对象
 * @param initial_state 初始状态
 * @param initial_covariance 初始协方差
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_linear_reset(alg_kalman_linear_t *me, const float *initial_state,
                                            const float *initial_covariance)
{
    size_t state_square;

    if ((me == NULL) || (initial_state == NULL) || (initial_covariance == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;

    state_square = me->config.state_dimension * me->config.state_dimension;
    if (!alg_kalman_internal_is_finite_array(initial_state, me->config.state_dimension) ||
        !alg_kalman_internal_is_finite_array(initial_covariance, state_square) ||
        !alg_kalman_internal_has_nonnegative_diagonal(initial_covariance,
                                                      me->config.state_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    alg_kalman_internal_copy(me->config.state, initial_state, me->config.state_dimension);
    alg_kalman_internal_copy(me->config.covariance, initial_covariance, state_square);
    alg_kalman_internal_symmetrize(me->config.covariance, me->config.state_dimension);

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 线性卡尔曼预测步骤
 * @param me 滤波器对象
 * @param control_input 控制输入（c×1），c=0 时可 NULL
 * @return 执行状态
 * @note x = F*x + B*u，P = F*P*F^T + Q
 *       控制矩阵 B 在配置中提供
 */
alg_kalman_status_t alg_kalman_linear_predict(alg_kalman_linear_t *me, const float *control_input)
{
    const alg_kalman_linear_config_t *config;
    size_t state_square;
    size_t state_index;
    size_t control_index;
    float *predicted_state;
    float *temporary_covariance;
    float *predicted_covariance;

    if (me == NULL)
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;

    config = &me->config;

    // 控制输入检查
    if ((config->control_dimension > 0U) && (control_input == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if ((config->control_dimension > 0U) &&
        !alg_kalman_internal_is_finite_array(control_input, config->control_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    state_square = config->state_dimension * config->state_dimension;
    predicted_state = config->workspace;
    temporary_covariance = predicted_state + config->state_dimension;
    predicted_covariance = temporary_covariance + state_square;

    // ---- 状态预测：x = F*x ----
    alg_kalman_internal_multiply(config->transition_matrix, config->state_dimension,
                                 config->state_dimension, config->state, 1U, predicted_state);

    // ---- 控制输入：x = x + B*u ----
    if (config->control_dimension > 0U)
    {
        for (state_index = 0U; state_index < config->state_dimension; ++state_index)
        {
            for (control_index = 0U; control_index < config->control_dimension; ++control_index)
            {
                predicted_state[state_index] +=
                    config->control_matrix[(state_index * config->control_dimension) +
                                           control_index] *
                    control_input[control_index];
            }
        }
    }

    // ---- 协方差预测：P = F*P*F^T + Q ----
    alg_kalman_internal_multiply(config->transition_matrix, config->state_dimension,
                                 config->state_dimension, config->covariance,
                                 config->state_dimension, temporary_covariance);
    alg_kalman_internal_multiply_right_transpose(temporary_covariance, config->state_dimension,
                                                 config->state_dimension, config->transition_matrix,
                                                 config->state_dimension, predicted_covariance);

    for (state_index = 0U; state_index < state_square; ++state_index)
        predicted_covariance[state_index] += config->process_noise[state_index];

    alg_kalman_internal_symmetrize(predicted_covariance, config->state_dimension);

    // ---- 检查结果有效性 ----
    if (!alg_kalman_internal_is_finite_array(predicted_state, config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(predicted_covariance, state_square))
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;

    // ---- 提交更新 ----
    alg_kalman_internal_copy(config->state, predicted_state, config->state_dimension);
    alg_kalman_internal_copy(config->covariance, predicted_covariance, state_square);

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 线性卡尔曼校正步骤
 * @param me 滤波器对象
 * @param measurement 测量值（m×1）
 * @return 执行状态
 * @note 使用内部 alg_kalman_internal_correct 执行校正
 */
alg_kalman_status_t alg_kalman_linear_correct(alg_kalman_linear_t *me, const float *measurement)
{
    const alg_kalman_linear_config_t *config;
    float *predicted_measurement;

    if ((me == NULL) || (measurement == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;

    config = &me->config;
    if (!alg_kalman_internal_is_finite_array(measurement, config->measurement_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    // ---- 计算预测测量值 z_pred = H*x ----
    predicted_measurement = config->workspace;
    alg_kalman_internal_multiply(config->measurement_matrix, config->measurement_dimension,
                                 config->state_dimension, config->state, 1U, predicted_measurement);

    // ---- 执行校正 ----
    return alg_kalman_internal_correct(
        config->state, config->covariance, config->state_dimension, config->measurement_matrix,
        config->measurement_noise, measurement, predicted_measurement,
        config->measurement_dimension, config->workspace + config->measurement_dimension,
        config->workspace_size - config->measurement_dimension);
}

/**
 * @brief 获取当前状态（只读）
 * @param me 滤波器对象
 * @return 状态指针，未初始化则返回 NULL
 */
const float *alg_kalman_linear_get_state(const alg_kalman_linear_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.state : NULL;
}

/**
 * @brief 获取当前协方差矩阵（只读）
 * @param me 滤波器对象
 * @return 协方差指针，未初始化则返回 NULL
 */
const float *alg_kalman_linear_get_covariance(const alg_kalman_linear_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.covariance : NULL;
}