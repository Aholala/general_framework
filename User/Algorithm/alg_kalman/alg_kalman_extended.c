/**
 * @file alg_kalman_extended.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 扩展卡尔曼滤波器（EKF）实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持非线性状态模型和非线性观测模型。
 *       使用线性化（雅可比矩阵）进行协方差传播。
 *       模型：x = f(x, u, dt)，z = h(x)
 *       需要调用者提供四个模型回调函数。
 */

#include "alg_kalman_internal.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 验证扩展卡尔曼配置的有效性
 * @param config 配置指针
 * @return 执行状态
 */
static alg_kalman_status_t
alg_kalman_extended_validate_config(const alg_kalman_extended_config_t *config)
{
    size_t required_workspace;
    size_t state_square;
    size_t measurement_square;

    // ---- 基本指针检查 ----
    if ((config == NULL) || (config->state == NULL) || (config->covariance == NULL) ||
        (config->process_noise == NULL) || (config->measurement_noise == NULL) ||
        (config->workspace == NULL) || (config->state_function == NULL) ||
        (config->state_jacobian_function == NULL) || (config->measurement_function == NULL) ||
        (config->measurement_jacobian_function == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;

    // ---- 维度检查 ----
    if ((config->state_dimension == 0U) || (config->measurement_dimension == 0U))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

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
        !alg_kalman_internal_is_finite_array(config->process_noise, state_square) ||
        !alg_kalman_internal_is_finite_array(config->measurement_noise, measurement_square) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->covariance,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->process_noise,
                                                      config->state_dimension) ||
        !alg_kalman_internal_has_nonnegative_diagonal(config->measurement_noise,
                                                      config->measurement_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 初始化扩展卡尔曼滤波器
 * @param me 滤波器对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_extended_init(alg_kalman_extended_t *me,
                                             const alg_kalman_extended_config_t *config)
{
    alg_kalman_status_t status;

    if (me == NULL)
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;

    status = alg_kalman_extended_validate_config(config);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    me->config = *config;
    alg_kalman_internal_symmetrize(me->config.covariance, me->config.state_dimension);
    me->is_initialized = true;

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 重置扩展卡尔曼滤波器
 * @param me 滤波器对象
 * @param initial_state 初始状态
 * @param initial_covariance 初始协方差
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_extended_reset(alg_kalman_extended_t *me, const float *initial_state,
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
 * @brief 扩展卡尔曼预测步骤
 * @param me 滤波器对象
 * @param control_input 控制输入（c×1），c=0 时可 NULL
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note x = f(x, u, dt)，P = F*P*F^T + Q
 *       其中 F = ∂f/∂x 由状态雅可比回调提供
 */
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
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
    if (!isfinite(delta_time_s) || (delta_time_s <= 0.0F))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    config = &me->config;

    // 控制输入检查
    if ((config->control_dimension > 0U) && (control_input == NULL))
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if ((config->control_dimension > 0U) &&
        !alg_kalman_internal_is_finite_array(control_input, config->control_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    state_square = config->state_dimension * config->state_dimension;
    predicted_state = config->workspace;
    state_jacobian = predicted_state + config->state_dimension;
    temporary_covariance = state_jacobian + state_square;
    predicted_covariance = temporary_covariance + state_square;

    // ---- 1. 调用状态转移函数计算预测状态 ----
    status = config->state_function(config->state, config->state_dimension, control_input,
                                    config->control_dimension, delta_time_s, predicted_state,
                                    config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    // ---- 2. 调用状态雅可比函数计算 F = ∂f/∂x ----
    status = config->state_jacobian_function(config->state, config->state_dimension, control_input,
                                             config->control_dimension, delta_time_s,
                                             state_jacobian, config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    // ---- 3. 检查模型输出有效性 ----
    if (!alg_kalman_internal_is_finite_array(predicted_state, config->state_dimension) ||
        !alg_kalman_internal_is_finite_array(state_jacobian, state_square))
        return ALG_KALMAN_STATUS_MODEL_ERROR;

    // ---- 4. 协方差预测：P = F*P*F^T + Q ----
    alg_kalman_internal_multiply(state_jacobian, config->state_dimension, config->state_dimension,
                                 config->covariance, config->state_dimension, temporary_covariance);
    alg_kalman_internal_multiply_right_transpose(temporary_covariance, config->state_dimension,
                                                 config->state_dimension, state_jacobian,
                                                 config->state_dimension, predicted_covariance);

    for (index = 0U; index < state_square; ++index)
        predicted_covariance[index] += config->process_noise[index];

    alg_kalman_internal_symmetrize(predicted_covariance, config->state_dimension);

    // ---- 5. 检查结果有效性 ----
    if (!alg_kalman_internal_is_finite_array(predicted_covariance, state_square))
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;

    // ---- 6. 提交更新 ----
    alg_kalman_internal_copy(config->state, predicted_state, config->state_dimension);
    alg_kalman_internal_copy(config->covariance, predicted_covariance, state_square);

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 扩展卡尔曼校正步骤
 * @param me 滤波器对象
 * @param measurement 测量值（m×1）
 * @return 执行状态
 * @note z = h(x)，H = ∂h/∂x
 *       使用内部 alg_kalman_internal_correct 执行校正
 */
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
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;

    config = &me->config;
    if (!alg_kalman_internal_is_finite_array(measurement, config->measurement_dimension))
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;

    cross_size = config->state_dimension * config->measurement_dimension;
    predicted_measurement = config->workspace;
    measurement_jacobian = predicted_measurement + config->measurement_dimension;
    correction_workspace = measurement_jacobian + cross_size;
    correction_workspace_size = config->workspace_size - config->measurement_dimension - cross_size;

    // ---- 1. 调用测量函数计算 h(x) ----
    status = config->measurement_function(config->state, config->state_dimension,
                                          config->measurement_dimension, predicted_measurement,
                                          config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    // ---- 2. 调用测量雅可比函数计算 H = ∂h/∂x ----
    status = config->measurement_jacobian_function(config->state, config->state_dimension,
                                                   config->measurement_dimension,
                                                   measurement_jacobian, config->user_context);
    if (status != ALG_KALMAN_STATUS_OK)
        return status;

    // ---- 3. 检查模型输出有效性 ----
    if (!alg_kalman_internal_is_finite_array(predicted_measurement,
                                             config->measurement_dimension) ||
        !alg_kalman_internal_is_finite_array(measurement_jacobian, cross_size))
        return ALG_KALMAN_STATUS_MODEL_ERROR;

    // ---- 4. 执行校正 ----
    return alg_kalman_internal_correct(config->state, config->covariance, config->state_dimension,
                                       measurement_jacobian, config->measurement_noise, measurement,
                                       predicted_measurement, config->measurement_dimension,
                                       correction_workspace, correction_workspace_size);
}

/**
 * @brief 获取当前状态（只读）
 * @param me 滤波器对象
 * @return 状态指针，未初始化则返回 NULL
 */
const float *alg_kalman_extended_get_state(const alg_kalman_extended_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.state : NULL;
}

/**
 * @brief 获取当前协方差矩阵（只读）
 * @param me 滤波器对象
 * @return 协方差指针，未初始化则返回 NULL
 */
const float *alg_kalman_extended_get_covariance(const alg_kalman_extended_t *me)
{
    return ((me != NULL) && me->is_initialized) ? me->config.covariance : NULL;
}