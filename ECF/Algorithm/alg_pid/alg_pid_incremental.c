/**
 * @file alg_pid_incremental.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 增量式 PID 控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 计算每周期输出变化量，适用于执行器需要增量累加的场景。
 *       支持增量限幅、总输出限幅、微分低通和死区。
 */

#include "alg_pid_internal.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#define ALG_PID_TWO_PI_F (6.28318530717958647692F)

/**
 * @brief 验证增量式 PID 配置
 */
static alg_pid_status_t
alg_pid_incremental_validate_config(const alg_pid_incremental_config_t *config)
{
    if (config == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) || !isfinite(config->derivative_filter_cutoff_hz) ||
        !isfinite(config->error_deadband) || !isfinite(config->delta_output_min) ||
        !isfinite(config->delta_output_max) || !isfinite(config->output_min) ||
        !isfinite(config->output_max)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}
    if ((config->derivative_filter_cutoff_hz < 0.0F) || (config->error_deadband < 0.0F) ||
        (config->delta_output_min > config->delta_output_max) ||
        (config->output_min >= config->output_max)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 初始化增量式 PID 配置为默认值
 */
alg_pid_status_t alg_pid_incremental_config_init(alg_pid_incremental_config_t *config)
{
    if (config == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}

    *config = (alg_pid_incremental_config_t){.proportional_gain = 0.0F,
                                             .integral_gain = 0.0F,
                                             .derivative_gain = 0.0F,
                                             .derivative_filter_cutoff_hz = 0.0F,
                                             .error_deadband = 0.0F,
                                             .delta_output_min = -FLT_MAX,
                                             .delta_output_max = FLT_MAX,
                                             .output_min = -FLT_MAX,
                                             .output_max = FLT_MAX};
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 初始化增量式 PID 控制器
 */
alg_pid_status_t alg_pid_incremental_init(alg_pid_incremental_t *me,
                                          const alg_pid_incremental_config_t *config)
{
    alg_pid_status_t status;

    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;
    status = alg_pid_incremental_validate_config(config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    me->config = *config;
    me->terms = (alg_pid_terms_t){0};
    me->previous_error = 0.0F;
    me->second_previous_error = 0.0F;
    me->filtered_derivative_delta = 0.0F;
    me->has_previous_sample = false;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 运行时更新增量式 PID 配置
 */
alg_pid_status_t alg_pid_incremental_set_config(alg_pid_incremental_t *me,
                                                const alg_pid_incremental_config_t *config)
{
    alg_pid_status_t status;

    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    status = alg_pid_incremental_validate_config(config);
    if (status != ALG_PID_STATUS_OK) {
        return status;
}

    me->config = *config;
    me->terms.output =
        alg_pid_internal_clamp(me->terms.output, config->output_min, config->output_max);
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置增量式 PID
 */
alg_pid_status_t alg_pid_incremental_reset(alg_pid_incremental_t *me, float initial_output)
{
    if (me == NULL) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(initial_output)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    me->terms = (alg_pid_terms_t){0};
    me->terms.output =
        alg_pid_internal_clamp(initial_output, me->config.output_min, me->config.output_max);
    me->terms.unsaturated_output = me->terms.output;
    me->previous_error = 0.0F;
    me->second_previous_error = 0.0F;
    me->filtered_derivative_delta = 0.0F;
    me->has_previous_sample = false;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 增量式 PID 更新
 */
alg_pid_status_t alg_pid_incremental_update(alg_pid_incremental_t *me, float setpoint,
                                            float measurement, float feedforward_delta,
                                            float delta_time_s, float *output)
{
    float error;
    float proportional_delta;
    float integral_delta;
    float derivative_delta;
    float filtered_derivative_delta;
    float total_delta;
    float unsaturated_output;
    float saturated_output;
    float time_constant_s;
    float smoothing_factor;

    // ---- 参数检查 ----
    if ((me == NULL) || (output == NULL)) {
        return ALG_PID_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_PID_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(feedforward_delta) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F)) {
        return ALG_PID_STATUS_OUT_OF_RANGE;
}

    // ---- 计算误差及死区 ----
    error = alg_pid_internal_apply_deadband(setpoint - measurement, me->config.error_deadband);

    // ---- 比例增量：Kp * (e(k) - e(k-1)) ----
    proportional_delta = me->config.proportional_gain * (error - me->previous_error);

    // ---- 积分增量：Ki * e(k) * dt ----
    integral_delta = me->config.integral_gain * error * delta_time_s;

    // ---- 微分增量：Kd * (e(k) - 2e(k-1) + e(k-2)) / dt ----
    derivative_delta = 0.0F;
    if (me->has_previous_sample)
    {
        derivative_delta = me->config.derivative_gain *
                           (error - (2.0F * me->previous_error) + me->second_previous_error) /
                           delta_time_s;
    }

    // ---- 微分增量低通滤波 ----
    filtered_derivative_delta = derivative_delta;
    if (me->has_previous_sample && (me->config.derivative_filter_cutoff_hz > 0.0F))
    {
        time_constant_s = 1.0F / (ALG_PID_TWO_PI_F * me->config.derivative_filter_cutoff_hz);
        smoothing_factor = delta_time_s / (time_constant_s + delta_time_s);
        filtered_derivative_delta =
            me->filtered_derivative_delta +
            smoothing_factor * (derivative_delta - me->filtered_derivative_delta);
    }

    // ---- 总增量 = 比例 + 积分 + 微分 + 前馈增量 ----
    total_delta =
        proportional_delta + integral_delta + filtered_derivative_delta + feedforward_delta;
    total_delta = alg_pid_internal_clamp(total_delta, me->config.delta_output_min,
                                         me->config.delta_output_max);

    // ---- 更新累积输出 ----
    unsaturated_output = me->terms.output + total_delta;
    saturated_output =
        alg_pid_internal_clamp(unsaturated_output, me->config.output_min, me->config.output_max);

    // ---- 数值检查 ----
    if (!isfinite(total_delta) || !isfinite(saturated_output)) {
        return ALG_PID_STATUS_NUMERICAL_ERROR;
}

    // ---- 更新状态 ----
    me->terms.proportional = proportional_delta;
    me->terms.integral = integral_delta;
    me->terms.derivative = filtered_derivative_delta;
    me->terms.feedforward = feedforward_delta;
    me->terms.unsaturated_output = unsaturated_output;
    me->terms.output = saturated_output;
    me->second_previous_error = me->has_previous_sample ? me->previous_error : error;
    me->previous_error = error;
    me->filtered_derivative_delta = filtered_derivative_delta;
    me->has_previous_sample = true;
    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 获取增量式 PID 各项增量
 */
const alg_pid_terms_t *alg_pid_incremental_get_terms(const alg_pid_incremental_t *me)
{
    return ((me != NULL) && me->is_initialized) ? &me->terms : NULL;
}