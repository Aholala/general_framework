/**
 * @file alg_pid_core.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 位置式 PID 控制器核心实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含配置初始化、控制器初始化、重置、输出跟踪、更新等功能。
 *       支持二自由度、微分滤波、多种抗积分饱和模式。
 */

#include "alg_pid_internal.h"
#include <float.h>
#include <math.h>
#include <stddef.h>

#define ALG_PID_TWO_PI_F (6.28318530717958647692F)

/**
 * @brief 检查浮点数是否有限
 */
bool alg_pid_internal_is_finite(float value)
{
    return isfinite(value);
}

/**
 * @brief 限幅函数
 */
float alg_pid_internal_clamp(float value, float minimum, float maximum)
{
    if (value > maximum)
        return maximum;
    if (value < minimum)
        return minimum;
    return value;
}

/**
 * @brief 应用死区
 */
float alg_pid_internal_apply_deadband(float value, float deadband)
{
    return (fabsf(value) <= deadband) ? 0.0F : value;
}

/**
 * @brief 验证位置式 PID 配置
 */
alg_pid_status_t alg_pid_internal_validate_config(const alg_pid_config_t *config)
{
    if (config == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;

    if (!isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->derivative_gain) || !isfinite(config->setpoint_weight) ||
        !isfinite(config->derivative_setpoint_weight) ||
        !isfinite(config->velocity_feedforward_gain) ||
        !isfinite(config->acceleration_feedforward_gain) ||
        !isfinite(config->derivative_filter_cutoff_hz) || !isfinite(config->error_deadband) ||
        !isfinite(config->integral_separation_threshold) || !isfinite(config->integral_min) ||
        !isfinite(config->integral_max) || !isfinite(config->output_min) ||
        !isfinite(config->output_max) || !isfinite(config->back_calculation_gain))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    if ((config->setpoint_weight < 0.0F) || (config->setpoint_weight > 1.0F) ||
        (config->derivative_setpoint_weight < 0.0F) ||
        (config->derivative_setpoint_weight > 1.0F) ||
        (config->derivative_filter_cutoff_hz < 0.0F) || (config->error_deadband < 0.0F) ||
        (config->integral_separation_threshold < 0.0F) ||
        (config->integral_min > config->integral_max) ||
        (config->output_min >= config->output_max) || (config->back_calculation_gain < 0.0F) ||
        (config->anti_windup_mode > ALG_PID_ANTI_WINDUP_BACK_CALCULATION) ||
        (config->derivative_mode > ALG_PID_DERIVATIVE_ON_MEASUREMENT))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    return ALG_PID_STATUS_OK;
}

/**
 * @brief 初始化配置为默认值
 */
alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config)
{
    if (config == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;

    *config = (alg_pid_config_t){.proportional_gain = 0.0F,
                                 .integral_gain = 0.0F,
                                 .derivative_gain = 0.0F,
                                 .setpoint_weight = 1.0F,
                                 .derivative_setpoint_weight = 0.0F,
                                 .velocity_feedforward_gain = 0.0F,
                                 .acceleration_feedforward_gain = 0.0F,
                                 .derivative_filter_cutoff_hz = 0.0F,
                                 .error_deadband = 0.0F,
                                 .integral_separation_threshold = 0.0F,
                                 .integral_min = -FLT_MAX,
                                 .integral_max = FLT_MAX,
                                 .output_min = -FLT_MAX,
                                 .output_max = FLT_MAX,
                                 .back_calculation_gain = 0.0F,
                                 .anti_windup_mode = ALG_PID_ANTI_WINDUP_CLAMPING,
                                 .derivative_mode = ALG_PID_DERIVATIVE_ON_MEASUREMENT};
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 初始化位置式 PID 控制器
 */
alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config)
{
    alg_pid_status_t status;

    if (me == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;
    status = alg_pid_internal_validate_config(config);
    if (status != ALG_PID_STATUS_OK)
        return status;

    me->config = *config;
    me->terms = (alg_pid_terms_t){0};
    me->previous_error = 0.0F;
    me->previous_setpoint = 0.0F;
    me->previous_measurement = 0.0F;
    me->filtered_derivative = 0.0F;
    me->has_previous_sample = false;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

alg_pid_status_t alg_pid_position_init(alg_pid_position_t *me, const alg_pid_config_t *config)
{
    return alg_pid_init(me, config);
}

alg_pid_status_t alg_pid_velocity_init(alg_pid_velocity_t *me, const alg_pid_config_t *config)
{
    return alg_pid_init(me, config);
}

/**
 * @brief 运行时更新配置
 */
alg_pid_status_t alg_pid_set_config(alg_pid_t *me, const alg_pid_config_t *config)
{
    alg_pid_status_t status;

    if (me == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;

    status = alg_pid_internal_validate_config(config);
    if (status != ALG_PID_STATUS_OK)
        return status;

    me->config = *config;
    me->terms.integral =
        alg_pid_internal_clamp(me->terms.integral, config->integral_min, config->integral_max);
    me->terms.output =
        alg_pid_internal_clamp(me->terms.output, config->output_min, config->output_max);
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置控制器
 */
alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output)
{
    if (me == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;
    if (!isfinite(measurement) || !isfinite(initial_output))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    me->terms = (alg_pid_terms_t){0};
    me->terms.integral =
        alg_pid_internal_clamp(initial_output, me->config.integral_min, me->config.integral_max);
    me->terms.unsaturated_output = me->terms.integral;
    me->terms.output =
        alg_pid_internal_clamp(initial_output, me->config.output_min, me->config.output_max);
    me->previous_error = 0.0F;
    me->previous_setpoint = measurement;
    me->previous_measurement = measurement;
    me->filtered_derivative = 0.0F;
    me->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 输出跟踪（无扰切换）
 */
alg_pid_status_t alg_pid_track_output(alg_pid_t *me, float setpoint, float measurement,
                                      float feedforward, float tracked_output)
{
    float error;
    float proportional;
    float integral;

    if (me == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;
    if (!isfinite(setpoint) || !isfinite(measurement) || !isfinite(feedforward) ||
        !isfinite(tracked_output))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    error = alg_pid_internal_apply_deadband(setpoint - measurement, me->config.error_deadband);
    proportional = (error == 0.0F) ? 0.0F
                                   : me->config.proportional_gain *
                                         ((me->config.setpoint_weight * setpoint) - measurement);
    // 反算积分项使得 P + I + FF = tracked_output
    integral = tracked_output - proportional - feedforward;
    integral = alg_pid_internal_clamp(integral, me->config.integral_min, me->config.integral_max);

    me->terms.proportional = proportional;
    me->terms.integral = integral;
    me->terms.derivative = 0.0F;
    me->terms.feedforward = feedforward;
    me->terms.unsaturated_output = proportional + integral + feedforward;
    me->terms.output = alg_pid_internal_clamp(me->terms.unsaturated_output, me->config.output_min,
                                              me->config.output_max);
    me->previous_error = error;
    me->previous_setpoint = setpoint;
    me->previous_measurement = measurement;
    me->filtered_derivative = 0.0F;
    me->has_previous_sample = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 简单更新（无前馈）
 */
alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                float delta_time_s, float *output)
{
    const alg_pid_input_t input = {.setpoint = setpoint,
                                   .measurement = measurement,
                                   .setpoint_rate_per_s = 0.0F,
                                   .setpoint_acceleration_per_s2 = 0.0F,
                                   .additional_feedforward = 0.0F,
                                   .delta_time_s = delta_time_s};
    return alg_pid_update_advanced(me, &input, output);
}

/**
 * @brief 高级更新
 */
alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input, float *output)
{
    float error;
    float control_error;
    float proportional;
    float integral_candidate;
    float derivative_signal;
    float filtered_derivative;
    float derivative;
    float feedforward;
    float unsaturated_output;
    float saturated_output;
    float time_constant_s;
    float smoothing_factor;
    bool integration_enabled;
    bool saturation_pushes_with_error;

    // ---- 参数检查 ----
    if ((me == NULL) || (input == NULL) || (output == NULL))
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->setpoint_rate_per_s) || !isfinite(input->setpoint_acceleration_per_s2) ||
        !isfinite(input->additional_feedforward) || !isfinite(input->delta_time_s) ||
        (input->delta_time_s <= 0.0F))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    // ---- 计算误差和死区 ----
    error = input->setpoint - input->measurement;
    control_error = alg_pid_internal_apply_deadband(error, me->config.error_deadband);

    // ---- 比例项（二自由度） ----
    proportional = (control_error == 0.0F)
                       ? 0.0F
                       : me->config.proportional_gain *
                             ((me->config.setpoint_weight * input->setpoint) - input->measurement);

    // ---- 积分项（含积分分离） ----
    integration_enabled = (me->config.integral_separation_threshold <= 0.0F) ||
                          (fabsf(error) <= me->config.integral_separation_threshold);
    integral_candidate = me->terms.integral;
    if (integration_enabled)
        integral_candidate += me->config.integral_gain * control_error * input->delta_time_s;
    integral_candidate = alg_pid_internal_clamp(integral_candidate, me->config.integral_min,
                                                me->config.integral_max);

    // ---- 微分项（含微分先行和滤波） ----
    derivative_signal = 0.0F;
    if (me->has_previous_sample)
    {
        if (me->config.derivative_mode == ALG_PID_DERIVATIVE_ON_MEASUREMENT)
        {
            derivative_signal =
                -(input->measurement - me->previous_measurement) / input->delta_time_s;
        }
        else
        {
            derivative_signal =
                (((me->config.derivative_setpoint_weight * input->setpoint) - input->measurement) -
                 ((me->config.derivative_setpoint_weight * me->previous_setpoint) -
                  me->previous_measurement)) /
                input->delta_time_s;
        }
    }

    // 微分一阶低通滤波
    filtered_derivative = derivative_signal;
    if (me->has_previous_sample && (me->config.derivative_filter_cutoff_hz > 0.0F))
    {
        time_constant_s = 1.0F / (ALG_PID_TWO_PI_F * me->config.derivative_filter_cutoff_hz);
        smoothing_factor = input->delta_time_s / (time_constant_s + input->delta_time_s);
        filtered_derivative = me->filtered_derivative +
                              smoothing_factor * (derivative_signal - me->filtered_derivative);
    }
    derivative = me->config.derivative_gain * filtered_derivative;

    // ---- 前馈项 ----
    feedforward = (me->config.velocity_feedforward_gain * input->setpoint_rate_per_s) +
                  (me->config.acceleration_feedforward_gain * input->setpoint_acceleration_per_s2) +
                  input->additional_feedforward;

    // ---- 计算未限幅输出 ----
    unsaturated_output = proportional + integral_candidate + derivative + feedforward;
    saturated_output =
        alg_pid_internal_clamp(unsaturated_output, me->config.output_min, me->config.output_max);

    // ---- 抗积分饱和处理 ----
    if (me->config.anti_windup_mode == ALG_PID_ANTI_WINDUP_CLAMPING)
    {
        // 条件积分：饱和且误差同向时保持积分不变
        saturation_pushes_with_error =
            ((unsaturated_output > me->config.output_max) && (control_error > 0.0F)) ||
            ((unsaturated_output < me->config.output_min) && (control_error < 0.0F));
        if (saturation_pushes_with_error)
        {
            integral_candidate = me->terms.integral;
            unsaturated_output = proportional + integral_candidate + derivative + feedforward;
            saturated_output = alg_pid_internal_clamp(unsaturated_output, me->config.output_min,
                                                      me->config.output_max);
        }
    }
    else if (me->config.anti_windup_mode == ALG_PID_ANTI_WINDUP_BACK_CALCULATION)
    {
        // 反算抗饱和：通过输出差值反馈修正积分
        integral_candidate += me->config.back_calculation_gain *
                              (saturated_output - unsaturated_output) * input->delta_time_s;
        integral_candidate = alg_pid_internal_clamp(integral_candidate, me->config.integral_min,
                                                    me->config.integral_max);
        unsaturated_output = proportional + integral_candidate + derivative + feedforward;
        saturated_output = alg_pid_internal_clamp(unsaturated_output, me->config.output_min,
                                                  me->config.output_max);
    }

    // ---- 检查数值有效性 ----
    if (!isfinite(proportional) || !isfinite(integral_candidate) || !isfinite(derivative) ||
        !isfinite(feedforward) || !isfinite(unsaturated_output) || !isfinite(saturated_output))
        return ALG_PID_STATUS_NUMERICAL_ERROR;

    // ---- 更新状态 ----
    me->terms.proportional = proportional;
    me->terms.integral = integral_candidate;
    me->terms.derivative = derivative;
    me->terms.feedforward = feedforward;
    me->terms.unsaturated_output = unsaturated_output;
    me->terms.output = saturated_output;
    me->previous_error = control_error;
    me->previous_setpoint = input->setpoint;
    me->previous_measurement = input->measurement;
    me->filtered_derivative = filtered_derivative;
    me->has_previous_sample = true;
    *output = saturated_output;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 获取各项分量
 */
const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me)
{
    return ((me != NULL) && me->is_initialized) ? &me->terms : NULL;
}