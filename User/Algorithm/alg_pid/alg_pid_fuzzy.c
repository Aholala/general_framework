/**
 * @file alg_pid_fuzzy.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 模糊自适应 PID 控制器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 根据归一化误差和误差变化率查二维规则表，对 Kp、Ki、Kd 进行在线调整。
 *       规则表采用双线性插值，轴点数可自定义。
 */

#include "alg_pid_internal.h"
#include <math.h>
#include <stddef.h>

/**
 * @brief 检查规则表是否全为有限数
 */
static bool alg_pid_fuzzy_is_finite_table(const float *table, size_t element_count)
{
    size_t i;
    if (table == NULL)
        return false;
    for (i = 0U; i < element_count; ++i)
        if (!isfinite(table[i]))
            return false;
    return true;
}

/**
 * @brief 双线性插值查询规则表
 */
static float alg_pid_fuzzy_interpolate_table(const float *table, size_t axis_point_count,
                                             float normalized_error, float normalized_error_rate)
{
    const float max_idx = (float)(axis_point_count - 1U);
    float ec = 0.5F * (normalized_error + 1.0F) * max_idx;
    float rc = 0.5F * (normalized_error_rate + 1.0F) * max_idx;
    size_t e_low = (size_t)floorf(ec);
    size_t r_low = (size_t)floorf(rc);
    size_t e_high, r_high;
    float e_frac, r_frac;
    float lower, upper;

    // 边界修正
    if (e_low >= (axis_point_count - 1U))
        e_low = axis_point_count - 1U;
    if (r_low >= (axis_point_count - 1U))
        r_low = axis_point_count - 1U;
    e_high = (e_low + 1U < axis_point_count) ? e_low + 1U : e_low;
    r_high = (r_low + 1U < axis_point_count) ? r_low + 1U : r_low;
    e_frac = ec - (float)e_low;
    r_frac = rc - (float)r_low;

    // 双线性插值：先沿 rate 方向，再沿 error 方向
    lower = table[(e_low * axis_point_count) + r_low] +
            r_frac * (table[(e_low * axis_point_count) + r_high] -
                      table[(e_low * axis_point_count) + r_low]);
    upper = table[(e_high * axis_point_count) + r_low] +
            r_frac * (table[(e_high * axis_point_count) + r_high] -
                      table[(e_high * axis_point_count) + r_low]);
    return lower + e_frac * (upper - lower);
}

/**
 * @brief 初始化模糊自适应 PID
 */
alg_pid_status_t alg_pid_fuzzy_init(alg_pid_fuzzy_t *me, const alg_pid_fuzzy_config_t *config)
{
    size_t table_size;
    alg_pid_status_t status;

    if ((me == NULL) || (config == NULL))
        return ALG_PID_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;
    if ((config->axis_point_count < 2U) || !isfinite(config->error_normalization) ||
        !isfinite(config->error_rate_normalization) || (config->error_normalization <= 0.0F) ||
        (config->error_rate_normalization <= 0.0F))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    if (config->axis_point_count > (SIZE_MAX / config->axis_point_count))
        return ALG_PID_STATUS_OUT_OF_RANGE;
    table_size = config->axis_point_count * config->axis_point_count;
    if (!alg_pid_fuzzy_is_finite_table(config->proportional_adjustment_table, table_size) ||
        !alg_pid_fuzzy_is_finite_table(config->integral_adjustment_table, table_size) ||
        !alg_pid_fuzzy_is_finite_table(config->derivative_adjustment_table, table_size))
        return ALG_PID_STATUS_INVALID_ARGUMENT;

    status = alg_pid_init(&me->controller, &config->base_config);
    if (status != ALG_PID_STATUS_OK)
        return status;

    me->config = *config;
    me->previous_error = 0.0F;
    me->has_previous_sample = false;
    me->is_initialized = true;
    return ALG_PID_STATUS_OK;
}

/**
 * @brief 重置模糊自适应 PID
 */
alg_pid_status_t alg_pid_fuzzy_reset(alg_pid_fuzzy_t *me, float measurement, float initial_output)
{
    alg_pid_status_t status;

    if (me == NULL)
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;

    status = alg_pid_reset(&me->controller, measurement, initial_output);
    if (status == ALG_PID_STATUS_OK)
    {
        me->previous_error = 0.0F;
        me->has_previous_sample = false;
    }
    return status;
}

/**
 * @brief 模糊自适应 PID 更新
 */
alg_pid_status_t alg_pid_fuzzy_update(alg_pid_fuzzy_t *me, const alg_pid_input_t *input,
                                      float *output)
{
    float error, error_rate;
    float norm_error, norm_error_rate;
    alg_pid_status_t status;

    // ---- 参数检查 ----
    if ((me == NULL) || (input == NULL) || (output == NULL))
        return ALG_PID_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_PID_STATUS_NOT_INITIALIZED;
    if (!isfinite(input->setpoint) || !isfinite(input->measurement) ||
        !isfinite(input->delta_time_s) || (input->delta_time_s <= 0.0F))
        return ALG_PID_STATUS_OUT_OF_RANGE;

    // ---- 计算误差和误差变化率 ----
    error = input->setpoint - input->measurement;
    error_rate =
        me->has_previous_sample ? (error - me->previous_error) / input->delta_time_s : 0.0F;
    norm_error = alg_pid_internal_clamp(error / me->config.error_normalization, -1.0F, 1.0F);
    norm_error_rate =
        alg_pid_internal_clamp(error_rate / me->config.error_rate_normalization, -1.0F, 1.0F);

    // ---- 查表调整增益 ----
    me->controller.config.proportional_gain =
        me->config.base_config.proportional_gain +
        alg_pid_fuzzy_interpolate_table(me->config.proportional_adjustment_table,
                                        me->config.axis_point_count, norm_error, norm_error_rate);
    me->controller.config.integral_gain =
        me->config.base_config.integral_gain +
        alg_pid_fuzzy_interpolate_table(me->config.integral_adjustment_table,
                                        me->config.axis_point_count, norm_error, norm_error_rate);
    me->controller.config.derivative_gain =
        me->config.base_config.derivative_gain +
        alg_pid_fuzzy_interpolate_table(me->config.derivative_adjustment_table,
                                        me->config.axis_point_count, norm_error, norm_error_rate);

    // ---- 执行 PID 更新 ----
    status = alg_pid_update_advanced(&me->controller, input, output);
    if (status == ALG_PID_STATUS_OK)
    {
        me->previous_error = error;
        me->has_previous_sample = true;
    }
    return status;
}