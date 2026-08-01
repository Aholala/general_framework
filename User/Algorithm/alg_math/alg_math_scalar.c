/**
 * @file alg_math_scalar.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 标量与角度工具函数实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供限幅、插值、映射、死区、回绕、角度差及安全算术运算。
 *       所有函数检查输入有限性，错误时返回状态码。
 */

#include "alg_math.h"
#include <float.h>
#include <math.h>

/**
 * @brief 检查数组是否全为有限数
 */
bool alg_math_is_finite_array(const float *values, size_t value_count)
{
    size_t index;
    if ((values == NULL) && (value_count > 0U)) {
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
 * @brief 值限幅
 */
alg_math_status_t alg_math_clamp(float value, float lower_limit, float upper_limit, float *result)
{
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(value) || !isfinite(lower_limit) || !isfinite(upper_limit) ||
        (lower_limit > upper_limit)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    *result = fminf(fmaxf(value, lower_limit), upper_limit);
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 线性插值
 */
alg_math_status_t alg_math_lerp(float start, float end, float ratio, float *result)
{
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(start) || !isfinite(end) || !isfinite(ratio)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    *result = start + (ratio * (end - start));
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 一维线性查表插值
 */
alg_math_status_t alg_math_interpolate_linear1_d(const float *x_values, const float *y_values,
                                                 size_t point_count, float input,
                                                 bool clamp_to_table, float *output)
{
    size_t index;
    size_t lower_index;
    float ratio;

    if ((x_values == NULL) || (y_values == NULL) || (output == NULL) || (point_count < 2U))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(input))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }

    for (index = 0U; index < point_count; ++index)
    {
        if (!isfinite(x_values[index]) || !isfinite(y_values[index]))
        {
            return ALG_MATH_STATUS_OUT_OF_RANGE;
        }
        if ((index > 0U) && (x_values[index] <= x_values[index - 1U]))
        {
            return ALG_MATH_STATUS_OUT_OF_RANGE;
        }
    }

    if (clamp_to_table && (input <= x_values[0]))
    {
        *output = y_values[0];
        return ALG_MATH_STATUS_OK;
    }
    if (clamp_to_table && (input >= x_values[point_count - 1U]))
    {
        *output = y_values[point_count - 1U];
        return ALG_MATH_STATUS_OK;
    }

    if (input <= x_values[0])
    {
        lower_index = 0U;
    }
    else if (input >= x_values[point_count - 1U])
    {
        lower_index = point_count - 2U;
    }
    else
    {
        lower_index = 0U;
        while ((lower_index + 1U < point_count) &&
               (input > x_values[lower_index + 1U]))
        {
            ++lower_index;
        }
    }

    ratio = (input - x_values[lower_index]) /
            (x_values[lower_index + 1U] - x_values[lower_index]);
    *output = y_values[lower_index] +
              (ratio * (y_values[lower_index + 1U] - y_values[lower_index]));

    return isfinite(*output) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 单位正方形内的双线性插值
 */
alg_math_status_t alg_math_interpolate_bilinear(float x_ratio, float y_ratio, float value_00,
                                                float value_10, float value_01, float value_11,
                                                float *output)
{
    float lower_value;
    float upper_value;

    if (output == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(x_ratio) || !isfinite(y_ratio) ||
        !isfinite(value_00) || !isfinite(value_10) ||
        !isfinite(value_01) || !isfinite(value_11) ||
        (x_ratio < 0.0F) || (x_ratio > 1.0F) ||
        (y_ratio < 0.0F) || (y_ratio > 1.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }

    lower_value = value_00 + (x_ratio * (value_10 - value_00));
    upper_value = value_01 + (x_ratio * (value_11 - value_01));
    *output = lower_value + (y_ratio * (upper_value - lower_value));

    return isfinite(*output) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 区间映射
 */
alg_math_status_t alg_math_map_range(float value, float input_minimum, float input_maximum,
                                     float output_minimum, float output_maximum, bool clamp_output,
                                     float *result)
{
    float ratio;
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(value) || !isfinite(input_minimum) || !isfinite(input_maximum) ||
        !isfinite(output_minimum) || !isfinite(output_maximum) || (input_minimum >= input_maximum)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    // ---- 计算输入区间比例 ----
    ratio = (value - input_minimum) / (input_maximum - input_minimum);
    if (clamp_output) {
        ratio = fminf(fmaxf(ratio, 0.0F), 1.0F);
}

    *result = output_minimum + (ratio * (output_maximum - output_minimum));
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 死区处理
 */
alg_math_status_t alg_math_apply_deadband(float value, float deadband, bool rescale_output,
                                          float *result)
{
    float magnitude;
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(value) || !isfinite(deadband) || (deadband < 0.0F) || (deadband >= 1.0F)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    magnitude = fabsf(value);
    if (magnitude <= deadband)
    {
        *result = 0.0F;
    }
    else if (rescale_output)
    {
        // ---- 重新缩放至 [0,1] 区间，消除死区断层 ----
        *result = copysignf((magnitude - deadband) / (1.0F - deadband), value);
    }
    else
    {
        *result = value;
    }
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 通用区间回绕
 */
alg_math_status_t alg_math_wrap(float value, float lower_bound, float upper_bound, float *result)
{
    float width, offset;
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(value) || !isfinite(lower_bound) || !isfinite(upper_bound) ||
        (lower_bound >= upper_bound)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    width = upper_bound - lower_bound;
    offset = fmodf(value - lower_bound, width);
    if (offset < 0.0F) {
        offset += width;
}
    *result = lower_bound + offset;
    // ---- 防止浮点误差导致超出上限 ----
    if (*result >= upper_bound) {
        *result = lower_bound;
}
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 角度回绕至 [-π, π)
 */
alg_math_status_t alg_math_wrap_angle_pi(float angle_rad, float *result_rad)
{
    return alg_math_wrap(angle_rad, -ALG_MATH_PI_F, ALG_MATH_PI_F, result_rad);
}

/**
 * @brief 最短角度差
 */
alg_math_status_t alg_math_angle_difference(float target_rad, float current_rad,
                                            float *difference_rad)
{
    if (!isfinite(target_rad) || !isfinite(current_rad)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    return alg_math_wrap_angle_pi(target_rad - current_rad, difference_rad);
}

/**
 * @brief 角度转弧度
 */
float alg_math_degrees_to_radians(float angle_deg)
{
    return angle_deg * ALG_MATH_DEG_TO_RAD_F;
}

/**
 * @brief 弧度转角度
 */
float alg_math_radians_to_degrees(float angle_rad)
{
    return angle_rad * ALG_MATH_RAD_TO_DEG_F;
}

/**
 * @brief 安全平方根
 */
alg_math_status_t alg_math_safe_sqrt(float value, float *result)
{
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(value) || (value < 0.0F)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    *result = sqrtf(value);
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 安全除法
 */
alg_math_status_t alg_math_safe_divide(float numerator, float denominator,
                                       float minimum_denominator, float *result)
{
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(numerator) || !isfinite(denominator) || !isfinite(minimum_denominator) ||
        (minimum_denominator <= 0.0F)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    if (fabsf(denominator) < minimum_denominator) {
        return ALG_MATH_STATUS_SINGULAR;
}
    *result = numerator / denominator;
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}
