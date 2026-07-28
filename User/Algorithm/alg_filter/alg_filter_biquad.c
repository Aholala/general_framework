/**
 * @file alg_filter_biquad.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 二阶 Biquad 滤波器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持低通、高通、带通、陷波四种类型。
 *       使用直接 II 型转置结构，数值稳定性好。
 *       内置系数生成基于标准 Biquad 设计公式。
 */

#include "alg_filter.h"

#include <math.h>   // cosf, sinf, isfinite
#include <stddef.h> // NULL

/** @brief π 常量 */
#define ALG_FILTER_PI_F (3.14159265358979323846F)

/**
 * @brief 检查 5 个浮点数是否都有限
 * @return true=全有限
 */
static bool alg_filter_biquad_are_finite(float value_0, float value_1, float value_2, float value_3,
                                         float value_4)
{
    return isfinite(value_0) && isfinite(value_1) && isfinite(value_2) && isfinite(value_3) &&
           isfinite(value_4);
}

/**
 * @brief 初始化 Biquad 滤波器（自动生成系数）
 * @param me 滤波器对象
 * @param type 滤波器类型（低通/高通/带通/陷波）
 * @param sample_frequency_hz 采样频率（Hz）
 * @param center_frequency_hz 中心/截止频率（Hz）
 * @param quality_factor 品质因数（Q）
 * @return 执行状态
 * @note 约束：0 < center_frequency_hz < sample_frequency_hz / 2
 *       quality_factor > 0
 *       二阶 Butterworth 低通/高通常用 Q = 0.70710678F
 */
alg_filter_status_t alg_filter_biquad_init(alg_filter_biquad_t *me, alg_filter_biquad_type_t type,
                                           float sample_frequency_hz, float center_frequency_hz,
                                           float quality_factor)
{
    float angular_frequency;
    float cosine;
    float alpha;
    float a0;
    float a1;
    float a2;
    float b0;
    float b1;
    float b2;

    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;

    // ---- 参数校验 ----
    if (!isfinite(sample_frequency_hz) || !isfinite(center_frequency_hz) ||
        !isfinite(quality_factor) || (sample_frequency_hz <= 0.0F) ||
        (center_frequency_hz <= 0.0F) || (center_frequency_hz >= (0.5F * sample_frequency_hz)) ||
        (quality_factor <= 0.0F) || (type > ALG_FILTER_BIQUAD_NOTCH))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // ---- 计算 Biquad 系数 ----
    // 预畸变角频率：omega = 2*pi*fc/fs
    angular_frequency = 2.0F * ALG_FILTER_PI_F * center_frequency_hz / sample_frequency_hz;
    cosine = cosf(angular_frequency);
    alpha = sinf(angular_frequency) / (2.0F * quality_factor);

    // 分母系数（未归一化）
    a0 = 1.0F + alpha;
    a1 = -2.0F * cosine;
    a2 = 1.0F - alpha;

    // 分子系数（根据类型计算）
    switch (type)
    {
    case ALG_FILTER_BIQUAD_LOW_PASS:
        b0 = 0.5F * (1.0F - cosine);
        b1 = 1.0F - cosine;
        b2 = b0;
        break;

    case ALG_FILTER_BIQUAD_HIGH_PASS:
        b0 = 0.5F * (1.0F + cosine);
        b1 = -(1.0F + cosine);
        b2 = b0;
        break;

    case ALG_FILTER_BIQUAD_BAND_PASS:
        b0 = alpha;
        b1 = 0.0F;
        b2 = -alpha;
        break;

    case ALG_FILTER_BIQUAD_NOTCH:
        b0 = 1.0F;
        b1 = -2.0F * cosine;
        b2 = 1.0F;
        break;

    default:
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    // 归一化：分母首项 a0 = 1
    return alg_filter_biquad_set_coefficients(me, b0 / a0, b1 / a0, b2 / a0, a1 / a0, a2 / a0);
}

/**
 * @brief 手动设置 Biquad 系数（已归一化）
 * @param me 滤波器对象
 * @param b0 分子系数 b0
 * @param b1 分子系数 b1
 * @param b2 分子系数 b2
 * @param a1 分母系数 a1（注意：分母为 1 + a1*z^-1 + a2*z^-2）
 * @param a2 分母系数 a2
 * @return 执行状态
 * @note 适用于离线设计工具预计算系数后直接加载
 *       所有系数必须为有限数
 */
alg_filter_status_t alg_filter_biquad_set_coefficients(alg_filter_biquad_t *me, float b0, float b1,
                                                       float b2, float a1, float a2)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!alg_filter_biquad_are_finite(b0, b1, b2, a1, a2))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->b0 = b0;
    me->b1 = b1;
    me->b2 = b2;
    me->a1 = a1;
    me->a2 = a2;
    me->state_1 = 0.0F;
    me->state_2 = 0.0F;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 重置 Biquad 滤波器状态（清零内部状态）
 * @param me 滤波器对象
 * @return 执行状态
 */
alg_filter_status_t alg_filter_biquad_reset(alg_filter_biquad_t *me)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;

    me->state_1 = 0.0F;
    me->state_2 = 0.0F;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新 Biquad 滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param output 输出滤波值
 * @return 执行状态
 * @note 使用直接 II 型转置结构：
 *       y = b0*x + state_1
 *       state_1 = b1*x - a1*y + state_2
 *       state_2 = b2*x - a2*y
 */
alg_filter_status_t alg_filter_biquad_update(alg_filter_biquad_t *me, float input, float *output)
{
    float current_output;
    float next_state_1;
    float next_state_2;

    if ((me == NULL) || (output == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(input))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // 直接 II 型转置结构
    current_output = (me->b0 * input) + me->state_1;
    next_state_1 = (me->b1 * input) - (me->a1 * current_output) + me->state_2;
    next_state_2 = (me->b2 * input) - (me->a2 * current_output);

    // 检查数值稳定性
    if (!alg_filter_biquad_are_finite(current_output, next_state_1, next_state_2, me->a1, me->a2))
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;

    me->state_1 = next_state_1;
    me->state_2 = next_state_2;
    *output = current_output;
    return ALG_FILTER_STATUS_OK;
}