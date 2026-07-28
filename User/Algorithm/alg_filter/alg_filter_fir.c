/**
 * @file alg_filter_fir.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用 FIR 滤波器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note coefficients[0] 对应当前输入，coefficients[1] 对应前一个输入，以此类推。
 *       系数数组和状态数组长度必须等于 tap_count。
 *       使用环形缓冲区存储状态，高效实现延迟线。
 */

#include "alg_filter.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 初始化 FIR 滤波器
 * @param me 滤波器对象
 * @param coefficients FIR 系数数组（调用者提供），长度 tap_count
 * @param state_buffer 状态缓冲区（调用者提供），长度 tap_count
 * @param tap_count 抽头数，必须 > 0
 * @return 执行状态
 * @note 两个数组在滤波器生命周期内必须保持有效
 */
alg_filter_status_t alg_filter_fir_init(alg_filter_fir_t *me, const float *coefficients,
                                        float *state_buffer, size_t tap_count)
{
    size_t tap_index;

    if ((me == NULL) || (coefficients == NULL) || (state_buffer == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (tap_count == 0U)
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->is_initialized = false;

    // 校验所有系数是否为有限数
    for (tap_index = 0U; tap_index < tap_count; ++tap_index)
    {
        if (!isfinite(coefficients[tap_index]))
            return ALG_FILTER_STATUS_OUT_OF_RANGE;
    }

    me->coefficients = coefficients;
    me->state_buffer = state_buffer;
    me->tap_count = tap_count;
    me->is_initialized = true;
    return alg_filter_fir_reset(me);
}

/**
 * @brief 重置 FIR 滤波器（清空状态缓冲区）
 * @param me 滤波器对象
 * @return 执行状态
 */
alg_filter_status_t alg_filter_fir_reset(alg_filter_fir_t *me)
{
    size_t tap_index;

    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized || (me->tap_count == 0U) || (me->coefficients == NULL) ||
        (me->state_buffer == NULL))
        return ALG_FILTER_STATUS_NOT_INITIALIZED;

    for (tap_index = 0U; tap_index < me->tap_count; ++tap_index)
        me->state_buffer[tap_index] = 0.0F;
    me->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新 FIR 滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param output 输出滤波值
 * @return 执行状态
 * @note 使用环形缓冲区保存输入历史
 *       计算 y = sum(coeff[i] * state[(write_index - i) mod tap_count])
 */
alg_filter_status_t alg_filter_fir_update(alg_filter_fir_t *me, float input, float *output)
{
    size_t coefficient_index;
    size_t state_index;
    float accumulator = 0.0F;

    if ((me == NULL) || (output == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized || (me->tap_count == 0U) || (me->coefficients == NULL) ||
        (me->state_buffer == NULL) || (me->write_index >= me->tap_count))
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(input))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // 写入新输入
    me->state_buffer[me->write_index] = input;

    // 卷积计算：系数按时间顺序（coeff[0] 对应当前输入）
    state_index = me->write_index;
    for (coefficient_index = 0U; coefficient_index < me->tap_count; ++coefficient_index)
    {
        accumulator += me->coefficients[coefficient_index] * me->state_buffer[state_index];
        // 环形缓冲区向前遍历（时间上从当前向过去）
        state_index = (state_index == 0U) ? (me->tap_count - 1U) : (state_index - 1U);
    }

    // 更新写入位置
    me->write_index = (me->write_index + 1U) % me->tap_count;

    if (!isfinite(accumulator))
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;

    *output = accumulator;
    return ALG_FILTER_STATUS_OK;
}