/**
 * @file alg_filter_basic.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 基础滤波器实现（低通、高通、指数平均）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含一阶低通 RC 滤波器、一阶高通 RC 滤波器和指数移动平均滤波器。
 *       所有滤波器都是无状态依赖的，支持任意多个独立实例。
 */

#include "alg_filter.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/** @brief 2π 常量 */
#define ALG_FILTER_TWO_PI_F (6.28318530717958647692F)

/**
 * @brief 检查值是否为正有限数
 * @param value 要检查的值
 * @return true=正有限数
 */
static bool alg_filter_basic_is_positive_finite(float value)
{
    return isfinite(value) && (value > 0.0F);
}

/* ======================== 一阶低通滤波器 ======================== */

/**
 * @brief 初始化一阶低通滤波器
 * @param me 滤波器对象
 * @param cutoff_frequency_hz 截止频率（Hz），必须 > 0
 * @return 执行状态
 */
alg_filter_status_t alg_filter_low_pass_init(alg_filter_low_pass_t *me, float cutoff_frequency_hz)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}

    // 先标记为未初始化，防止中途失败留下半成品
    me->is_initialized = false;
    me->has_previous_sample = false;
    me->output = 0.0F;

    // 校验截止频率
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 修改低通滤波器截止频率
 * @param me 滤波器对象
 * @param cutoff_frequency_hz 新的截止频率（Hz）
 * @return 执行状态
 */
alg_filter_status_t alg_filter_low_pass_set_cutoff(alg_filter_low_pass_t *me,
                                                   float cutoff_frequency_hz)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 重置低通滤波器状态
 * @param me 滤波器对象
 * @param initial_output 初始输出值
 * @return 执行状态
 */
alg_filter_status_t alg_filter_low_pass_reset(alg_filter_low_pass_t *me, float initial_output)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(initial_output)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->output = initial_output;
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新低通滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param delta_time_s 时间步长（秒），必须 > 0
 * @param output 输出滤波值
 * @return 执行状态
 * @note 首次更新时直接输出输入值（无历史数据）
 *       后续使用一阶 RC 滤波：y = y + (dt/(tau+dt)) * (x-y)
 *       tau = 1/(2*pi*fc)
 */
alg_filter_status_t alg_filter_low_pass_update(alg_filter_low_pass_t *me, float input,
                                               float delta_time_s, float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((me == NULL) || (output == NULL)) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(input) || !alg_filter_basic_is_positive_finite(delta_time_s)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    // 首次采样：直接赋值
    if (!me->has_previous_sample)
    {
        me->output = input;
        me->has_previous_sample = true;
    }
    else
    {
        // 时间常数 tau = 1 / (2*pi*fc)
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * me->cutoff_frequency_hz);
        // 平滑因子 = dt / (tau + dt)
        smoothing_factor = delta_time_s / (time_constant_s + delta_time_s);
        // 一阶低通：y = y + alpha * (x - y)
        me->output += smoothing_factor * (input - me->output);
    }

    if (!isfinite(me->output)) {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}

/* ======================== 一阶高通滤波器 ======================== */

/**
 * @brief 初始化一阶高通滤波器
 * @param me 滤波器对象
 * @param cutoff_frequency_hz 截止频率（Hz），必须 > 0
 * @return 执行状态
 */
alg_filter_status_t alg_filter_high_pass_init(alg_filter_high_pass_t *me, float cutoff_frequency_hz)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;
    me->has_previous_sample = false;
    me->previous_input = 0.0F;
    me->output = 0.0F;

    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 修改高通滤波器截止频率
 */
alg_filter_status_t alg_filter_high_pass_set_cutoff(alg_filter_high_pass_t *me,
                                                    float cutoff_frequency_hz)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!alg_filter_basic_is_positive_finite(cutoff_frequency_hz)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->cutoff_frequency_hz = cutoff_frequency_hz;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 重置高通滤波器状态
 * @param me 滤波器对象
 * @param initial_input 初始输入值（用于初始化 previous_input）
 * @return 执行状态
 */
alg_filter_status_t alg_filter_high_pass_reset(alg_filter_high_pass_t *me, float initial_input)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(initial_input)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->previous_input = initial_input;
    me->output = 0.0F;
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新高通滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param delta_time_s 时间步长（秒），必须 > 0
 * @param output 输出滤波值
 * @return 执行状态
 * @note 首次更新时输出 0（高通滤波器需要历史数据）
 *       后续使用：y = (tau/(tau+dt)) * (y_prev + x - x_prev)
 *       tau = 1/(2*pi*fc)
 */
alg_filter_status_t alg_filter_high_pass_update(alg_filter_high_pass_t *me, float input,
                                                float delta_time_s, float *output)
{
    float time_constant_s;
    float smoothing_factor;

    if ((me == NULL) || (output == NULL)) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(input) || !alg_filter_basic_is_positive_finite(delta_time_s)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    // 首次采样：输出 0，记录输入
    if (!me->has_previous_sample)
    {
        me->previous_input = input;
        me->output = 0.0F;
        me->has_previous_sample = true;
    }
    else
    {
        time_constant_s = 1.0F / (ALG_FILTER_TWO_PI_F * me->cutoff_frequency_hz);
        // 高通平滑因子 = tau / (tau + dt)
        smoothing_factor = time_constant_s / (time_constant_s + delta_time_s);
        // 一阶高通：y = beta * (y_prev + x - x_prev)
        me->output = smoothing_factor * (me->output + input - me->previous_input);
        me->previous_input = input;
    }

    if (!isfinite(me->output)) {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}

/* ======================== 指数移动平均滤波器 ======================== */

/**
 * @brief 初始化指数移动平均滤波器
 * @param me 滤波器对象
 * @param smoothing_factor 平滑因子（0~1），越大响应越快
 * @return 执行状态
 * @note 平滑因子 alpha ∈ (0, 1]，alpha=1 时无滤波（直接输出输入）
 */
alg_filter_status_t alg_filter_exponential_init(alg_filter_exponential_t *me,
                                                float smoothing_factor)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;
    me->has_previous_sample = false;
    me->output = 0.0F;

    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) || (smoothing_factor > 1.0F)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->smoothing_factor = smoothing_factor;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 修改指数平均滤波器的平滑因子
 */
alg_filter_status_t alg_filter_exponential_set_factor(alg_filter_exponential_t *me,
                                                      float smoothing_factor)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(smoothing_factor) || (smoothing_factor <= 0.0F) || (smoothing_factor > 1.0F)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->smoothing_factor = smoothing_factor;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 重置指数平均滤波器
 * @param me 滤波器对象
 * @param initial_output 初始输出值
 * @return 执行状态
 */
alg_filter_status_t alg_filter_exponential_reset(alg_filter_exponential_t *me, float initial_output)
{
    if (me == NULL) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(initial_output)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    me->output = initial_output;
    me->has_previous_sample = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新指数平均滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param output 输出滤波值
 * @return 执行状态
 * @note 实现：y = y + alpha * (x - y)
 *       首次更新时直接输出输入值
 */
alg_filter_status_t alg_filter_exponential_update(alg_filter_exponential_t *me, float input,
                                                  float *output)
{
    if ((me == NULL) || (output == NULL)) {
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(input)) {
        return ALG_FILTER_STATUS_OUT_OF_RANGE;
}

    // 首次采样：直接赋值
    if (!me->has_previous_sample)
    {
        me->output = input;
        me->has_previous_sample = true;
    }
    else
    {
        // y = y + alpha * (x - y)
        me->output += me->smoothing_factor * (input - me->output);
    }

    if (!isfinite(me->output)) {
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}