/**
 * @file alg_kalman_scalar.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 标量卡尔曼滤波器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 单变量卡尔曼滤波器，适用于单个传感器量的低成本递推估计。
 *       模型：x = x + state_delta（预测），z = x + v（观测）
 *       公式：预测 x = x，P = P + Q
 *       更新：K = P/(P+R)，x = x + K*(z-x)，P = (1-K)*P
 */

#include "alg_kalman.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 检查值是否为非负有限数
 * @param value 要检查的值
 * @return true=非负有限数
 */
static bool alg_kalman_scalar_is_nonnegative_finite(float value)
{
    return isfinite(value) && (value >= 0.0F);
}

/**
 * @brief 初始化标量卡尔曼滤波器
 * @param me 滤波器对象
 * @param process_noise 过程噪声方差 Q（>= 0）
 * @param measurement_noise 测量噪声方差 R（> 0）
 * @param initial_estimate 初始估计值
 * @param initial_covariance 初始协方差 P（>= 0）
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_scalar_init(alg_kalman_scalar_t *me, float process_noise,
                                           float measurement_noise, float initial_estimate,
                                           float initial_covariance)
{
    if (me == NULL) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}

    me->is_initialized = false;

    // 参数校验：Q >= 0，R > 0，P >= 0
    if (!alg_kalman_scalar_is_nonnegative_finite(process_noise) || !isfinite(measurement_noise) ||
        (measurement_noise <= 0.0F) || !isfinite(initial_estimate) ||
        !alg_kalman_scalar_is_nonnegative_finite(initial_covariance)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    me->process_noise = process_noise;
    me->measurement_noise = measurement_noise;
    me->estimate = initial_estimate;
    me->covariance = initial_covariance;
    me->gain = 0.0F;
    me->is_initialized = true;

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 修改标量卡尔曼的噪声参数
 * @param me 滤波器对象
 * @param process_noise 过程噪声方差 Q（>= 0）
 * @param measurement_noise 测量噪声方差 R（> 0）
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_scalar_set_noise(alg_kalman_scalar_t *me, float process_noise,
                                                float measurement_noise)
{
    if (me == NULL) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
}
    if (!alg_kalman_scalar_is_nonnegative_finite(process_noise) || !isfinite(measurement_noise) ||
        (measurement_noise <= 0.0F)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    me->process_noise = process_noise;
    me->measurement_noise = measurement_noise;
    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 重置标量卡尔曼滤波器
 * @param me 滤波器对象
 * @param initial_estimate 初始估计值
 * @param initial_covariance 初始协方差（>= 0）
 * @return 执行状态
 */
alg_kalman_status_t alg_kalman_scalar_reset(alg_kalman_scalar_t *me, float initial_estimate,
                                            float initial_covariance)
{
    if (me == NULL) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(initial_estimate) || !alg_kalman_scalar_is_nonnegative_finite(initial_covariance)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    me->estimate = initial_estimate;
    me->covariance = initial_covariance;
    me->gain = 0.0F;
    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 标量卡尔曼预测步骤
 * @param me 滤波器对象
 * @param state_delta 状态变化量（可为零）
 * @return 执行状态
 * @note x = x + delta，P = P + Q
 */
alg_kalman_status_t alg_kalman_scalar_predict(alg_kalman_scalar_t *me, float state_delta)
{
    if (me == NULL) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(state_delta)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    me->estimate += state_delta;
    me->covariance += me->process_noise;

    if (!isfinite(me->estimate) || !alg_kalman_scalar_is_nonnegative_finite(me->covariance)) {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 标量卡尔曼校正步骤
 * @param me 滤波器对象
 * @param measurement 测量值
 * @param output 输出滤波结果
 * @return 执行状态
 * @note K = P/(P+R)，x = x + K*(z-x)，P = (1-K)*P
 */
alg_kalman_status_t alg_kalman_scalar_correct(alg_kalman_scalar_t *me, float measurement,
                                              float *output)
{
    float innovation_covariance;

    if ((me == NULL) || (output == NULL)) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(measurement)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    // 创新协方差 S = P + R
    innovation_covariance = me->covariance + me->measurement_noise;
    if (!isfinite(innovation_covariance) || (innovation_covariance <= 0.0F)) {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

    // 卡尔曼增益 K = P / (P + R)
    me->gain = me->covariance / innovation_covariance;

    // 状态更新 x = x + K*(z - x)
    me->estimate += me->gain * (measurement - me->estimate);

    // 协方差更新 P = (1 - K) * P
    me->covariance = (1.0F - me->gain) * me->covariance;

    if (!isfinite(me->estimate) || !alg_kalman_scalar_is_nonnegative_finite(me->covariance)) {
        return ALG_KALMAN_STATUS_NUMERICAL_ERROR;
}

    *output = me->estimate;
    return ALG_KALMAN_STATUS_OK;
}

/**
 * @brief 标量卡尔曼完整更新（预测 + 校正，state_delta = 0）
 * @param me 滤波器对象
 * @param measurement 测量值
 * @param output 输出滤波结果
 * @return 执行状态
 * @note 相当于 predict(0) + correct()
 */
alg_kalman_status_t alg_kalman_scalar_update(alg_kalman_scalar_t *me, float measurement,
                                             float *output)
{
    alg_kalman_status_t status;

    if ((me == NULL) || (output == NULL)) {
        return ALG_KALMAN_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_KALMAN_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(measurement)) {
        return ALG_KALMAN_STATUS_OUT_OF_RANGE;
}

    status = alg_kalman_scalar_predict(me, 0.0F);
    if (status != ALG_KALMAN_STATUS_OK) {
        return status;
}

    return alg_kalman_scalar_correct(me, measurement, output);
}