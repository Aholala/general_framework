/**
 * @file alg_filter_complementary.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 互补滤波器实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 融合测量值和变化率（积分）进行状态估计。
 *       适用于 IMU 姿态融合（加速度计 + 陀螺仪）、位置估计（GPS + 速度积分）等场景。
 *       实现：output = weight * (output + rate*dt) + (1-weight) * measured
 */

#include "alg_filter.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 初始化互补滤波器
 * @param me 滤波器对象
 * @param prediction_weight 预测权重（0~1），值越大越信任积分预测
 * @param initial_output 初始输出值
 * @return 执行状态
 * @note prediction_weight=1 时完全信任积分（纯预测），可能发散
 *       prediction_weight=0 时完全信任测量值（无滤波）
 *       推荐值 0.5~0.95，取决于测量噪声和积分漂移特性
 */
alg_filter_status_t alg_filter_complementary_init(alg_filter_complementary_t *me,
                                                  float prediction_weight, float initial_output)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) || (prediction_weight > 1.0F) ||
        !isfinite(initial_output))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->prediction_weight = prediction_weight;
    me->output = initial_output;
    me->is_initialized = true;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 修改互补滤波器的预测权重
 * @param me 滤波器对象
 * @param prediction_weight 新的预测权重（0~1）
 * @return 执行状态
 */
alg_filter_status_t alg_filter_complementary_set_weight(alg_filter_complementary_t *me,
                                                        float prediction_weight)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(prediction_weight) || (prediction_weight < 0.0F) || (prediction_weight > 1.0F))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->prediction_weight = prediction_weight;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 重置互补滤波器
 * @param me 滤波器对象
 * @param initial_output 初始输出值
 * @return 执行状态
 */
alg_filter_status_t alg_filter_complementary_reset(alg_filter_complementary_t *me,
                                                   float initial_output)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(initial_output))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->output = initial_output;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新互补滤波器
 * @param me 滤波器对象
 * @param measured_value 当前测量值
 * @param measured_rate_per_s 变化率（单位/秒）
 * @param delta_time_s 时间步长（秒），必须 > 0
 * @param output 输出滤波值
 * @return 执行状态
 * @note 算法：output = weight * (output + rate*dt) + (1-weight) * measured
 *       即：预测值 = output + rate*dt（用输出积分预测当前值）
 *       然后与测量值加权平均
 */
alg_filter_status_t alg_filter_complementary_update(alg_filter_complementary_t *me,
                                                    float measured_value, float measured_rate_per_s,
                                                    float delta_time_s, float *output)
{
    float predicted_value;

    if ((me == NULL) || (output == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(measured_value) || !isfinite(measured_rate_per_s) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // 预测值 = 上次输出 + 变化率 * 时间步长
    predicted_value = me->output + (measured_rate_per_s * delta_time_s);

    // 互补加权：output = weight * 预测值 + (1-weight) * 测量值
    me->output = (me->prediction_weight * predicted_value) +
                 ((1.0F - me->prediction_weight) * measured_value);

    if (!isfinite(me->output))
        return ALG_FILTER_STATUS_NUMERICAL_ERROR;

    *output = me->output;
    return ALG_FILTER_STATUS_OK;
}