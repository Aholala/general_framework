/**
 * @file alg_filter_window.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 窗口滤波器实现（滑动平均、中值滤波）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 滑动平均维护固定大小窗口的均值。
 *       中值滤波通过排序窗口样本取中位数，需要两个等长缓冲区。
 *       所有缓冲区由调用者提供，不分配动态内存。
 */

#include "alg_filter.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 清空缓冲区（全部置零）
 * @param buffer 缓冲区指针
 * @param capacity 缓冲区大小
 */
static void alg_filter_window_clear(float *buffer, size_t capacity)
{
    size_t index;
    for (index = 0U; index < capacity; ++index)
        buffer[index] = 0.0F;
}

/* ======================== 滑动平均滤波器 ======================== */

/**
 * @brief 初始化滑动平均滤波器
 * @param me 滤波器对象
 * @param sample_buffer 样本缓冲区（调用者分配），大小至少为 capacity
 * @param capacity 窗口大小，必须 > 0
 * @return 执行状态
 * @note 缓冲区在滤波器生命周期内必须保持有效
 */
alg_filter_status_t alg_filter_moving_average_init(alg_filter_moving_average_t *me,
                                                   float *sample_buffer, size_t capacity)
{
    if ((me == NULL) || (sample_buffer == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (capacity == 0U)
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    me->sample_buffer = sample_buffer;
    me->capacity = capacity;
    me->is_initialized = true;
    return alg_filter_moving_average_reset(me);
}

/**
 * @brief 重置滑动平均滤波器（清空所有样本）
 * @param me 滤波器对象
 * @return 执行状态
 */
alg_filter_status_t alg_filter_moving_average_reset(alg_filter_moving_average_t *me)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;

    alg_filter_window_clear(me->sample_buffer, me->capacity);
    me->sample_count = 0U;
    me->write_index = 0U;
    me->sum = 0.0F;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新滑动平均滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param output 输出滤波值
 * @return 执行状态
 * @note 使用环形缓冲区维护窗口，平均值为 sum / sample_count
 *       窗口满后移除最旧样本再添加新样本
 */
alg_filter_status_t alg_filter_moving_average_update(alg_filter_moving_average_t *me, float input,
                                                     float *output)
{
    if ((me == NULL) || (output == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(input))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // 窗口已满：移除最旧样本
    if (me->sample_count == me->capacity)
        me->sum -= me->sample_buffer[me->write_index];
    else
        ++me->sample_count; // 窗口未满：增加计数

    // 写入新样本
    me->sample_buffer[me->write_index] = input;
    me->sum += input;
    me->write_index = (me->write_index + 1U) % me->capacity;

    // 计算均值
    *output = me->sum / (float)me->sample_count;
    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}

/* ======================== 中值滤波器 ======================== */

/**
 * @brief 初始化中值滤波器
 * @param me 滤波器对象
 * @param sample_buffer 样本缓冲区（调用者分配）
 * @param sort_buffer 排序工作区（调用者分配），不能与 sample_buffer 相同
 * @param capacity 窗口大小，必须 > 0
 * @return 执行状态
 * @note 两个缓冲区在滤波器生命周期内必须保持有效
 *       排序工作在每次更新时被覆盖，不需要持久保存
 */
alg_filter_status_t alg_filter_median_init(alg_filter_median_t *me, float *sample_buffer,
                                           float *sort_buffer, size_t capacity)
{
    if ((me == NULL) || (sample_buffer == NULL) || (sort_buffer == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if ((capacity == 0U) || (sample_buffer == sort_buffer))
        return ALG_FILTER_STATUS_OUT_OF_RANGE; // sample_buffer 和 sort_buffer 不能相同

    me->sample_buffer = sample_buffer;
    me->sort_buffer = sort_buffer;
    me->capacity = capacity;
    me->is_initialized = true;
    return alg_filter_median_reset(me);
}

/**
 * @brief 重置中值滤波器
 * @param me 滤波器对象
 * @return 执行状态
 */
alg_filter_status_t alg_filter_median_reset(alg_filter_median_t *me)
{
    if (me == NULL)
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;

    alg_filter_window_clear(me->sample_buffer, me->capacity);
    alg_filter_window_clear(me->sort_buffer, me->capacity);
    me->sample_count = 0U;
    me->write_index = 0U;
    return ALG_FILTER_STATUS_OK;
}

/**
 * @brief 更新中值滤波器
 * @param me 滤波器对象
 * @param input 当前输入值
 * @param output 输出滤波值（中位数）
 * @return 执行状态
 * @note 算法步骤：
 *       1. 将新样本写入环形缓冲区
 *       2. 复制样本到排序工作区
 *       3. 使用插入排序对工作区排序
 *       4. 取中位数（奇数取中间值，偶数取两个中间值的平均）
 */
alg_filter_status_t alg_filter_median_update(alg_filter_median_t *me, float input, float *output)
{
    size_t source_index;
    size_t insertion_index;
    float value;

    if ((me == NULL) || (output == NULL))
        return ALG_FILTER_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_FILTER_STATUS_NOT_INITIALIZED;
    if (!isfinite(input))
        return ALG_FILTER_STATUS_OUT_OF_RANGE;

    // 写入新样本到环形缓冲区
    me->sample_buffer[me->write_index] = input;
    me->write_index = (me->write_index + 1U) % me->capacity;
    if (me->sample_count < me->capacity)
        ++me->sample_count;

    // 复制样本到排序工作区
    for (source_index = 0U; source_index < me->sample_count; ++source_index)
        me->sort_buffer[source_index] = me->sample_buffer[source_index];

    // 插入排序（对少量数据效率足够）
    for (source_index = 1U; source_index < me->sample_count; ++source_index)
    {
        value = me->sort_buffer[source_index];
        insertion_index = source_index;
        while ((insertion_index > 0U) && (me->sort_buffer[insertion_index - 1U] > value))
        {
            me->sort_buffer[insertion_index] = me->sort_buffer[insertion_index - 1U];
            --insertion_index;
        }
        me->sort_buffer[insertion_index] = value;
    }

    // 取中位数
    if ((me->sample_count % 2U) == 0U)
    {
        // 偶数：取两个中间值的平均
        const size_t upper_index = me->sample_count / 2U;
        *output = 0.5F * (me->sort_buffer[upper_index - 1U] + me->sort_buffer[upper_index]);
    }
    else
    {
        // 奇数：取中间值
        *output = me->sort_buffer[me->sample_count / 2U];
    }

    return isfinite(*output) ? ALG_FILTER_STATUS_OK : ALG_FILTER_STATUS_NUMERICAL_ERROR;
}