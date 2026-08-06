/**
 * @file alg_chassis_wheel_monitor.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 车轮状态监测器
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 使用残差和传感器可用性判断车轮是否可用于底盘解算。
 *       故障/恢复使用滞回阈值和防抖确认，避免状态频繁跳变。
 */

#include "alg_chassis_wheel_monitor.h"

#include <math.h>   // fabsf, isfinite
#include <stddef.h> // NULL

/**
 * @brief 安全递增计数器（防溢出）
 * @param counter 计数器值
 * @return 递增后的值（饱和到 UINT32_MAX）
 */
static uint32_t alg_chassis_wheel_monitor_increment_counter(uint32_t counter)
{
    return (counter < UINT32_MAX) ? counter + 1U : UINT32_MAX;
}

/**
 * @brief 初始化车轮监测器
 * @param me 监测器对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_chassis_status_t
alg_chassis_wheel_monitor_init(alg_chassis_wheel_monitor_t *me,
                               const alg_chassis_wheel_monitor_config_t *config)
{
    size_t wheel_index;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->wheel_count == 0U) ||
        (config->wheel_state_storage == NULL) ||
        !isfinite(config->fault_residual_threshold_m_per_s) ||
        !isfinite(config->recovery_residual_threshold_m_per_s) ||
        (config->fault_residual_threshold_m_per_s <= 0.0F) ||
        (config->recovery_residual_threshold_m_per_s < 0.0F) ||
        (config->recovery_residual_threshold_m_per_s >= config->fault_residual_threshold_m_per_s) ||
        (config->fault_confirmation_samples == 0U) || (config->recovery_confirmation_samples == 0U))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    me->is_initialized = false;
    me->wheel_count = config->wheel_count;
    me->fault_residual_threshold_m_per_s = config->fault_residual_threshold_m_per_s;
    me->recovery_residual_threshold_m_per_s = config->recovery_residual_threshold_m_per_s;
    me->fault_confirmation_samples = config->fault_confirmation_samples;
    me->recovery_confirmation_samples = config->recovery_confirmation_samples;
    me->wheel_states = config->wheel_state_storage;

    // ---- 清空所有轮子状态 ----
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        me->wheel_states[wheel_index] = (alg_chassis_wheel_monitor_wheel_state_t){0};
    }

    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 更新车轮监测状态（核心函数）
 * @param me 监测器对象
 * @param wheel_residuals_m_per_s 轮速残差数组
 * @param sensor_is_available 传感器可用性数组
 * @param wheel_is_available 输出轮子可用性数组
 * @param output_capacity 输出数组容量
 * @return 执行状态
 * @note 状态机：
 *       1. 传感器不可用 → 直接标记故障
 *       2. 正常状态 + 残差 >= 故障阈值 → 累积故障计数，达到样本数后标记故障
 *       3. 故障状态 + 残差 <= 恢复阈值 → 累积恢复计数，达到样本数后恢复
 *       4. 其他情况 → 重置计数器
 */
alg_chassis_status_t alg_chassis_wheel_monitor_update(alg_chassis_wheel_monitor_t *me,
                                                      const float *wheel_residuals_m_per_s,
                                                      const bool *sensor_is_available,
                                                      bool *wheel_is_available,
                                                      size_t output_capacity)
{
    size_t wheel_index;
    size_t available_wheel_count = 0U;

    // ---- 参数校验 ----
    if ((me == NULL) || (wheel_residuals_m_per_s == NULL) || (wheel_is_available == NULL)) {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
}
    if (output_capacity < me->wheel_count) {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
}

    // ---- 遍历所有轮子 ----
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        alg_chassis_wheel_monitor_wheel_state_t *const wheel_state = &me->wheel_states[wheel_index];
        const bool sensor_available =
            (sensor_is_available == NULL) || sensor_is_available[wheel_index];
        const float residual_m_per_s = fabsf(wheel_residuals_m_per_s[wheel_index]);

        // 检查残差是否为有限数
        if (!isfinite(residual_m_per_s)) {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
}

        // 保存当前残差
        wheel_state->residual_m_per_s = residual_m_per_s;

        // ---- 情况1：传感器不可用 ----
        // 直接标记故障，无需等待残差确认
        if (!sensor_available)
        {
            wheel_state->is_faulted = true;
            wheel_state->fault_confirmation_count = me->fault_confirmation_samples;
            wheel_state->recovery_confirmation_count = 0U;
        }
        // ---- 情况2：当前为正常状态 ----
        else if (!wheel_state->is_faulted)
        {
            wheel_state->recovery_confirmation_count = 0U; // 重置恢复计数

            // 检查残差是否超过故障阈值
            if (residual_m_per_s >= me->fault_residual_threshold_m_per_s)
            {
                // 递增故障确认计数
                wheel_state->fault_confirmation_count = alg_chassis_wheel_monitor_increment_counter(
                    wheel_state->fault_confirmation_count);
                // 达到确认样本数 → 标记故障
                if (wheel_state->fault_confirmation_count >= me->fault_confirmation_samples)
                {
                    wheel_state->is_faulted = true;
                    wheel_state->recovery_confirmation_count = 0U;
                }
            }
            else
            {
                // 残差正常，重置故障计数
                wheel_state->fault_confirmation_count = 0U;
            }
        }
        // ---- 情况3：当前为故障状态 ----
        else
        {
            wheel_state->fault_confirmation_count = 0U; // 重置故障计数

            // 检查残差是否低于恢复阈值
            if (residual_m_per_s <= me->recovery_residual_threshold_m_per_s)
            {
                // 递增恢复确认计数
                wheel_state->recovery_confirmation_count =
                    alg_chassis_wheel_monitor_increment_counter(
                        wheel_state->recovery_confirmation_count);
                // 达到确认样本数 → 恢复正常
                if (wheel_state->recovery_confirmation_count >= me->recovery_confirmation_samples)
                {
                    wheel_state->is_faulted = false;
                    wheel_state->recovery_confirmation_count = 0U;
                }
            }
            else
            {
                // 残差仍高于恢复阈值，重置恢复计数
                wheel_state->recovery_confirmation_count = 0U;
            }
        }

        // ---- 输出可用性 ----
        wheel_is_available[wheel_index] = !wheel_state->is_faulted;
        if (wheel_is_available[wheel_index]) {
            ++available_wheel_count;
}
    }

    // 部分轮子不可用 → 返回 DEGRADED
    return (available_wheel_count == me->wheel_count) ? ALG_CHASSIS_STATUS_OK
                                                      : ALG_CHASSIS_STATUS_DEGRADED;
}

/**
 * @brief 重置指定轮子的状态
 * @param me 监测器对象
 * @param wheel_index 轮子索引
 * @param assume_available true=设为可用，false=设为故障
 * @return 执行状态
 * @note 常用于手动恢复被误判的轮子，或在初始化后强制设置初始状态
 */
alg_chassis_status_t alg_chassis_wheel_monitor_reset_wheel(alg_chassis_wheel_monitor_t *me,
                                                           size_t wheel_index,
                                                           bool assume_available)
{
    if (me == NULL) {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
}
    if (wheel_index >= me->wheel_count) {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
}

    // 重置状态：清零计数，根据 assume_available 设置故障标志
    me->wheel_states[wheel_index] = (alg_chassis_wheel_monitor_wheel_state_t){
        .is_faulted = !assume_available,
    };
    return ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 获取指定轮子的监测状态
 * @param me 监测器对象
 * @param wheel_index 轮子索引
 * @return 状态指针，无效则返回 NULL
 */
const alg_chassis_wheel_monitor_wheel_state_t *
alg_chassis_wheel_monitor_get_wheel_state(const alg_chassis_wheel_monitor_t *me, size_t wheel_index)
{
    return ((me != NULL) && me->is_initialized && (wheel_index < me->wheel_count))
               ? &me->wheel_states[wheel_index]
               : NULL;
}