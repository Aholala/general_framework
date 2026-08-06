/**
 * @file alg_chassis_wheel_monitor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 车轮状态监测器头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 使用轮速残差和传感器在线状态生成稳定的 wheel_is_available 数组。
 *       残差连续超过故障阈值后标记异常，连续低于恢复阈值后恢复，
 *       两个阈值形成滞回，避免状态反复跳变。
 *       本模块只判断车轮是否可用于解算，不读取 CAN、不采集编码器，也不直接停止电机。
 */

#ifndef ALG_CHASSIS_WHEEL_MONITOR_H
#define ALG_CHASSIS_WHEEL_MONITOR_H

#include "alg_chassis_motion.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 单轮状态结构体 ======================== */

    /**
     * @brief 车轮监测状态
     * @note 每个轮子独立维护，包含当前残差、故障/恢复确认计数和故障标志
     */
    typedef struct
    {
        float residual_m_per_s;               // 当前残差绝对值（m/s）
        uint32_t fault_confirmation_count;    // 故障确认计数（连续超过故障阈值次数）
        uint32_t recovery_confirmation_count; // 恢复确认计数（连续低于恢复阈值次数）
        bool is_faulted;                      // 是否已标记为故障
    } alg_chassis_wheel_monitor_wheel_state_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 车轮监测器配置
     * @note 故障阈值 > 恢复阈值，形成滞回
     *       确认样本数用于防抖，避免瞬时噪声导致状态跳变
     */
    typedef struct
    {
        size_t wheel_count;                        // 轮子数量
        float fault_residual_threshold_m_per_s;    // 故障残差阈值（m/s）
        float recovery_residual_threshold_m_per_s; // 恢复残差阈值（m/s），必须 < 故障阈值
        uint32_t fault_confirmation_samples;       // 故障确认所需连续样本数
        uint32_t recovery_confirmation_samples;    // 恢复确认所需连续样本数
        alg_chassis_wheel_monitor_wheel_state_t *wheel_state_storage; // 状态存储数组
    } alg_chassis_wheel_monitor_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 车轮监测器对象
     * @note wheel_states 指向调用者提供的状态存储数组
     */
    typedef struct
    {
        size_t wheel_count;                                    // 轮子数量
        float fault_residual_threshold_m_per_s;                // 故障残差阈值
        float recovery_residual_threshold_m_per_s;             // 恢复残差阈值
        uint32_t fault_confirmation_samples;                   // 故障确认样本数
        uint32_t recovery_confirmation_samples;                // 恢复确认样本数
        alg_chassis_wheel_monitor_wheel_state_t *wheel_states; // 状态数组
        bool is_initialized;                                   // 是否已初始化
    } alg_chassis_wheel_monitor_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化车轮监测器
     * @param me 监测器对象
     * @param config 配置参数
     * @return 执行状态
     */
    alg_chassis_status_t
    alg_chassis_wheel_monitor_init(alg_chassis_wheel_monitor_t *me,
                                   const alg_chassis_wheel_monitor_config_t *config);

    /**
     * @brief 更新车轮监测状态
     * @param me 监测器对象
     * @param wheel_residuals_m_per_s 轮速残差数组（m/s）
     * @param sensor_is_available 传感器可用性数组（NULL 表示全部可用）
     * @param wheel_is_available 输出轮子可用性数组
     * @param output_capacity 输出数组容量
     * @return 执行状态（OK=全部可用，DEGRADED=部分不可用）
     * @note 残差连续超过故障阈值后标记故障，连续低于恢复阈值后恢复
     *       传感器不可用直接标记为故障，无需等待残差确认
     */
    alg_chassis_status_t alg_chassis_wheel_monitor_update(alg_chassis_wheel_monitor_t *me,
                                                          const float *wheel_residuals_m_per_s,
                                                          const bool *sensor_is_available,
                                                          bool *wheel_is_available,
                                                          size_t output_capacity);

    /**
     * @brief 重置指定轮子的状态
     * @param me 监测器对象
     * @param wheel_index 轮子索引
     * @param assume_available true=设为可用，false=设为故障
     * @return 执行状态
     */
    alg_chassis_status_t alg_chassis_wheel_monitor_reset_wheel(alg_chassis_wheel_monitor_t *me,
                                                               size_t wheel_index,
                                                               bool assume_available);

    /**
     * @brief 获取指定轮子的监测状态
     * @param me 监测器对象
     * @param wheel_index 轮子索引
     * @return 状态指针，无效则返回 NULL
     */
    const alg_chassis_wheel_monitor_wheel_state_t *
    alg_chassis_wheel_monitor_get_wheel_state(const alg_chassis_wheel_monitor_t *me,
                                              size_t wheel_index);

#ifdef __cplusplus
}
#endif

#endif