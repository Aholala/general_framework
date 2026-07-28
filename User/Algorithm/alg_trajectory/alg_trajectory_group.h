/**
 * @file alg_trajectory_group.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 多轴同步轨迹生成器头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 使用五次多项式时间缩放，实现任意数量轴同时起步、同时停止。
 *       每轴可独立配置速度、加速度和加加速度限制。
 *       所有存储由调用者提供，无动态内存。
 *       同步组以零端速度和零端加速度结束。
 *       适用于云台 pitch/yaw、多舵轮转向、机械臂联动等场景。
 */

#ifndef ALG_TRAJECTORY_GROUP_H
#define ALG_TRAJECTORY_GROUP_H

#include "alg_trajectory.h"
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 多轴同步轨迹组实例
     * @note 所有数组由调用者提供并在对象生命周期内保持有效。
     */
    typedef struct
    {
        alg_trajectory_config_t *axis_configs; // 每轴配置数组（长度 axis_count）
        alg_trajectory_state_t *axis_states;   // 每轴当前状态数组
        float *start_positions;                // 每轴起始位置数组
        float *target_positions;               // 每轴目标位置数组
        size_t axis_count;                     // 轴数量
        float elapsed_time_s;                  // 已运行时间
        float duration_s;                      // 总持续时间
        bool is_finished;                      // 是否已完成
        bool is_initialized;                   // 是否已初始化
    } alg_trajectory_group_t;

    /* ======================== API 函数 ======================== */

    /**
     * @brief 初始化多轴同步轨迹组
     * @param me                    轨迹组对象
     * @param axis_config_storage   每轴配置存储（外部数组）
     * @param axis_state_storage    每轴状态存储（外部数组）
     * @param start_position_storage 起始位置存储（外部数组）
     * @param target_position_storage 目标位置存储（外部数组）
     * @param axis_count            轴数量
     * @param initial_states        初始状态数组（每轴位置、速度、加速度）
     * @param axis_configs          每轴配置数组
     * @return 执行状态
     * @note 所有存储数组在对象生命周期内必须保持有效。
     *       初始速度和加速度必须为零（同步组以静止起始）。
     */
    alg_trajectory_status_t alg_trajectory_group_init(
        alg_trajectory_group_t *me, alg_trajectory_config_t *axis_config_storage,
        alg_trajectory_state_t *axis_state_storage, float *start_position_storage,
        float *target_position_storage, size_t axis_count,
        const alg_trajectory_state_t *initial_states, const alg_trajectory_config_t *axis_configs);

    /**
     * @brief 设置目标位置（所有轴同步运动）
     * @param me               轨迹组对象
     * @param target_positions 每轴目标位置数组
     * @return 执行状态
     * @note 根据每轴位移和限制计算共同的持续时间，保证同步起停。
     *       若位移为零，则该轴保持静止。
     */
    alg_trajectory_status_t alg_trajectory_group_set_target(alg_trajectory_group_t *me,
                                                            const float *target_positions);

    /**
     * @brief 更新轨迹（单步）
     * @param me            轨迹组对象
     * @param delta_time_s  时间步长（>0）
     * @return 执行状态
     * @note 使用五次多项式对归一化时间进行插值，生成位置、速度、加速度。
     *       所有轴同步推进。
     */
    alg_trajectory_status_t alg_trajectory_group_update(alg_trajectory_group_t *me,
                                                        float delta_time_s);

    /**
     * @brief 获取指定轴当前状态
     * @param me          轨迹组对象
     * @param axis_index  轴索引
     * @return 状态指针（若索引无效或对象未初始化则返回 NULL）
     */
    const alg_trajectory_state_t *alg_trajectory_group_get_state(const alg_trajectory_group_t *me,
                                                                 size_t axis_index);

    /**
     * @brief 查询轨迹组是否已完成
     * @param me 轨迹组对象
     * @return true 表示已完成
     */
    bool alg_trajectory_group_is_finished(const alg_trajectory_group_t *me);

#ifdef __cplusplus
}
#endif

#endif