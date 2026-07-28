/**
 * @file alg_trajectory.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 一维在线轨迹生成器头文件（梯形速度 / S 曲线）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       支持位置目标和速度目标两种模式，可在运行中平滑切换目标。
 *       提供梯形速度（恒加/减速）和 S 曲线（加加速度限制）两种剖面。
 *       所有数据由调用者管理，无动态内存。
 *       每个轨迹对象独立，支持多实例。
 */

#ifndef ALG_TRAJECTORY_H
#define ALG_TRAJECTORY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 轨迹生成器状态码
     */
    typedef enum
    {
        ALG_TRAJECTORY_STATUS_OK = 0,           // 正常更新（目标未完成）
        ALG_TRAJECTORY_STATUS_FINISHED,         // 目标已完成（同时返回状态码）
        ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT, // 参数非法（空指针、非法数值等）
        ALG_TRAJECTORY_STATUS_NOT_INITIALIZED,  // 对象未初始化
        ALG_TRAJECTORY_STATUS_NUMERICAL_ERROR   // 数值错误（溢出、非有限结果）
    } alg_trajectory_status_t;

    /**
     * @brief 轨迹剖面类型
     */
    typedef enum
    {
        ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL = 0, // 梯形速度（恒加/减速）
        ALG_TRAJECTORY_PROFILE_S_CURVE          // S 曲线（加加速度限制）
    } alg_trajectory_profile_t;

    /**
     * @brief 目标类型（内部使用）
     */
    typedef enum
    {
        ALG_TRAJECTORY_TARGET_POSITION = 0, // 位置目标（到达指定位置）
        ALG_TRAJECTORY_TARGET_VELOCITY      // 速度目标（达到指定速度）
    } alg_trajectory_target_type_t;

    /* ======================== 配置与状态结构体 ======================== */

    /**
     * @brief 轨迹生成器配置
     * @note 所有物理量需使用一致的单位（如 m、m/s 或 rad、rad/s）。
     */
    typedef struct
    {
        float maximum_velocity_per_s;      // 最大速度（>0）
        float maximum_acceleration_per_s2; // 最大加速度（>0）
        float maximum_deceleration_per_s2; // 最大减速度（>0）
        float maximum_jerk_per_s3;         // 最大加加速度（>0，S 曲线有效）
        float position_tolerance;          // 位置到达容差（>=0）
        float velocity_tolerance_per_s;    // 速度到达容差（>=0）
    } alg_trajectory_config_t;

    /**
     * @brief 轨迹状态（位置、速度、加速度）
     */
    typedef struct
    {
        float position;            // 当前位置
        float velocity_per_s;      // 当前速度
        float acceleration_per_s2; // 当前加速度
    } alg_trajectory_state_t;

    /**
     * @brief 一维轨迹生成器实例
     * @note 存储配置、当前状态、目标值及运行标志。
     */
    typedef struct
    {
        alg_trajectory_config_t config;           // 配置
        alg_trajectory_state_t state;             // 当前状态
        float target_position;                    // 目标位置（位置模式）
        float target_velocity_per_s;              // 目标速度（速度模式或位置模式的终端速度）
        alg_trajectory_profile_t profile;         // 当前剖面类型
        alg_trajectory_target_type_t target_type; // 当前目标类型
        bool is_finished;                         // 是否已完成
        bool is_initialized;                      // 是否已初始化
    } alg_trajectory_t;

    /* ======================== API 函数 ======================== */

    /**
     * @brief 初始化轨迹生成器
     * @param me            轨迹对象
     * @param config        配置参数
     * @param profile       初始剖面类型
     * @param initial_state 初始状态（位置、速度、加速度）
     * @return 执行状态
     */
    alg_trajectory_status_t alg_trajectory_init(alg_trajectory_t *me,
                                                const alg_trajectory_config_t *config,
                                                alg_trajectory_profile_t profile,
                                                const alg_trajectory_state_t *initial_state);

    /**
     * @brief 重置轨迹生成器（软复位，状态变为给定值并视为已完成）
     * @param me    轨迹对象
     * @param state 新状态
     * @return 执行状态
     */
    alg_trajectory_status_t alg_trajectory_reset(alg_trajectory_t *me,
                                                 const alg_trajectory_state_t *state);

    /**
     * @brief 设置位置目标
     * @param me                   轨迹对象
     * @param target_position      目标位置
     * @param terminal_velocity_per_s  到达目标时的期望速度（绝对值不超过最大速度）
     * @return 执行状态
     * @note 从当前位置/速度开始规划，目标类型切换为位置。
     *       如果 terminal_velocity_per_s 不为零，则到达时保持该速度（可用于连续轨迹）。
     */
    alg_trajectory_status_t alg_trajectory_set_position_target(alg_trajectory_t *me,
                                                               float target_position,
                                                               float terminal_velocity_per_s);

    /**
     * @brief 设置速度目标
     * @param me                   轨迹对象
     * @param target_velocity_per_s  目标速度（绝对值不超过最大速度）
     * @return 执行状态
     * @note 目标类型切换为速度，生成器持续调整速度直至达到目标速度。
     *       到达后保持该速度，不限制位置。
     */
    alg_trajectory_status_t alg_trajectory_set_velocity_target(alg_trajectory_t *me,
                                                               float target_velocity_per_s);

    /**
     * @brief 运行时切换剖面类型
     * @param me      轨迹对象
     * @param profile 新剖面类型
     * @return 执行状态
     */
    alg_trajectory_status_t alg_trajectory_set_profile(alg_trajectory_t *me,
                                                       alg_trajectory_profile_t profile);

    /**
     * @brief 更新轨迹（单步积分）
     * @param me           轨迹对象
     * @param delta_time_s 时间步长（秒，>0）
     * @param output_state 输出更新后的状态
     * @return 执行状态（如果目标完成则返回 ALG_TRAJECTORY_STATUS_FINISHED）
     * @note 根据当前目标类型和剖面计算加速度，然后更新速度和位置。
     *       梯形剖面直接使用最大加/减速度；S 曲线额外限制加加速度。
     *       完成时会将状态精确对齐目标值。
     */
    alg_trajectory_status_t alg_trajectory_update(alg_trajectory_t *me, float delta_time_s,
                                                  alg_trajectory_state_t *output_state);

    /**
     * @brief 获取当前状态（只读）
     * @param me    轨迹对象
     * @param state 输出状态
     * @return 执行状态
     */
    alg_trajectory_status_t alg_trajectory_get_state(const alg_trajectory_t *me,
                                                     alg_trajectory_state_t *state);

    /**
     * @brief 查询轨迹是否已完成
     * @param me 轨迹对象
     * @return true 表示已完成
     */
    bool alg_trajectory_is_finished(const alg_trajectory_t *me);

    /**
     * @brief 计算恒定减速度下的制动距离
     * @param velocity_per_s       当前速度
     * @param deceleration_per_s2  减速度（>0）
     * @return 制动距离（如果输入无效则返回 NaN）
     * @note 用于限位预判，不考虑 jerk 过渡。
     */
    float alg_trajectory_calculate_stopping_distance(float velocity_per_s,
                                                     float deceleration_per_s2);

#ifdef __cplusplus
}
#endif

#endif /* ALG_TRAJECTORY_H */