/**
 * @file alg_imu_ekf_internal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 内部接口头文件
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 仅供本模块内部使用，不对外暴露。
 *       包含内部工具函数和 EKF 模型函数声明。
 */

#ifndef ALG_IMU_EKF_INTERNAL_H
#define ALG_IMU_EKF_INTERNAL_H

#include "alg_imu_ekf.h"

/**
 * @brief 检查数组元素是否全部为有限数
 * @param values 数组指针
 * @param value_count 元素个数
 * @return true=全有限
 */
bool alg_imu_ekf_internal_is_finite_array(const float *values, size_t value_count);

/**
 * @brief 四元数归一化及协方差投影
 * @param me EKF 对象
 * @return 执行状态
 * @note 保持四元数单位长度，并将协方差投影到单位四元数约束流形上
 */
alg_imu_ekf_status_t alg_imu_ekf_internal_normalize_and_project(alg_imu_ekf_t *me);

/**
 * @brief 将卡尔曼状态码映射到 IMU EKF 状态码
 * @param status 卡尔曼状态码
 * @return IMU EKF 状态码
 */
alg_imu_ekf_status_t alg_imu_ekf_internal_map_kalman_status(alg_kalman_status_t status);

/**
 * @brief EKF 状态转移函数
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param control_input 控制输入（陀螺仪）
 * @param control_dimension 控制维度
 * @param delta_time_s 时间步长
 * @param predicted_state 输出预测状态
 * @param user_context 用户上下文（EKF 对象）
 * @return 卡尔曼状态码
 */
alg_kalman_status_t alg_imu_ekf_internal_state_function(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *predicted_state,
                                                        void *user_context);

/**
 * @brief 状态转移雅可比矩阵
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param control_input 控制输入
 * @param control_dimension 控制维度
 * @param delta_time_s 时间步长
 * @param state_jacobian 输出雅可比矩阵
 * @param user_context 用户上下文
 * @return 卡尔曼状态码
 */
alg_kalman_status_t alg_imu_ekf_internal_state_jacobian(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *state_jacobian,
                                                        void *user_context);

/**
 * @brief 测量函数（从状态预测加速度方向）
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param measurement_dimension 测量维度
 * @param predicted_measurement 输出预测测量
 * @param user_context 用户上下文
 * @return 卡尔曼状态码
 */
alg_kalman_status_t alg_imu_ekf_internal_measurement_function(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *predicted_measurement,
                                                              void *user_context);

/**
 * @brief 测量雅可比矩阵
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param measurement_dimension 测量维度
 * @param measurement_jacobian 输出雅可比矩阵
 * @param user_context 用户上下文
 * @return 卡尔曼状态码
 */
alg_kalman_status_t alg_imu_ekf_internal_measurement_jacobian(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *measurement_jacobian,
                                                              void *user_context);

#endif /* ALG_IMU_EKF_INTERNAL_H */