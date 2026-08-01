/**
 * @file alg_imu_ekf_model.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 模型函数实现
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 包含状态转移函数、状态雅可比、测量函数和测量雅可比。
 *       这些函数被通用 EKF 框架调用，用于预测和校正。
 */

#include "alg_imu_ekf_internal.h"

#include <math.h>   // sqrtf, isfinite
#include <stddef.h> // NULL

/* ======================== 状态转移函数 ======================== */

/**
 * @brief EKF 状态转移函数（四元数积分 + 零偏保持）
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param control_input 控制输入（陀螺仪）
 * @param control_dimension 控制维度
 * @param delta_time_s 时间步长
 * @param predicted_state 输出预测状态
 * @param user_context 用户上下文（未使用）
 * @return 卡尔曼状态码
 * @note 使用一阶四元数积分：q' = q + 0.5 * dt * q ⊗ ω
 *       X/Y 零偏保持不变（随机游走在过程噪声中体现）
 */
alg_kalman_status_t alg_imu_ekf_internal_state_function(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *predicted_state,
                                                        void *user_context)
{
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float quaternion_norm;

    (void)user_context; // 本函数不需要用户上下文

    // ---- 参数校验 ----
    if ((state == NULL) || (control_input == NULL) || (predicted_state == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (control_dimension != ALG_IMU_EKF_CONTROL_DIMENSION) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F)) {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
}

    // ---- 去除 X/Y 零偏 ----
    angular_rate_x = control_input[0] - state[4];
    angular_rate_y = control_input[1] - state[5];
    // Z 轴不做零偏估计（六轴 IMU 无法观测）
    angular_rate_z = control_input[2];

    // ---- 四元数一阶积分 ----
    // q' = q + 0.5 * dt * q ⊗ ω
    predicted_state[0] =
        state[0] -
        (0.5F * delta_time_s *
         ((state[1] * angular_rate_x) + (state[2] * angular_rate_y) + (state[3] * angular_rate_z)));
    predicted_state[1] =
        state[1] +
        (0.5F * delta_time_s *
         ((state[0] * angular_rate_x) + (state[2] * angular_rate_z) - (state[3] * angular_rate_y)));
    predicted_state[2] =
        state[2] +
        (0.5F * delta_time_s *
         ((state[0] * angular_rate_y) - (state[1] * angular_rate_z) + (state[3] * angular_rate_x)));
    predicted_state[3] =
        state[3] +
        (0.5F * delta_time_s *
         ((state[0] * angular_rate_z) + (state[1] * angular_rate_y) - (state[2] * angular_rate_x)));

    // ---- 零偏保持不变 ----
    predicted_state[4] = state[4];
    predicted_state[5] = state[5];

    // ---- 归一化四元数（预测后立即归一化） ----
    quaternion_norm = sqrtf(
        (predicted_state[0] * predicted_state[0]) + (predicted_state[1] * predicted_state[1]) +
        (predicted_state[2] * predicted_state[2]) + (predicted_state[3] * predicted_state[3]));
    if (!isfinite(quaternion_norm) || (quaternion_norm <= 1.0e-6F)) {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
}

    predicted_state[0] /= quaternion_norm;
    predicted_state[1] /= quaternion_norm;
    predicted_state[2] /= quaternion_norm;
    predicted_state[3] /= quaternion_norm;

    return ALG_KALMAN_STATUS_OK;
}

/* ======================== 状态雅可比矩阵 ======================== */

/**
 * @brief 状态转移雅可比矩阵
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param control_input 控制输入
 * @param control_dimension 控制维度
 * @param delta_time_s 时间步长
 * @param state_jacobian 输出雅可比矩阵（6×6）
 * @param user_context 用户上下文（未使用）
 * @return 卡尔曼状态码
 * @note F = ∂(状态转移函数)/∂(状态)
 *       包含四元数对四元数的偏导和四元数对零偏的偏导
 */
alg_kalman_status_t alg_imu_ekf_internal_state_jacobian(const float *state, size_t state_dimension,
                                                        const float *control_input,
                                                        size_t control_dimension,
                                                        float delta_time_s, float *state_jacobian,
                                                        void *user_context)
{
    float angular_rate_x;
    float angular_rate_y;
    float angular_rate_z;
    float half_delta_time;
    size_t index;

    (void)user_context;

    if ((state == NULL) || (control_input == NULL) || (state_jacobian == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (control_dimension != ALG_IMU_EKF_CONTROL_DIMENSION) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F)) {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
}

    angular_rate_x = control_input[0] - state[4];
    angular_rate_y = control_input[1] - state[5];
    angular_rate_z = control_input[2];
    half_delta_time = 0.5F * delta_time_s;

    // ---- 清零雅可比矩阵 ----
    for (index = 0U; index < (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION); ++index) {
        state_jacobian[index] = 0.0F;
}

    // ---- 使用宏简化矩阵填充 ----
#define F(row, column) state_jacobian[((row) * ALG_IMU_EKF_STATE_DIMENSION) + (column)]

    // 四元数对四元数偏导（左上 4×4 块）
    F(0U, 0U) = 1.0F;
    F(0U, 1U) = -half_delta_time * angular_rate_x;
    F(0U, 2U) = -half_delta_time * angular_rate_y;
    F(0U, 3U) = -half_delta_time * angular_rate_z;

    F(1U, 0U) = half_delta_time * angular_rate_x;
    F(1U, 1U) = 1.0F;
    F(1U, 2U) = half_delta_time * angular_rate_z;
    F(1U, 3U) = -half_delta_time * angular_rate_y;

    F(2U, 0U) = half_delta_time * angular_rate_y;
    F(2U, 1U) = -half_delta_time * angular_rate_z;
    F(2U, 2U) = 1.0F;
    F(2U, 3U) = half_delta_time * angular_rate_x;

    F(3U, 0U) = half_delta_time * angular_rate_z;
    F(3U, 1U) = half_delta_time * angular_rate_y;
    F(3U, 2U) = -half_delta_time * angular_rate_x;
    F(3U, 3U) = 1.0F;

    // 四元数对零偏偏导（右上 4×2 块）
    // 零偏变化会影响角速度，进而影响四元数
    F(0U, 4U) = half_delta_time * state[1];
    F(0U, 5U) = half_delta_time * state[2];
    F(1U, 4U) = -half_delta_time * state[0];
    F(1U, 5U) = half_delta_time * state[3];
    F(2U, 4U) = -half_delta_time * state[3];
    F(2U, 5U) = -half_delta_time * state[0];
    F(3U, 4U) = half_delta_time * state[2];
    F(3U, 5U) = -half_delta_time * state[1];

    // 零偏对零偏偏导（右下 2×2 块为单位矩阵）
    F(4U, 4U) = 1.0F;
    F(5U, 5U) = 1.0F;

#undef F

    return ALG_KALMAN_STATUS_OK;
}

/* ======================== 测量函数 ======================== */

/**
 * @brief 测量函数（从状态预测加速度方向）
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param measurement_dimension 测量维度
 * @param predicted_measurement 输出预测测量
 * @param user_context 用户上下文（未使用）
 * @return 卡尔曼状态码
 * @note 将重力向量 [0, 0, g] 从世界系旋转到机体系
 *       即预测加速度方向（归一化后）
 */
alg_kalman_status_t alg_imu_ekf_internal_measurement_function(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *predicted_measurement,
                                                              void *user_context)
{
    (void)user_context;

    if ((state == NULL) || (predicted_measurement == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (measurement_dimension != ALG_IMU_EKF_MEASUREMENT_DIMENSION)) {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
}

    // 重力在世界系中为 [0, 0, 1]（归一化后）
    // 旋转到机体系：g_body = R^T * [0, 0, 1]
    predicted_measurement[0] = 2.0F * ((state[1] * state[3]) - (state[0] * state[2]));
    predicted_measurement[1] = 2.0F * ((state[0] * state[1]) + (state[2] * state[3]));
    predicted_measurement[2] = (state[0] * state[0]) - (state[1] * state[1]) -
                               (state[2] * state[2]) + (state[3] * state[3]);

    return ALG_KALMAN_STATUS_OK;
}

/* ======================== 测量雅可比矩阵 ======================== */

/**
 * @brief 测量雅可比矩阵
 * @param state 当前状态
 * @param state_dimension 状态维度
 * @param measurement_dimension 测量维度
 * @param measurement_jacobian 输出雅可比矩阵（3×6）
 * @param user_context 用户上下文（未使用）
 * @return 卡尔曼状态码
 * @note H = ∂(测量函数)/∂(状态)
 *       仅四元数部分有非零偏导，零偏部分为 0
 */
alg_kalman_status_t alg_imu_ekf_internal_measurement_jacobian(const float *state,
                                                              size_t state_dimension,
                                                              size_t measurement_dimension,
                                                              float *measurement_jacobian,
                                                              void *user_context)
{
    size_t index;

    (void)user_context;

    if ((state == NULL) || (measurement_jacobian == NULL) ||
        (state_dimension != ALG_IMU_EKF_STATE_DIMENSION) ||
        (measurement_dimension != ALG_IMU_EKF_MEASUREMENT_DIMENSION)) {
        return ALG_KALMAN_STATUS_MODEL_ERROR;
}

    // ---- 清零雅可比矩阵 ----
    for (index = 0U; index < (ALG_IMU_EKF_MEASUREMENT_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
         ++index) {
        measurement_jacobian[index] = 0.0F;
}

    // ---- 使用宏简化矩阵填充 ----
#define H(row, column) measurement_jacobian[((row) * ALG_IMU_EKF_STATE_DIMENSION) + (column)]

    // 第一行：∂h0/∂q (X 分量)
    H(0U, 0U) = -2.0F * state[2];
    H(0U, 1U) = 2.0F * state[3];
    H(0U, 2U) = -2.0F * state[0];
    H(0U, 3U) = 2.0F * state[1];

    // 第二行：∂h1/∂q (Y 分量)
    H(1U, 0U) = 2.0F * state[1];
    H(1U, 1U) = 2.0F * state[0];
    H(1U, 2U) = 2.0F * state[3];
    H(1U, 3U) = 2.0F * state[2];

    // 第三行：∂h2/∂q (Z 分量)
    H(2U, 0U) = 2.0F * state[0];
    H(2U, 1U) = -2.0F * state[1];
    H(2U, 2U) = -2.0F * state[2];
    H(2U, 3U) = 2.0F * state[3];

    // 零偏部分（第 4~5 列）全为 0
#undef H

    return ALG_KALMAN_STATUS_OK;
}