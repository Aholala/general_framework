/**
 * @file alg_imu_ekf_update.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 更新实现（预测 + 校正）
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 包含过程噪声计算、零偏渐消、创新统计计算、
 *       加速度计校正和完整更新流程。
 */

#include "alg_imu_ekf_internal.h"

#include <math.h>   // sqrtf, fabsf, isfinite
#include <stddef.h> // NULL

/** @brief 最小模长阈值，用于避免除零 */
#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)
/** @brief 奇异阈值，用于矩阵求逆判断 */
#define ALG_IMU_EKF_SINGULAR_THRESHOLD (1.0e-12F)

/* ======================== 过程噪声计算 ======================== */

/**
 * @brief 更新过程噪声矩阵
 * @param me EKF 对象
 * @param delta_time_s 时间步长（秒）
 * @note Q = G * Q_gyro * G^T + Q_bias
 *       其中 G 是四元数对陀螺仪噪声的映射矩阵
 */
static void alg_imu_ekf_update_process_noise(alg_imu_ekf_t *me, float delta_time_s)
{
    float gyro_mapping[4U * 3U]; // 四元数对陀螺仪噪声的雅可比
    float gyro_variance_factor;  // 陀螺仪噪声方差因子
    float bias_variance;         // 零偏随机游走方差
    size_t row;
    size_t column;
    size_t axis;
    float accumulator;

    // ---- 清空过程噪声矩阵 ----
    for (row = 0U; row < (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION); ++row)
        me->process_noise[row] = 0.0F;

    // ---- 构建四元数对陀螺仪噪声的映射矩阵 G ----
    // G 将 3 维陀螺仪噪声映射到 4 维四元数状态
    // 每列对应一个轴的噪声，每行对应四元数一个分量
    gyro_mapping[0] = -me->state[1]; // 列0（X 轴噪声）→ q0
    gyro_mapping[1] = -me->state[2]; // 列0 → q1
    gyro_mapping[2] = -me->state[3]; // 列0 → q2
    gyro_mapping[3] = me->state[0];  // 列0 → q3

    gyro_mapping[4] = -me->state[3]; // 列1（Y 轴噪声）→ q0
    gyro_mapping[5] = me->state[2];  // 列1 → q1
    gyro_mapping[6] = me->state[0];  // 列1 → q2
    gyro_mapping[7] = -me->state[1]; // 列1 → q3

    gyro_mapping[8] = me->state[2];  // 列2（Z 轴噪声）→ q0
    gyro_mapping[9] = -me->state[1]; // 列2 → q1
    gyro_mapping[10] = me->state[0]; // 列2 → q2
    gyro_mapping[11] = me->state[3]; // 列2 → q3

    // ---- 计算陀螺仪噪声对四元数协方差的贡献 ----
    // Q_quat = G * Q_gyro * G^T * (0.5 * dt)^2
    gyro_variance_factor = 0.25F * me->config.gyro_noise_std_rad_s *
                           me->config.gyro_noise_std_rad_s * delta_time_s * delta_time_s;

    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            accumulator = 0.0F;
            // 对三个轴累加
            for (axis = 0U; axis < 3U; ++axis)
            {
                accumulator += gyro_mapping[(row * 3U) + axis] * gyro_mapping[(column * 3U) + axis];
            }
            me->process_noise[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                gyro_variance_factor * accumulator;
        }
    }

    // ---- 零偏随机游走噪声（仅 X/Y 轴） ----
    bias_variance = me->config.gyro_bias_random_walk_std_rad_s2 *
                    me->config.gyro_bias_random_walk_std_rad_s2 * delta_time_s * delta_time_s;
    me->process_noise[(4U * ALG_IMU_EKF_STATE_DIMENSION) + 4U] = bias_variance;
    me->process_noise[(5U * ALG_IMU_EKF_STATE_DIMENSION) + 5U] = bias_variance;

    // Z 轴零偏无过程噪声（不估计）
}

/* ======================== 零偏协方差渐消 ======================== */

/**
 * @brief 应用零偏协方差渐消因子
 * @param me EKF 对象
 * @note 每次预测时对 X/Y 零偏相关协方差乘以渐消因子
 *       防止零偏估计过于自信，保持对慢漂移的跟踪能力
 */
static void alg_imu_ekf_apply_bias_fading(alg_imu_ekf_t *me)
{
    const float bias_scale = sqrtf(me->config.gyro_bias_fading_factor);
    size_t row;
    size_t column;
    float scale;

    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            scale = 1.0F;
            // 若行或列涉及零偏状态（索引 4~5），应用渐消因子
            if (row >= 4U)
                scale *= bias_scale;
            if (column >= 4U)
                scale *= bias_scale;
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] *= scale;
        }
    }
}

/* ======================== 3×3 对称矩阵求逆 ======================== */

/**
 * @brief 求 3×3 对称矩阵的逆
 * @param matrix 输入矩阵（3×3，行优先）
 * @param inverse 输出逆矩阵（3×3）
 * @return true=成功，false=奇异
 */
static bool alg_imu_ekf_invert_symmetric3x3(const float matrix[9], float inverse[9])
{
    // 计算余子式
    const float cofactor_00 = (matrix[4] * matrix[8]) - (matrix[5] * matrix[7]);
    const float cofactor_01 = (matrix[5] * matrix[6]) - (matrix[3] * matrix[8]);
    const float cofactor_02 = (matrix[3] * matrix[7]) - (matrix[4] * matrix[6]);

    // 行列式 = a00*C00 + a01*C01 + a02*C02
    const float determinant =
        (matrix[0] * cofactor_00) + (matrix[1] * cofactor_01) + (matrix[2] * cofactor_02);

    if (!isfinite(determinant) || (fabsf(determinant) <= ALG_IMU_EKF_SINGULAR_THRESHOLD))
        return false;

    // 使用伴随矩阵法求逆
    inverse[0] = cofactor_00 / determinant;
    inverse[1] = ((matrix[2] * matrix[7]) - (matrix[1] * matrix[8])) / determinant;
    inverse[2] = ((matrix[1] * matrix[5]) - (matrix[2] * matrix[4])) / determinant;
    inverse[3] = cofactor_01 / determinant;
    inverse[4] = ((matrix[0] * matrix[8]) - (matrix[2] * matrix[6])) / determinant;
    inverse[5] = ((matrix[2] * matrix[3]) - (matrix[0] * matrix[5])) / determinant;
    inverse[6] = cofactor_02 / determinant;
    inverse[7] = ((matrix[1] * matrix[6]) - (matrix[0] * matrix[7])) / determinant;
    inverse[8] = ((matrix[0] * matrix[4]) - (matrix[1] * matrix[3])) / determinant;

    return alg_imu_ekf_internal_is_finite_array(inverse, 9U);
}

/* ======================== 创新统计计算 ======================== */

/**
 * @brief 计算创新统计量（NIS 和自适应噪声倍率）
 * @param me EKF 对象
 * @param normalized_measurement 归一化后的加速度测量
 * @param base_measurement_variance 基础测量噪声方差
 * @return 执行状态
 * @note 计算 NIS = innovation^T * S^-1 * innovation
 *       用于判断观测是否一致（卡方检验）
 */
static alg_imu_ekf_status_t
alg_imu_ekf_compute_innovation_statistics(alg_imu_ekf_t *me, const float normalized_measurement[3],
                                          float base_measurement_variance)
{
    float *predicted_measurement = me->innovation_workspace;            // 预测测量（3 维）
    float *measurement_jacobian = predicted_measurement + 3U;           // 测量雅可比（3×6 = 18）
    float *measurement_covariance_product = measurement_jacobian + 18U; // H*P（3×6 = 18）
    float *innovation_covariance =
        measurement_covariance_product + 18U;                          // S = H*P*H^T + R（3×3 = 9）
    float *innovation_covariance_inverse = innovation_covariance + 9U; // S^-1（3×3 = 9）
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;
    float transformed_innovation[3];
    alg_kalman_status_t kalman_status;

    // ---- 1. 计算预测测量 h(x) ----
    kalman_status = alg_imu_ekf_internal_measurement_function(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        predicted_measurement, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;

    // ---- 2. 计算测量雅可比 H ----
    kalman_status = alg_imu_ekf_internal_measurement_jacobian(
        me->state, ALG_IMU_EKF_STATE_DIMENSION, ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        measurement_jacobian, me);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;

    // ---- 3. 计算创新残差 y = z - h(x) ----
    for (row = 0U; row < 3U; ++row)
        me->innovation[row] = normalized_measurement[row] - predicted_measurement[row];

    // ---- 4. 计算 H*P ----
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    me->covariance[(shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                accumulator;
        }
    }

    // ---- 5. 计算创新协方差 S = H*P*H^T + R ----
    for (row = 0U; row < 3U; ++row)
    {
        for (column = 0U; column < 3U; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    measurement_covariance_product[(row * ALG_IMU_EKF_STATE_DIMENSION) +
                                                   shared_index] *
                    measurement_jacobian[(column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            innovation_covariance[(row * 3U) + column] =
                accumulator + ((row == column) ? base_measurement_variance : 0.0F);
        }
    }

    // ---- 6. 求创新协方差逆矩阵 ----
    if (!alg_imu_ekf_invert_symmetric3x3(innovation_covariance, innovation_covariance_inverse))
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;

    // ---- 7. 计算 NIS = y^T * S^-1 * y ----
    // 先计算 S^-1 * y
    for (row = 0U; row < 3U; ++row)
    {
        transformed_innovation[row] =
            (innovation_covariance_inverse[(row * 3U)] * me->innovation[0]) +
            (innovation_covariance_inverse[(row * 3U) + 1U] * me->innovation[1]) +
            (innovation_covariance_inverse[(row * 3U) + 2U] * me->innovation[2]);
    }

    // 再计算 y^T * (S^-1 * y)
    me->last_normalized_innovation_squared = (me->innovation[0] * transformed_innovation[0]) +
                                             (me->innovation[1] * transformed_innovation[1]) +
                                             (me->innovation[2] * transformed_innovation[2]);

    return isfinite(me->last_normalized_innovation_squared) ? ALG_IMU_EKF_STATUS_OK
                                                            : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

/* ======================== EKF 预测 ======================== */

/**
 * @brief EKF 预测步骤（仅陀螺仪积分）
 * @param me EKF 对象
 * @param gyroscope_rad_s 陀螺仪读数（rad/s），长度 3
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_predict(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                         float delta_time_s)
{
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (gyroscope_rad_s == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    if (!alg_imu_ekf_internal_is_finite_array(gyroscope_rad_s, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;

    // ---- 1. 零偏协方差渐消 ----
    alg_imu_ekf_apply_bias_fading(me);

    // ---- 2. 更新过程噪声 ----
    alg_imu_ekf_update_process_noise(me, delta_time_s);

    // ---- 3. 执行通用 EKF 预测 ----
    kalman_status = alg_kalman_extended_predict(&me->kalman, gyroscope_rad_s, delta_time_s);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);

    // ---- 4. 四元数归一化与协方差投影 ----
    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = false;

    return status;
}

/* ======================== 加速度计校正 ======================== */

/**
 * @brief EKF 校正步骤（加速度计观测）
 * @param me EKF 对象
 * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 观测被拒绝时返回 ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED
 */
alg_imu_ekf_status_t alg_imu_ekf_correct_accelerometer(alg_imu_ekf_t *me,
                                                       const float accelerometer_m_s2[3],
                                                       float delta_time_s)
{
    float raw_norm;
    float filtered_norm;
    float relative_deviation;
    float normalized_measurement[3];
    float base_measurement_variance;
    float adaptation_ratio;
    float noise_scale;
    size_t index;
    alg_filter_status_t filter_status;
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;

    // ---- 1. 模长检查：硬拒绝 ----
    raw_norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                     (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                     (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(raw_norm) || (raw_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    // 计算相对 1g 的偏差
    relative_deviation = fabsf(raw_norm - me->config.gravity_m_s2) / me->config.gravity_m_s2;
    me->last_accelerometer_norm_m_s2 = raw_norm;
    me->last_accelerometer_deviation_g = relative_deviation;

    if (relative_deviation > me->config.accelerometer_rejection_threshold_g)
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    // ---- 2. 低通滤波 ----
    for (index = 0U; index < 3U; ++index)
    {
        filter_status =
            alg_filter_low_pass_update(&me->accelerometer_filter[index], accelerometer_m_s2[index],
                                       delta_time_s, &me->filtered_accelerometer_m_s2[index]);
        if (filter_status != ALG_FILTER_STATUS_OK)
            return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    }

    // ---- 3. 归一化滤波后加速度（仅使用方向） ----
    filtered_norm =
        sqrtf((me->filtered_accelerometer_m_s2[0] * me->filtered_accelerometer_m_s2[0]) +
              (me->filtered_accelerometer_m_s2[1] * me->filtered_accelerometer_m_s2[1]) +
              (me->filtered_accelerometer_m_s2[2] * me->filtered_accelerometer_m_s2[2]));
    if (!isfinite(filtered_norm) || (filtered_norm <= ALG_IMU_EKF_MINIMUM_NORM))
    {
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    for (index = 0U; index < 3U; ++index)
        normalized_measurement[index] = me->filtered_accelerometer_m_s2[index] / filtered_norm;

    // ---- 4. 计算创新统计（NIS） ----
    base_measurement_variance =
        me->config.accelerometer_direction_noise_std * me->config.accelerometer_direction_noise_std;

    status = alg_imu_ekf_compute_innovation_statistics(me, normalized_measurement,
                                                       base_measurement_variance);
    if (status != ALG_IMU_EKF_STATUS_OK)
        return status;

    // ---- 5. 卡方检验：NIS 超过拒绝阈值 ----
    if (me->last_normalized_innovation_squared > me->config.chi_square_rejection_threshold)
    {
        me->last_measurement_noise_scale = me->config.maximum_measurement_noise_scale;
        me->was_accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED;
    }

    // ---- 6. 自适应噪声倍率 ----
    noise_scale = 1.0F;
    if (me->last_normalized_innovation_squared > me->config.chi_square_adaptation_threshold)
    {
        // NIS 在自适应区间内：线性增加噪声倍率
        adaptation_ratio =
            (me->last_normalized_innovation_squared - me->config.chi_square_adaptation_threshold) /
            (me->config.chi_square_rejection_threshold -
             me->config.chi_square_adaptation_threshold);
        noise_scale += (me->config.maximum_measurement_noise_scale - 1.0F) * adaptation_ratio *
                       adaptation_ratio;
    }
    me->last_measurement_noise_scale = noise_scale;

    // ---- 7. 更新测量噪声矩阵 ----
    for (index = 0U; index < 9U; ++index)
        me->measurement_noise[index] = 0.0F;
    for (index = 0U; index < 3U; ++index)
        me->measurement_noise[(index * 3U) + index] = base_measurement_variance * noise_scale;

    // ---- 8. 执行 EKF 校正 ----
    kalman_status = alg_kalman_extended_correct(&me->kalman, normalized_measurement);
    if (kalman_status != ALG_KALMAN_STATUS_OK)
    {
        me->was_accelerometer_used = false;
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
    }

    // ---- 9. 归一化与投影 ----
    status = alg_imu_ekf_internal_normalize_and_project(me);
    me->was_accelerometer_used = (status == ALG_IMU_EKF_STATUS_OK);
    return status;
}

/* ======================== 完整更新 ======================== */

/**
 * @brief 完整 EKF 更新（预测 + 校正）
 * @param me EKF 对象
 * @param gyroscope_rad_s 陀螺仪读数
 * @param accelerometer_m_s2 加速度计读数
 * @param delta_time_s 时间步长
 * @param accelerometer_used 输出是否使用加速度观测
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_update(alg_imu_ekf_t *me, const float gyroscope_rad_s[3],
                                        const float accelerometer_m_s2[3], float delta_time_s,
                                        bool *accelerometer_used)
{
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (gyroscope_rad_s == NULL) || (accelerometer_m_s2 == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;

    // ---- 1. 预测 ----
    status = alg_imu_ekf_predict(me, gyroscope_rad_s, delta_time_s);
    if (status != ALG_IMU_EKF_STATUS_OK)
        return status;

    // ---- 2. 校正 ----
    status = alg_imu_ekf_correct_accelerometer(me, accelerometer_m_s2, delta_time_s);

    // 加速度被拒绝时返回 OK（预测结果仍然有效）
    if (status == ALG_IMU_EKF_STATUS_ACCELEROMETER_REJECTED)
    {
        if (accelerometer_used != NULL)
            *accelerometer_used = false;
        return ALG_IMU_EKF_STATUS_OK;
    }

    if (status != ALG_IMU_EKF_STATUS_OK)
        return status;

    if (accelerometer_used != NULL)
        *accelerometer_used = true;

    return ALG_IMU_EKF_STATUS_OK;
}