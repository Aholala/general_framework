/**
 * @file alg_imu_ekf_core.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 核心实现（初始化、重置、归一化）
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 包含对象初始化、配置默认值、四元数归一化与协方差投影、
 *       从加速度计重置姿态等核心功能。
 */

#include "alg_imu_ekf_internal.h"

#include <math.h>   // sqrtf, atan2f, sinf, cosf, isfinite
#include <stddef.h> // NULL

/** @brief 标准重力加速度（m/s²） */
#define ALG_IMU_EKF_STANDARD_GRAVITY_M_S2 (9.80665F)
/** @brief 最小模长阈值，用于避免除零 */
#define ALG_IMU_EKF_MINIMUM_NORM (1.0e-6F)

/* ======================== 内部工具函数 ======================== */

/**
 * @brief 检查数组元素是否全部为有限数
 * @param values 数组指针
 * @param value_count 元素个数
 * @return true=全有限
 */
bool alg_imu_ekf_internal_is_finite_array(const float *values, size_t value_count)
{
    size_t index;

    if (values == NULL) {
        return false;
}

    for (index = 0U; index < value_count; ++index)
    {
        if (!isfinite(values[index])) {
            return false;
}
    }
    return true;
}

/**
 * @brief 将卡尔曼状态码映射到 IMU EKF 状态码
 * @param status 卡尔曼状态码
 * @return IMU EKF 状态码
 */
alg_imu_ekf_status_t alg_imu_ekf_internal_map_kalman_status(alg_kalman_status_t status)
{
    switch (status)
    {
    case ALG_KALMAN_STATUS_OK:
        return ALG_IMU_EKF_STATUS_OK;
    case ALG_KALMAN_STATUS_INVALID_ARGUMENT:
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    case ALG_KALMAN_STATUS_OUT_OF_RANGE:
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    case ALG_KALMAN_STATUS_NOT_INITIALIZED:
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    case ALG_KALMAN_STATUS_NUMERICAL_ERROR:
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
    default:
        return ALG_IMU_EKF_STATUS_KALMAN_ERROR;
    }
}

/**
 * @brief 清空数组（全部置零）
 * @param values 数组指针
 * @param value_count 元素个数
 */
static void alg_imu_ekf_clear(float *values, size_t value_count)
{
    size_t index;
    for (index = 0U; index < value_count; ++index) {
        values[index] = 0.0F;
}
}

/**
 * @brief 重置协方差矩阵为初始值
 * @param me EKF 对象
 * @note 四元数部分方差为 initial_attitude_variance
 *       零偏部分方差为 initial_gyro_bias_variance
 */
static void alg_imu_ekf_reset_covariance(alg_imu_ekf_t *me)
{
    size_t index;

    // 清空协方差矩阵
    alg_imu_ekf_clear(me->covariance, ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);

    // 四元数部分（索引 0~3）
    for (index = 0U; index < 4U; ++index) {
        me->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            me->config.initial_attitude_variance;
}

    // 零偏部分（索引 4~5）
    for (index = 4U; index < ALG_IMU_EKF_STATE_DIMENSION; ++index) {
        me->covariance[(index * ALG_IMU_EKF_STATE_DIMENSION) + index] =
            me->config.initial_gyro_bias_variance;
}
}

/* ======================== 四元数归一化与协方差投影 ======================== */

/**
 * @brief 四元数归一化及协方差投影
 * @param me EKF 对象
 * @return 执行状态
 * @note 保持四元数单位长度，并将协方差投影到单位四元数约束流形上
 *       这是 EKF 的关键步骤，防止四元数数值漂移
 */
alg_imu_ekf_status_t alg_imu_ekf_internal_normalize_and_project(alg_imu_ekf_t *me)
{
    float quaternion_norm;
    float normalized_quaternion[4];
    float *normalization_jacobian; // 雅可比矩阵：J = ∂(归一化四元数)/∂(四元数)
    float *temporary_covariance;   // 临时协方差存储
    size_t row;
    size_t column;
    size_t shared_index;
    float accumulator;

    // ---- 1. 计算四元数模长并归一化 ----
    quaternion_norm = sqrtf((me->state[0] * me->state[0]) + (me->state[1] * me->state[1]) +
                            (me->state[2] * me->state[2]) + (me->state[3] * me->state[3]));
    if (!isfinite(quaternion_norm) || (quaternion_norm <= ALG_IMU_EKF_MINIMUM_NORM)) {
        return ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

    // 归一化四元数
    for (row = 0U; row < 4U; ++row)
    {
        normalized_quaternion[row] = me->state[row] / quaternion_norm;
        me->state[row] = normalized_quaternion[row];
    }

    // ---- 2. 计算归一化雅可比矩阵 ----
    // J = (I - q * q^T) / ||q||
    normalization_jacobian = me->normalization_workspace;
    temporary_covariance =
        normalization_jacobian + (ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);

    alg_imu_ekf_clear(normalization_jacobian,
                      ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);

    // 四元数部分（左上 4×4 块）
    for (row = 0U; row < 4U; ++row)
    {
        for (column = 0U; column < 4U; ++column)
        {
            normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] =
                (((row == column) ? 1.0F : 0.0F) -
                 (normalized_quaternion[row] * normalized_quaternion[column])) /
                quaternion_norm;
        }
    }

    // 零偏部分（右下 2×2 块为单位矩阵）
    for (row = 4U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row) {
        normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + row] = 1.0F;
}

    // ---- 3. 协方差投影：P' = J * P * J^T ----
    // 计算 J * P
    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    normalization_jacobian[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    me->covariance[(shared_index * ALG_IMU_EKF_STATE_DIMENSION) + column];
            }
            temporary_covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = accumulator;
        }
    }

    // 计算 (J*P) * J^T
    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = 0U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            accumulator = 0.0F;
            for (shared_index = 0U; shared_index < ALG_IMU_EKF_STATE_DIMENSION; ++shared_index)
            {
                accumulator +=
                    temporary_covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + shared_index] *
                    normalization_jacobian[(column * ALG_IMU_EKF_STATE_DIMENSION) + shared_index];
            }
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = accumulator;
        }
    }

    // ---- 4. 对称化协方差矩阵（消除数值误差） ----
    for (row = 0U; row < ALG_IMU_EKF_STATE_DIMENSION; ++row)
    {
        for (column = row + 1U; column < ALG_IMU_EKF_STATE_DIMENSION; ++column)
        {
            const float average =
                0.5F * (me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] +
                        me->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row]);
            me->covariance[(row * ALG_IMU_EKF_STATE_DIMENSION) + column] = average;
            me->covariance[(column * ALG_IMU_EKF_STATE_DIMENSION) + row] = average;
        }
    }

    return alg_imu_ekf_internal_is_finite_array(me->covariance, ALG_IMU_EKF_STATE_DIMENSION *
                                                                    ALG_IMU_EKF_STATE_DIMENSION)
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

/* ======================== 配置默认值 ======================== */

/**
 * @brief 初始化配置为默认值
 * @param config 配置结构体指针
 * @return 执行状态
 * @note 默认值适用于大多数消费级 IMU
 */
alg_imu_ekf_status_t alg_imu_ekf_config_init(alg_imu_ekf_config_t *config)
{
    if (config == NULL) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}

    *config =
        (alg_imu_ekf_config_t){.gravity_m_s2 = ALG_IMU_EKF_STANDARD_GRAVITY_M_S2,
                               .gyro_noise_std_rad_s = 0.015F,              // 约 0.86°/s 噪声
                               .gyro_bias_random_walk_std_rad_s2 = 0.0005F, // 慢漂移
                               .accelerometer_direction_noise_std = 0.03F,  // 约 1.7° 方向噪声
                               .accelerometer_lpf_cutoff_hz = 30.0F,
                               .accelerometer_rejection_threshold_g = 0.20F, // 20% 模长偏差
                               .chi_square_adaptation_threshold = 3.0F,
                               .chi_square_rejection_threshold = 11.345F, // 3 自由度 99% 分位点
                               .maximum_measurement_noise_scale = 20.0F,
                               .gyro_bias_fading_factor = 1.0001F,
                               .initial_attitude_variance = 0.10F,
                               .initial_gyro_bias_variance = 0.01F};
    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 验证配置参数有效性
 * @param config 配置结构体指针
 * @return 执行状态
 */
static alg_imu_ekf_status_t alg_imu_ekf_validate_config(const alg_imu_ekf_config_t *config)
{
    if (config == NULL) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}

    // 检查所有参数为有限正数且满足约束
    if (!isfinite(config->gravity_m_s2) || (config->gravity_m_s2 <= 0.0F) ||
        !isfinite(config->gyro_noise_std_rad_s) || (config->gyro_noise_std_rad_s < 0.0F) ||
        !isfinite(config->gyro_bias_random_walk_std_rad_s2) ||
        (config->gyro_bias_random_walk_std_rad_s2 < 0.0F) ||
        !isfinite(config->accelerometer_direction_noise_std) ||
        (config->accelerometer_direction_noise_std <= 0.0F) ||
        !isfinite(config->accelerometer_lpf_cutoff_hz) ||
        (config->accelerometer_lpf_cutoff_hz <= 0.0F) ||
        !isfinite(config->accelerometer_rejection_threshold_g) ||
        (config->accelerometer_rejection_threshold_g <= 0.0F) ||
        !isfinite(config->chi_square_adaptation_threshold) ||
        (config->chi_square_adaptation_threshold < 0.0F) ||
        !isfinite(config->chi_square_rejection_threshold) ||
        (config->chi_square_rejection_threshold <= config->chi_square_adaptation_threshold) ||
        !isfinite(config->maximum_measurement_noise_scale) ||
        (config->maximum_measurement_noise_scale < 1.0F) ||
        !isfinite(config->gyro_bias_fading_factor) || (config->gyro_bias_fading_factor < 1.0F) ||
        !isfinite(config->initial_attitude_variance) ||
        (config->initial_attitude_variance < 0.0F) ||
        !isfinite(config->initial_gyro_bias_variance) ||
        (config->initial_gyro_bias_variance < 0.0F))
    {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
    }
    return ALG_IMU_EKF_STATUS_OK;
}

/* ======================== EKF 初始化 ======================== */

/**
 * @brief 初始化 IMU EKF
 * @param me EKF 对象
 * @param config 配置参数
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_init(alg_imu_ekf_t *me, const alg_imu_ekf_config_t *config)
{
    alg_kalman_extended_config_t kalman_config;
    alg_kalman_status_t kalman_status;
    alg_imu_ekf_status_t status;
    size_t index;
    float accelerometer_variance;
    alg_filter_status_t filter_status;

    if (me == NULL) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}

    // 先标记为未初始化
    me->is_initialized = false;

    // 验证配置
    status = alg_imu_ekf_validate_config(config);
    if (status != ALG_IMU_EKF_STATUS_OK) {
        return status;
}

    // 保存配置
    me->config = *config;

    // ---- 初始化状态（默认单位四元数） ----
    alg_imu_ekf_clear(me->state, ALG_IMU_EKF_STATE_DIMENSION);
    me->state[0] = 1.0F;

    // ---- 初始化协方差 ----
    alg_imu_ekf_reset_covariance(me);

    // ---- 初始化过程噪声和测量噪声 ----
    alg_imu_ekf_clear(me->process_noise, ALG_IMU_EKF_STATE_DIMENSION * ALG_IMU_EKF_STATE_DIMENSION);
    alg_imu_ekf_clear(me->measurement_noise,
                      ALG_IMU_EKF_MEASUREMENT_DIMENSION * ALG_IMU_EKF_MEASUREMENT_DIMENSION);

    // 测量噪声：加速度计方向噪声方差
    accelerometer_variance =
        config->accelerometer_direction_noise_std * config->accelerometer_direction_noise_std;
    for (index = 0U; index < ALG_IMU_EKF_MEASUREMENT_DIMENSION; ++index) {
        me->measurement_noise[(index * ALG_IMU_EKF_MEASUREMENT_DIMENSION) + index] =
            accelerometer_variance;
}

    // ---- 初始化加速度计低通滤波器 ----
    for (index = 0U; index < 3U; ++index)
    {
        filter_status = alg_filter_low_pass_init(&me->accelerometer_filter[index],
                                                 config->accelerometer_lpf_cutoff_hz);
        if (filter_status != ALG_FILTER_STATUS_OK) {
            return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}
        me->filtered_accelerometer_m_s2[index] = 0.0F;
        me->innovation[index] = 0.0F;
    }

    // ---- 初始化通用 EKF ----
    kalman_config = (alg_kalman_extended_config_t){
        .state_dimension = ALG_IMU_EKF_STATE_DIMENSION,
        .measurement_dimension = ALG_IMU_EKF_MEASUREMENT_DIMENSION,
        .control_dimension = ALG_IMU_EKF_CONTROL_DIMENSION,
        .state = me->state,
        .covariance = me->covariance,
        .process_noise = me->process_noise,
        .measurement_noise = me->measurement_noise,
        .workspace = me->kalman_workspace,
        .workspace_size = sizeof(me->kalman_workspace) / sizeof(me->kalman_workspace[0]),
        .state_function = alg_imu_ekf_internal_state_function,
        .state_jacobian_function = alg_imu_ekf_internal_state_jacobian,
        .measurement_function = alg_imu_ekf_internal_measurement_function,
        .measurement_jacobian_function = alg_imu_ekf_internal_measurement_jacobian,
        .user_context = me};

    kalman_status = alg_kalman_extended_init(&me->kalman, &kalman_config);
    if (kalman_status != ALG_KALMAN_STATUS_OK) {
        return alg_imu_ekf_internal_map_kalman_status(kalman_status);
}

    // ---- 初始化诊断状态 ----
    me->last_accelerometer_norm_m_s2 = config->gravity_m_s2;
    me->last_accelerometer_deviation_g = 0.0F;
    me->last_normalized_innovation_squared = 0.0F;
    me->last_measurement_noise_scale = 1.0F;
    me->was_accelerometer_used = false;
    me->is_initialized = true;

    // ---- 归一化并投影（确保初始状态有效） ----
    return alg_imu_ekf_internal_normalize_and_project(me);
}

/* ======================== 重置功能 ======================== */

/**
 * @brief 重置 EKF 到指定姿态和零偏
 * @param me EKF 对象
 * @param quaternion 初始四元数
 * @param gyro_bias_rad_s X/Y 轴零偏（rad/s），长度 2
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_reset(alg_imu_ekf_t *me,
                                       const alg_imu_ekf_quaternion_t *quaternion,
                                       const float gyro_bias_rad_s[2])
{
    size_t index;

    if ((me == NULL) || (quaternion == NULL) || (gyro_bias_rad_s == NULL)) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
}
    if (!isfinite(quaternion->w) || !isfinite(quaternion->x) || !isfinite(quaternion->y) ||
        !isfinite(quaternion->z) || !alg_imu_ekf_internal_is_finite_array(gyro_bias_rad_s, 2U)) {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}

    // 设置状态
    me->state[0] = quaternion->w;
    me->state[1] = quaternion->x;
    me->state[2] = quaternion->y;
    me->state[3] = quaternion->z;
    me->state[4] = gyro_bias_rad_s[0];
    me->state[5] = gyro_bias_rad_s[1];

    // 重置协方差
    alg_imu_ekf_reset_covariance(me);

    // 重置低通滤波器
    for (index = 0U; index < 3U; ++index)
    {
        (void)alg_filter_low_pass_init(&me->accelerometer_filter[index],
                                       me->config.accelerometer_lpf_cutoff_hz);
        me->filtered_accelerometer_m_s2[index] = 0.0F;
        me->innovation[index] = 0.0F;
    }

    // 重置诊断
    me->last_accelerometer_norm_m_s2 = me->config.gravity_m_s2;
    me->last_accelerometer_deviation_g = 0.0F;
    me->last_normalized_innovation_squared = 0.0F;
    me->last_measurement_noise_scale = 1.0F;
    me->was_accelerometer_used = false;

    // 归一化并投影
    return alg_imu_ekf_internal_normalize_and_project(me);
}

/**
 * @brief 从加速度计重置姿态（Roll/Pitch 根据重力对齐，Yaw 置 0）
 * @param me EKF 对象
 * @param accelerometer_m_s2 加速度计读数（m/s²），长度 3
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_reset_from_accelerometer(alg_imu_ekf_t *me,
                                                          const float accelerometer_m_s2[3])
{
    float norm;
    float roll;
    float pitch;
    float half_roll;
    float half_pitch;
    alg_imu_ekf_quaternion_t quaternion;
    const float zero_bias[2] = {0.0F, 0.0F};
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL)) {
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
}
    if (!me->is_initialized) {
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
}
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U)) {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}

    // ---- 计算加速度模长 ----
    norm = sqrtf((accelerometer_m_s2[0] * accelerometer_m_s2[0]) +
                 (accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                 (accelerometer_m_s2[2] * accelerometer_m_s2[2]));
    if (!isfinite(norm) || (norm <= ALG_IMU_EKF_MINIMUM_NORM)) {
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;
}

    // ---- 从加速度计算 Roll 和 Pitch ----
    // Roll：绕 X 轴旋转，由 Y 和 Z 分量决定
    roll = atan2f(accelerometer_m_s2[1], accelerometer_m_s2[2]);
    // Pitch：绕 Y 轴旋转，由 X 分量和 YZ 平面分量决定
    pitch = atan2f(-accelerometer_m_s2[0], sqrtf((accelerometer_m_s2[1] * accelerometer_m_s2[1]) +
                                                 (accelerometer_m_s2[2] * accelerometer_m_s2[2])));

    // ---- 将 Roll/Pitch 转换为四元数（Yaw 置 0） ----
    half_roll = 0.5F * roll;
    half_pitch = 0.5F * pitch;
    quaternion.w = cosf(half_roll) * cosf(half_pitch);
    quaternion.x = sinf(half_roll) * cosf(half_pitch);
    quaternion.y = cosf(half_roll) * sinf(half_pitch);
    quaternion.z = -sinf(half_roll) * sinf(half_pitch);

    // ---- 重置 EKF ----
    status = alg_imu_ekf_reset(me, &quaternion, zero_bias);
    if (status != ALG_IMU_EKF_STATUS_OK) {
        return status;
}

    // ---- 预置低通滤波器状态 ----
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[0], accelerometer_m_s2[0]);
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[1], accelerometer_m_s2[1]);
    (void)alg_filter_low_pass_reset(&me->accelerometer_filter[2], accelerometer_m_s2[2]);
    me->filtered_accelerometer_m_s2[0] = accelerometer_m_s2[0];
    me->filtered_accelerometer_m_s2[1] = accelerometer_m_s2[1];
    me->filtered_accelerometer_m_s2[2] = accelerometer_m_s2[2];

    return ALG_IMU_EKF_STATUS_OK;
}