/**
 * @file alg_imu_ekf_output.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief IMU EKF 输出函数实现
 * @version 1.0
 * @date 2026-07-24
 * @copyright Copyright (c) 2026
 *
 * @note 包含四元数、欧拉角、零偏、校正陀螺仪、诊断信息、
 *       重力向量和线性加速度的输出函数。
 */

#include "alg_imu_ekf_internal.h"

#include <math.h>   // atan2f, asinf, sqrtf, isfinite
#include <stddef.h> // NULL

/**
 * @brief 将值钳位到 [-1, 1]（用于 asinf 输入）
 * @param value 输入值
 * @return 钳位后的值
 */
static float alg_imu_ekf_clamp_unit(float value)
{
    if (value > 1.0F)
        return 1.0F;
    if (value < -1.0F)
        return -1.0F;
    return value;
}

/**
 * @brief 获取当前四元数
 * @param me EKF 对象
 * @param quaternion 输出四元数
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_quaternion(const alg_imu_ekf_t *me,
                                                alg_imu_ekf_quaternion_t *quaternion)
{
    if ((me == NULL) || (quaternion == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;

    quaternion->w = me->state[0];
    quaternion->x = me->state[1];
    quaternion->y = me->state[2];
    quaternion->z = me->state[3];
    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取当前欧拉角（ZYX 旋转顺序）
 * @param me EKF 对象
 * @param euler 输出欧拉角
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_euler(const alg_imu_ekf_t *me, alg_imu_ekf_euler_t *euler)
{
    float sine_pitch;

    if ((me == NULL) || (euler == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;

    // Roll：atan2(2(q0*q1 + q2*q3), 1 - 2(q1² + q2²))
    euler->roll_rad =
        atan2f(2.0F * ((me->state[0] * me->state[1]) + (me->state[2] * me->state[3])),
               1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[2] * me->state[2]))));

    // Pitch：asin(2(q0*q2 - q3*q1))，输入钳位到 [-1, 1]
    sine_pitch = 2.0F * ((me->state[0] * me->state[2]) - (me->state[3] * me->state[1]));
    euler->pitch_rad = asinf(alg_imu_ekf_clamp_unit(sine_pitch));

    // Yaw：atan2(2(q0*q3 + q1*q2), 1 - 2(q2² + q3²))
    euler->yaw_rad =
        atan2f(2.0F * ((me->state[0] * me->state[3]) + (me->state[1] * me->state[2])),
               1.0F - (2.0F * ((me->state[2] * me->state[2]) + (me->state[3] * me->state[3]))));

    return (isfinite(euler->roll_rad) && isfinite(euler->pitch_rad) && isfinite(euler->yaw_rad))
               ? ALG_IMU_EKF_STATUS_OK
               : ALG_IMU_EKF_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 获取陀螺仪零偏（X/Y 轴）
 * @param me EKF 对象
 * @param gyro_bias_rad_s 输出零偏（长度 3），Z 轴恒为 0
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_gyro_bias(const alg_imu_ekf_t *me, float gyro_bias_rad_s[3])
{
    if ((me == NULL) || (gyro_bias_rad_s == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;

    gyro_bias_rad_s[0] = me->state[4];
    gyro_bias_rad_s[1] = me->state[5];
    gyro_bias_rad_s[2] = 0.0F; // Z 轴零偏不估计（六轴 IMU 无法观测）
    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取校正后的陀螺仪读数
 * @param me EKF 对象
 * @param gyroscope_rad_s 原始陀螺仪读数
 * @param corrected_gyroscope_rad_s 输出校正后读数
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_corrected_gyroscope(const alg_imu_ekf_t *me,
                                                         const float gyroscope_rad_s[3],
                                                         float corrected_gyroscope_rad_s[3])
{
    size_t axis;

    if ((me == NULL) || (gyroscope_rad_s == NULL) || (corrected_gyroscope_rad_s == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    if (!alg_imu_ekf_internal_is_finite_array(gyroscope_rad_s, 3U))
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;

    // X/Y 轴减去零偏
    for (axis = 0U; axis < 2U; ++axis)
        corrected_gyroscope_rad_s[axis] = gyroscope_rad_s[axis] - me->state[4U + axis];

    // Z 轴不变（无零偏估计）
    corrected_gyroscope_rad_s[2] = gyroscope_rad_s[2];

    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取诊断信息
 * @param me EKF 对象
 * @param diagnostics 输出诊断信息
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_diagnostics(const alg_imu_ekf_t *me,
                                                 alg_imu_ekf_diagnostics_t *diagnostics)
{
    size_t index;

    if ((me == NULL) || (diagnostics == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;

    for (index = 0U; index < 3U; ++index)
    {
        diagnostics->filtered_accelerometer_m_s2[index] = me->filtered_accelerometer_m_s2[index];
        diagnostics->innovation[index] = me->innovation[index];
    }

    diagnostics->accelerometer_norm_m_s2 = me->last_accelerometer_norm_m_s2;
    diagnostics->accelerometer_deviation_g = me->last_accelerometer_deviation_g;
    diagnostics->normalized_innovation_squared = me->last_normalized_innovation_squared;
    diagnostics->measurement_noise_scale = me->last_measurement_noise_scale;
    diagnostics->was_accelerometer_used = me->was_accelerometer_used;

    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取机体系中的重力向量
 * @param me EKF 对象
 * @param gravity_body_m_s2 输出重力向量（m/s²）
 * @return 执行状态
 */
alg_imu_ekf_status_t alg_imu_ekf_get_gravity_body(const alg_imu_ekf_t *me,
                                                  float gravity_body_m_s2[3])
{
    if ((me == NULL) || (gravity_body_m_s2 == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;

    // 将世界系重力 [0, 0, g] 旋转到机体系
    gravity_body_m_s2[0] = me->config.gravity_m_s2 * 2.0F *
                           ((me->state[1] * me->state[3]) - (me->state[0] * me->state[2]));
    gravity_body_m_s2[1] = me->config.gravity_m_s2 * 2.0F *
                           ((me->state[0] * me->state[1]) + (me->state[2] * me->state[3]));
    gravity_body_m_s2[2] =
        me->config.gravity_m_s2 * ((me->state[0] * me->state[0]) - (me->state[1] * me->state[1]) -
                                   (me->state[2] * me->state[2]) + (me->state[3] * me->state[3]));

    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取机体系中的线性加速度（测量值减去重力）
 * @param me EKF 对象
 * @param accelerometer_m_s2 加速度计读数
 * @param linear_acceleration_body_m_s2 输出线性加速度（m/s²）
 * @return 执行状态
 */
alg_imu_ekf_status_t
alg_imu_ekf_get_linear_acceleration_body(const alg_imu_ekf_t *me, const float accelerometer_m_s2[3],
                                         float linear_acceleration_body_m_s2[3])
{
    float gravity_body[3];
    size_t axis;
    alg_imu_ekf_status_t status;

    if ((me == NULL) || (accelerometer_m_s2 == NULL) || (linear_acceleration_body_m_s2 == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U))
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;

    status = alg_imu_ekf_get_gravity_body(me, gravity_body);
    if (status != ALG_IMU_EKF_STATUS_OK)
        return status;

    // 线性加速度 = 测量值 - 重力
    for (axis = 0U; axis < 3U; ++axis)
        linear_acceleration_body_m_s2[axis] = accelerometer_m_s2[axis] - gravity_body[axis];

    return ALG_IMU_EKF_STATUS_OK;
}

/**
 * @brief 获取世界系中的线性加速度（旋转后减去重力）
 * @param me EKF 对象
 * @param accelerometer_m_s2 加速度计读数
 * @param linear_acceleration_world_m_s2 输出世界系线性加速度（m/s²）
 * @return 执行状态
 */
alg_imu_ekf_status_t
alg_imu_ekf_get_linear_acceleration_world(const alg_imu_ekf_t *me,
                                          const float accelerometer_m_s2[3],
                                          float linear_acceleration_world_m_s2[3])
{
    float rotation[9]; // 机体系到世界系的旋转矩阵

    if ((me == NULL) || (accelerometer_m_s2 == NULL) || (linear_acceleration_world_m_s2 == NULL))
        return ALG_IMU_EKF_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_IMU_EKF_STATUS_NOT_INITIALIZED;
    if (!alg_imu_ekf_internal_is_finite_array(accelerometer_m_s2, 3U))
        return ALG_IMU_EKF_STATUS_OUT_OF_RANGE;

    // ---- 构建从机体系到世界系的旋转矩阵 ----
    rotation[0] = 1.0F - (2.0F * ((me->state[2] * me->state[2]) + (me->state[3] * me->state[3])));
    rotation[1] = 2.0F * ((me->state[1] * me->state[2]) - (me->state[0] * me->state[3]));
    rotation[2] = 2.0F * ((me->state[1] * me->state[3]) + (me->state[0] * me->state[2]));
    rotation[3] = 2.0F * ((me->state[1] * me->state[2]) + (me->state[0] * me->state[3]));
    rotation[4] = 1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[3] * me->state[3])));
    rotation[5] = 2.0F * ((me->state[2] * me->state[3]) - (me->state[0] * me->state[1]));
    rotation[6] = 2.0F * ((me->state[1] * me->state[3]) - (me->state[0] * me->state[2]));
    rotation[7] = 2.0F * ((me->state[2] * me->state[3]) + (me->state[0] * me->state[1]));
    rotation[8] = 1.0F - (2.0F * ((me->state[1] * me->state[1]) + (me->state[2] * me->state[2])));

    // ---- 将机体系加速度旋转到世界系 ----
    linear_acceleration_world_m_s2[0] = (rotation[0] * accelerometer_m_s2[0]) +
                                        (rotation[1] * accelerometer_m_s2[1]) +
                                        (rotation[2] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[1] = (rotation[3] * accelerometer_m_s2[0]) +
                                        (rotation[4] * accelerometer_m_s2[1]) +
                                        (rotation[5] * accelerometer_m_s2[2]);
    linear_acceleration_world_m_s2[2] =
        (rotation[6] * accelerometer_m_s2[0]) + (rotation[7] * accelerometer_m_s2[1]) +
        (rotation[8] * accelerometer_m_s2[2]) - me->config.gravity_m_s2;

    return ALG_IMU_EKF_STATUS_OK;
}