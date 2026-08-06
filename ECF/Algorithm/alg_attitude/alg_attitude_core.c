/**
 * @file alg_attitude_core.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 姿态算法核心实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含对象初始化、复位、更新调度、四元数积分和航向修正。
 */

#include "alg_attitude_internal.h"

#include <math.h> // sqrtf, isfinite, remainderf

/**
 * @brief 归一化四元数（内部函数）
 * @param quaternion 四元数指针（会被修改）
 * @return true=成功，false=失败（模长为零或非有限）
 * @note 若四元数模长接近零，归一化会失败
 */
static bool alg_attitude_normalize_quaternion(alg_attitude_quaternion_t *quaternion)
{
    // 计算四元数模长
    const float norm = sqrtf(quaternion->q0 * quaternion->q0 + quaternion->q1 * quaternion->q1 +
                             quaternion->q2 * quaternion->q2 + quaternion->q3 * quaternion->q3);
    // 检查模长是否有效
    if (!isfinite(norm) || (norm < 1.0e-8F))
    {
        return false;
    }
    // 各分量除以模长
    quaternion->q0 /= norm;
    quaternion->q1 /= norm;
    quaternion->q2 /= norm;
    quaternion->q3 /= norm;
    return true;
}

/**
 * @brief 陀螺仪积分更新四元数（内部函数）
 * @param quaternion 四元数指针（会被修改）
 * @param gyro_x_rad_per_s X 轴角速度（rad/s）
 * @param gyro_y_rad_per_s Y 轴角速度（rad/s）
 * @param gyro_z_rad_per_s Z 轴角速度（rad/s）
 * @param delta_time_s 时间步长（秒）
 * @note 使用一阶龙格-库塔（RK1）积分
 *       四元数微分方程：dq/dt = 0.5 * q ⊗ ω
 */
static void alg_attitude_integrate_gyro(alg_attitude_quaternion_t *quaternion,
                                        float gyro_x_rad_per_s,
                                        float gyro_y_rad_per_s,
                                        float gyro_z_rad_per_s,
                                        float delta_time_s)
{
    const float half_delta_time_s = 0.5F * delta_time_s;
    const float q0 = quaternion->q0;
    const float q1 = quaternion->q1;
    const float q2 = quaternion->q2;
    const float q3 = quaternion->q3;

    // 四元数微分方程：dq = 0.5 * q ⊗ ω * dt
    quaternion->q0 += (-q1 * gyro_x_rad_per_s - q2 * gyro_y_rad_per_s - q3 * gyro_z_rad_per_s) * half_delta_time_s;
    quaternion->q1 += (q0 * gyro_x_rad_per_s + q2 * gyro_z_rad_per_s - q3 * gyro_y_rad_per_s) * half_delta_time_s;
    quaternion->q2 += (q0 * gyro_y_rad_per_s - q1 * gyro_z_rad_per_s + q3 * gyro_x_rad_per_s) * half_delta_time_s;
    quaternion->q3 += (q0 * gyro_z_rad_per_s + q1 * gyro_y_rad_per_s - q2 * gyro_x_rad_per_s) * half_delta_time_s;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化姿态估计器
 * @param me 姿态估计器对象
 * @param config 配置参数
 * @param initial_quaternion 初始四元数（NULL 则使用单位四元数）
 * @return 执行状态
 */
alg_attitude_status_t alg_attitude_init(alg_attitude_t *me, const alg_attitude_config_t *config,
                                        const alg_attitude_quaternion_t *initial_quaternion)
{
    // 默认初始四元数：单位四元数（无旋转）
    alg_attitude_quaternion_t quaternion = {1.0F, 0.0F, 0.0F, 0.0F};

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->method > ALG_ATTITUDE_METHOD_MADGWICK) ||
        !isfinite(config->proportional_gain) || !isfinite(config->integral_gain) ||
        !isfinite(config->madgwick_beta) || !isfinite(config->acceleration_min_m_per_s2) ||
        !isfinite(config->acceleration_max_m_per_s2) || (config->proportional_gain < 0.0F) ||
        (config->integral_gain < 0.0F) || (config->madgwick_beta < 0.0F) ||
        (config->acceleration_min_m_per_s2 < 0.0F) ||
        (config->acceleration_max_m_per_s2 <= config->acceleration_min_m_per_s2))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 使用指定的初始四元数（或默认单位四元数） ----
    if (initial_quaternion != NULL)
    {
        quaternion = *initial_quaternion;
    }
    // 确保初始四元数归一化
    if (!alg_attitude_normalize_quaternion(&quaternion))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (alg_attitude_t){
        .config = *config,
        .quaternion = quaternion,
        .is_initialized = true,
    };
    return ALG_ATTITUDE_STATUS_OK;
}

/**
 * @brief 重置姿态到指定四元数
 * @param me 姿态估计器对象
 * @param quaternion 目标四元数
 * @return 执行状态
 * @note 同时清除 Mahony 积分误差
 */
alg_attitude_status_t alg_attitude_reset(alg_attitude_t *me,
                                         const alg_attitude_quaternion_t *quaternion)
{
    alg_attitude_quaternion_t normalized;

    if ((me == NULL) || (quaternion == NULL))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    // 归一化目标四元数
    normalized = *quaternion;
    if (!alg_attitude_normalize_quaternion(&normalized))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }

    // 重置状态
    me->quaternion = normalized;
    me->integral_error_x = 0.0F;
    me->integral_error_y = 0.0F;
    me->integral_error_z = 0.0F;
    return ALG_ATTITUDE_STATUS_OK;
}

/**
 * @brief 更新姿态估计（核心函数）
 * @param me 姿态估计器对象
 * @param gyro_x_rad_per_s X 轴陀螺仪（rad/s）
 * @param gyro_y_rad_per_s Y 轴陀螺仪（rad/s）
 * @param gyro_z_rad_per_s Z 轴陀螺仪（rad/s）
 * @param acceleration_x_m_per_s2 X 轴加速度（m/s²）
 * @param acceleration_y_m_per_s2 Y 轴加速度（m/s²）
 * @param acceleration_z_m_per_s2 Z 轴加速度（m/s²）
 * @param delta_time_s 时间步长（秒）
 * @return OK=使用加速度修正，GYRO_ONLY=仅陀螺仪积分
 */
alg_attitude_status_t alg_attitude_update(alg_attitude_t *me, float gyro_x_rad_per_s,
                                          float gyro_y_rad_per_s, float gyro_z_rad_per_s,
                                          float acceleration_x_m_per_s2,
                                          float acceleration_y_m_per_s2,
                                          float acceleration_z_m_per_s2, float delta_time_s)
{
    // 计算加速度模长
    const float acceleration_norm = sqrtf(acceleration_x_m_per_s2 * acceleration_x_m_per_s2 +
                                          acceleration_y_m_per_s2 * acceleration_y_m_per_s2 +
                                          acceleration_z_m_per_s2 * acceleration_z_m_per_s2);
    bool use_acceleration;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(gyro_x_rad_per_s) || !isfinite(gyro_y_rad_per_s) ||
        !isfinite(gyro_z_rad_per_s) || !isfinite(acceleration_norm) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    // ---- 检查加速度是否在有效范围内 ----
    // 若加速度模长在 [min, max] 内，认为 IMU 处于静止或匀速运动，使用加速度修正
    use_acceleration = (acceleration_norm >= me->config.acceleration_min_m_per_s2) &&
                       (acceleration_norm <= me->config.acceleration_max_m_per_s2);

    if (use_acceleration)
    {
        // 归一化加速度（仅使用方向信息）
        const float normalized_acceleration_x = acceleration_x_m_per_s2 / acceleration_norm;
        const float normalized_acceleration_y = acceleration_y_m_per_s2 / acceleration_norm;
        const float normalized_acceleration_z = acceleration_z_m_per_s2 / acceleration_norm;

        // 根据配置的算法进行修正
        if (me->config.method == ALG_ATTITUDE_METHOD_MAHONY)
        {
            // Mahony：使用 PI 反馈修正陀螺仪
            alg_attitude_mahony_correct_gyro(me, &gyro_x_rad_per_s, &gyro_y_rad_per_s,
                                             &gyro_z_rad_per_s, normalized_acceleration_x,
                                             normalized_acceleration_y, normalized_acceleration_z,
                                             delta_time_s);
        }
        else
        {
            // Madgwick：使用梯度下降修正陀螺仪
            alg_attitude_madgwick_correct_gyro(
                me, &gyro_x_rad_per_s, &gyro_y_rad_per_s, &gyro_z_rad_per_s,
                normalized_acceleration_x, normalized_acceleration_y, normalized_acceleration_z);
        }
    }

    // ---- 陀螺仪积分更新四元数 ----
    alg_attitude_integrate_gyro(&me->quaternion, gyro_x_rad_per_s, gyro_y_rad_per_s,
                                gyro_z_rad_per_s, delta_time_s);

    // ---- 四元数归一化 ----
    if (!alg_attitude_normalize_quaternion(&me->quaternion))
    {
        return ALG_ATTITUDE_STATUS_NUMERICAL_ERROR;
    }

    // 返回状态：若使用加速度修正则返回 OK，否则返回 GYRO_ONLY
    return use_acceleration ? ALG_ATTITUDE_STATUS_OK : ALG_ATTITUDE_STATUS_GYRO_ONLY;
}

/**
 * @brief 外部航向修正（注入磁航向、视觉航向或机构约束）
 * @param me 姿态估计器对象
 * @param measured_yaw_rad 测量的航向角（弧度）
 * @param correction_gain 修正增益（0~1）
 * @return 执行状态
 * @note 将外部航向作为虚拟角速度注入，修正当前 yaw
 */
alg_attitude_status_t alg_attitude_correct_yaw(alg_attitude_t *me, float measured_yaw_rad,
                                               float correction_gain)
{
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float yaw_error_rad;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(measured_yaw_rad) || !isfinite(correction_gain) ||
        (correction_gain < 0.0F) || (correction_gain > 1.0F))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 获取当前欧拉角 ----
    if (alg_attitude_get_euler(me, &roll_rad, &pitch_rad, &yaw_rad) != ALG_ATTITUDE_STATUS_OK)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    // ---- 计算航向误差（处理 π 环绕） ----
    yaw_error_rad = remainderf(measured_yaw_rad - yaw_rad, 2.0F * 3.14159265358979323846F);

    // ---- 将误差作为虚拟 Z 轴角速度注入 ----
    // 使用 1.0 秒的时间步长，等效于直接修正四元数
    alg_attitude_integrate_gyro(&me->quaternion, 0.0F, 0.0F, yaw_error_rad * correction_gain, 1.0F);

    // ---- 归一化 ----
    return alg_attitude_normalize_quaternion(&me->quaternion) ? ALG_ATTITUDE_STATUS_OK
                                                              : ALG_ATTITUDE_STATUS_NUMERICAL_ERROR;
}