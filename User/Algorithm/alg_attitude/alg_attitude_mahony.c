/**
 * @file alg_attitude_mahony.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief Mahony 互补滤波实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过向量叉积计算姿态误差，使用 PI 反馈修正陀螺仪角速度。
 *       比例增益（Kp）控制即时修正强度，积分增益（Ki）补偿陀螺零偏。
 */

#include "alg_attitude_internal.h"

/**
 * @brief Mahony 互补滤波修正
 * @param me 姿态估计器对象
 * @param gyro_x_rad_per_s X 轴陀螺仪指针（会被修正）
 * @param gyro_y_rad_per_s Y 轴陀螺仪指针（会被修正）
 * @param gyro_z_rad_per_s Z 轴陀螺仪指针（会被修正）
 * @param normalized_acceleration_x 归一化后 X 轴加速度
 * @param normalized_acceleration_y 归一化后 Y 轴加速度
 * @param normalized_acceleration_z 归一化后 Z 轴加速度
 * @param delta_time_s 时间步长（秒）
 * @note 算法步骤：
 *       1. 从当前四元数估计重力方向
 *       2. 计算加速度计测量与估计方向的叉积（姿态误差）
 *       3. PI 控制器将误差反馈修正陀螺仪角速度
 */
void alg_attitude_mahony_correct_gyro(alg_attitude_t *me, float *gyro_x_rad_per_s,
                                      float *gyro_y_rad_per_s, float *gyro_z_rad_per_s,
                                      float normalized_acceleration_x,
                                      float normalized_acceleration_y,
                                      float normalized_acceleration_z, float delta_time_s)
{
    const alg_attitude_quaternion_t *const quaternion = &me->quaternion;

    // ---- 从当前四元数估计重力方向（在传感器坐标系中） ----
    // 将世界坐标系的 [0, 0, 1]（重力方向）旋转到传感器坐标系
    // 使用四元数旋转公式：g_sensor = R^T * [0, 0, 1]
    const float estimated_gravity_x =
        2.0F * (quaternion->q1 * quaternion->q3 - quaternion->q0 * quaternion->q2);
    const float estimated_gravity_y =
        2.0F * (quaternion->q0 * quaternion->q1 + quaternion->q2 * quaternion->q3);
    const float estimated_gravity_z =
        quaternion->q0 * quaternion->q0 - quaternion->q1 * quaternion->q1 -
        quaternion->q2 * quaternion->q2 + quaternion->q3 * quaternion->q3;

    // ---- 向量叉积：加速度计测量方向 × 估计重力方向 ----
    // 当两者方向不一致时，叉积提供姿态误差的旋转轴和大小
    const float error_x = normalized_acceleration_y * estimated_gravity_z -
                          normalized_acceleration_z * estimated_gravity_y;
    const float error_y = normalized_acceleration_z * estimated_gravity_x -
                          normalized_acceleration_x * estimated_gravity_z;
    const float error_z = normalized_acceleration_x * estimated_gravity_y -
                          normalized_acceleration_y * estimated_gravity_x;

    // ---- PI 控制器 ----
    // 积分项：累积误差 × 积分增益 × 时间步长
    me->integral_error_x += me->config.integral_gain * error_x * delta_time_s;
    me->integral_error_y += me->config.integral_gain * error_y * delta_time_s;
    me->integral_error_z += me->config.integral_gain * error_z * delta_time_s;

    // 比例 + 积分反馈修正陀螺仪角速度
    *gyro_x_rad_per_s += me->config.proportional_gain * error_x + me->integral_error_x;
    *gyro_y_rad_per_s += me->config.proportional_gain * error_y + me->integral_error_y;
    *gyro_z_rad_per_s += me->config.proportional_gain * error_z + me->integral_error_z;
}