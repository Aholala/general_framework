/**
 * @file alg_attitude_madgwick.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief Madgwick 梯度下降滤波实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过梯度下降计算姿态误差，修正陀螺仪角速度。
 *       β（madgwick_beta）控制梯度下降的收敛速度。
 */

#include "alg_attitude_internal.h"

#include <math.h> // sqrtf

/**
 * @brief Madgwick 梯度下降滤波修正
 * @param me 姿态估计器对象
 * @param gyro_x_rad_per_s X 轴陀螺仪指针（会被修正）
 * @param gyro_y_rad_per_s Y 轴陀螺仪指针（会被修正）
 * @param gyro_z_rad_per_s Z 轴陀螺仪指针（会被修正）
 * @param normalized_acceleration_x 归一化后 X 轴加速度
 * @param normalized_acceleration_y 归一化后 Y 轴加速度
 * @param normalized_acceleration_z 归一化后 Z 轴加速度
 * @note 算法步骤：
 *       1. 构造重力对齐的误差函数 f(q, a)
 *       2. 计算误差函数的雅可比矩阵（梯度）
 *       3. 用梯度下降方向修正陀螺仪角速度
 */
void alg_attitude_madgwick_correct_gyro(const alg_attitude_t *me, float *gyro_x_rad_per_s,
                                        float *gyro_y_rad_per_s, float *gyro_z_rad_per_s,
                                        float normalized_acceleration_x,
                                        float normalized_acceleration_y,
                                        float normalized_acceleration_z)
{
    const float q0 = me->quaternion.q0;
    const float q1 = me->quaternion.q1;
    const float q2 = me->quaternion.q2;
    const float q3 = me->quaternion.q3;

    // ---- 计算误差函数的梯度 ----
    // 误差函数 f(q, a) 使四元数旋转后的重力方向与加速度计方向对齐
    // 梯度由偏导数构成（对 q0, q1, q2, q3 求导）
    float gradient_q0 = 4.0F * q0 * q2 * q2 + 2.0F * q2 * normalized_acceleration_x +
                        4.0F * q0 * q1 * q1 - 2.0F * q1 * normalized_acceleration_y;
    float gradient_q1 = 4.0F * q1 * q3 * q3 - 2.0F * q3 * normalized_acceleration_x +
                        4.0F * q0 * q0 * q1 - 2.0F * q0 * normalized_acceleration_y - 4.0F * q1 +
                        8.0F * q1 * q1 * q1 + 8.0F * q1 * q2 * q2 +
                        4.0F * q1 * normalized_acceleration_z;
    float gradient_q2 = 4.0F * q0 * q0 * q2 + 2.0F * q0 * normalized_acceleration_x +
                        4.0F * q2 * q3 * q3 - 2.0F * q3 * normalized_acceleration_y - 4.0F * q2 +
                        8.0F * q1 * q1 * q2 + 8.0F * q2 * q2 * q2 +
                        4.0F * q2 * normalized_acceleration_z;
    float gradient_q3 = 4.0F * q1 * q1 * q3 - 2.0F * q1 * normalized_acceleration_x +
                        4.0F * q2 * q2 * q3 - 2.0F * q2 * normalized_acceleration_y;

    // ---- 归一化梯度 ----
    const float norm = sqrtf(gradient_q0 * gradient_q0 + gradient_q1 * gradient_q1 +
                             gradient_q2 * gradient_q2 + gradient_q3 * gradient_q3);

    if (norm > 1.0e-8F)
    {
        gradient_q0 /= norm;
        gradient_q1 /= norm;
        gradient_q2 /= norm;
        gradient_q3 /= norm;

        // ---- 用梯度下降方向修正陀螺仪角速度 ----
        // 修正量 = β × 梯度方向 × 2（Madgwick 原始公式中的系数）
        *gyro_x_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q1 * gradient_q0 + q0 * gradient_q1 + q3 * gradient_q2 - q2 * gradient_q3);
        *gyro_y_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q2 * gradient_q0 - q3 * gradient_q1 + q0 * gradient_q2 + q1 * gradient_q3);
        *gyro_z_rad_per_s -=
            2.0F * me->config.madgwick_beta *
            (-q3 * gradient_q0 + q2 * gradient_q1 - q1 * gradient_q2 + q0 * gradient_q3);
    }
}