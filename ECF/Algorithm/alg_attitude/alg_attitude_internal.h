/**
 * @file alg_attitude_internal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 姿态算法内部头文件
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 仅供本模块各源文件共享的内部接口，不对外暴露。
 */

#ifndef ALG_ATTITUDE_INTERNAL_H
#define ALG_ATTITUDE_INTERNAL_H

#include "alg_attitude.h" // 公共类型和 API

/**
 * @brief Mahony 互补滤波修正（内部函数）
 * @param me 姿态估计器对象
 * @param gyro_x_rad_per_s X 轴陀螺仪指针（会被修正）
 * @param gyro_y_rad_per_s Y 轴陀螺仪指针（会被修正）
 * @param gyro_z_rad_per_s Z 轴陀螺仪指针（会被修正）
 * @param normalized_acceleration_x 归一化后 X 轴加速度
 * @param normalized_acceleration_y 归一化后 Y 轴加速度
 * @param normalized_acceleration_z 归一化后 Z 轴加速度
 * @param delta_time_s 时间步长（秒）
 * @note 通过向量叉积计算姿态误差，用 PI 反馈修正陀螺仪角速度
 */
void alg_attitude_mahony_correct_gyro(alg_attitude_t *me,
                                      float *gyro_x_rad_per_s,
                                      float *gyro_y_rad_per_s,
                                      float *gyro_z_rad_per_s,
                                      float normalized_acceleration_x,
                                      float normalized_acceleration_y,
                                      float normalized_acceleration_z,
                                      float delta_time_s);

/**
 * @brief Madgwick 梯度下降滤波修正（内部函数）
 * @param me 姿态估计器对象
 * @param gyro_x_rad_per_s X 轴陀螺仪指针（会被修正）
 * @param gyro_y_rad_per_s Y 轴陀螺仪指针（会被修正）
 * @param gyro_z_rad_per_s Z 轴陀螺仪指针（会被修正）
 * @param normalized_acceleration_x 归一化后 X 轴加速度
 * @param normalized_acceleration_y 归一化后 Y 轴加速度
 * @param normalized_acceleration_z 归一化后 Z 轴加速度
 * @note 通过梯度下降计算姿态误差，修正陀螺仪角速度
 */
void alg_attitude_madgwick_correct_gyro(const alg_attitude_t *me, float *gyro_x_rad_per_s,
                                        float *gyro_y_rad_per_s,
                                        float *gyro_z_rad_per_s,
                                        float normalized_acceleration_x,
                                        float normalized_acceleration_y,
                                        float normalized_acceleration_z);

#endif /* ALG_ATTITUDE_INTERNAL_H */