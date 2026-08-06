/**
 * @file alg_attitude_output.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 姿态输出函数实现
 * @version 1.0
 * @date 2026-06-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供欧拉角和旋转矩阵输出。
 *       欧拉角顺序为 roll → pitch → yaw（ZYX 旋转顺序），单位弧度。
 *       旋转矩阵从机体坐标系到世界坐标系。
 */

#include "alg_attitude.h"

#include <math.h> // atan2f, asinf, fmaxf, fminf

/**
 * @brief 获取欧拉角（roll/pitch/yaw）
 * @param me 姿态估计器对象
 * @param roll_rad 输出滚转角（弧度，范围 -π~π）
 * @param pitch_rad 输出俯仰角（弧度，范围 -π/2~π/2）
 * @param yaw_rad 输出偏航角（弧度，范围 -π~π）
 * @return 执行状态
 * @note 转换公式基于 ZYX 旋转顺序的四元数：
 *       roll = atan2(2(q0q1 + q2q3), 1 - 2(q1² + q2²))
 *       pitch = asin(2(q0q2 - q3q1))
 *       yaw = atan2(2(q0q3 + q1q2), 1 - 2(q2² + q3²))
 */
alg_attitude_status_t alg_attitude_get_euler(const alg_attitude_t *me,
                                             float *roll_rad,
                                             float *pitch_rad,
                                             float *yaw_rad)
{
    float pitch_sine;

    // ---- 参数校验 ----
    if ((me == NULL) || (roll_rad == NULL) || (pitch_rad == NULL) || (yaw_rad == NULL))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    // ---- 计算滚转角（roll） ----
    *roll_rad = atan2f(
        2.0F * (me->quaternion.q0 * me->quaternion.q1 + me->quaternion.q2 * me->quaternion.q3),
        1.0F - 2.0F * (me->quaternion.q1 * me->quaternion.q1 + me->quaternion.q2 * me->quaternion.q2));

    // ---- 计算俯仰角（pitch） ----
    // asin 的输入会被钳位到 [-1, 1] 防止数值误差
    pitch_sine =
        2.0F * (me->quaternion.q0 * me->quaternion.q2 - me->quaternion.q3 * me->quaternion.q1);
    *pitch_rad = asinf(fmaxf(-1.0F, fminf(1.0F, pitch_sine)));

    // ---- 计算偏航角（yaw） ----
    *yaw_rad = atan2f(
        2.0F * (me->quaternion.q0 * me->quaternion.q3 + me->quaternion.q1 * me->quaternion.q2),
        1.0F - 2.0F * (me->quaternion.q2 * me->quaternion.q2 + me->quaternion.q3 * me->quaternion.q3));

    return ALG_ATTITUDE_STATUS_OK;
}

/**
 * @brief 获取旋转矩阵
 * @param me 姿态估计器对象
 * @param rotation_matrix 输出 3×3 旋转矩阵
 * @return 执行状态
 * @note 旋转矩阵 R 满足：R * 机体向量 = 世界向量
 *       公式基于四元数：R = I + 2q×q× + 2q0(q×)
 */
alg_attitude_status_t
alg_attitude_get_rotation_matrix(const alg_attitude_t *me,
                                 alg_attitude_rotation_matrix_t *rotation_matrix)
{
    const alg_attitude_quaternion_t *quaternion;
    float q0q0, q0q1, q0q2, q0q3;
    float q1q1, q1q2, q1q3;
    float q2q2, q2q3;
    float q3q3;

    // ---- 参数校验 ----
    if ((me == NULL) || (rotation_matrix == NULL))
    {
        return ALG_ATTITUDE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_ATTITUDE_STATUS_NOT_INITIALIZED;
    }

    // ---- 预计算四元数分量乘积 ----
    quaternion = &me->quaternion;
    q0q0 = quaternion->q0 * quaternion->q0;
    q0q1 = quaternion->q0 * quaternion->q1;
    q0q2 = quaternion->q0 * quaternion->q2;
    q0q3 = quaternion->q0 * quaternion->q3;
    q1q1 = quaternion->q1 * quaternion->q1;
    q1q2 = quaternion->q1 * quaternion->q2;
    q1q3 = quaternion->q1 * quaternion->q3;
    q2q2 = quaternion->q2 * quaternion->q2;
    q2q3 = quaternion->q2 * quaternion->q3;
    q3q3 = quaternion->q3 * quaternion->q3;

    // ---- 填充旋转矩阵 ----
    // 第一行：X 轴旋转分量
    rotation_matrix->element[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    rotation_matrix->element[0][1] = 2.0F * (q1q2 - q0q3);
    rotation_matrix->element[0][2] = 2.0F * (q1q3 + q0q2);

    // 第二行：Y 轴旋转分量
    rotation_matrix->element[1][0] = 2.0F * (q1q2 + q0q3);
    rotation_matrix->element[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    rotation_matrix->element[1][2] = 2.0F * (q2q3 - q0q1);

    // 第三行：Z 轴旋转分量
    rotation_matrix->element[2][0] = 2.0F * (q1q3 - q0q2);
    rotation_matrix->element[2][1] = 2.0F * (q2q3 + q0q1);
    rotation_matrix->element[2][2] = q0q0 - q1q1 - q2q2 + q3q3;

    return ALG_ATTITUDE_STATUS_OK;
}