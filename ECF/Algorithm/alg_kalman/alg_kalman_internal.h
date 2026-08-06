/**
 * @file alg_kalman_internal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 卡尔曼滤波库内部接口头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 仅供本模块内部使用，不对外暴露。
 *       包含矩阵运算、数组检查、协方差校正等内部工具函数。
 */

#ifndef ALG_KALMAN_INTERNAL_H
#define ALG_KALMAN_INTERNAL_H

#include "alg_kalman.h"

/**
 * @brief 检查数组元素是否全部为有限数
 * @param values 数组指针
 * @param value_count 元素个数
 * @return true=全有限
 */
bool alg_kalman_internal_is_finite_array(const float *values, size_t value_count);

/**
 * @brief 检查矩阵对角线是否全部为非负有限数
 * @param matrix 矩阵指针（行优先）
 * @param dimension 矩阵维度（方阵）
 * @return true=对角线全为非负有限数
 * @note 用于检查协方差矩阵和噪声矩阵的有效性
 */
bool alg_kalman_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension);

/**
 * @brief 拷贝数组
 * @param destination 目标数组
 * @param source 源数组
 * @param value_count 元素个数
 */
void alg_kalman_internal_copy(float *destination, const float *source, size_t value_count);

/**
 * @brief 矩阵乘法：C = A * B
 * @param left 左矩阵 A（left_rows × shared_dimension）
 * @param left_rows A 的行数
 * @param shared_dimension A 的列数 = B 的行数
 * @param right 右矩阵 B（shared_dimension × right_columns）
 * @param right_columns B 的列数
 * @param output 输出矩阵 C（left_rows × right_columns）
 */
void alg_kalman_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                                  const float *right, size_t right_columns, float *output);

/**
 * @brief 矩阵乘法：C = A * B^T
 * @param left 左矩阵 A（left_rows × shared_dimension）
 * @param left_rows A 的行数
 * @param shared_dimension A 的列数 = B 的列数
 * @param right 右矩阵 B（right_rows × shared_dimension）
 * @param right_rows B 的行数
 * @param output 输出矩阵 C（left_rows × right_rows）
 * @note 用于计算 P * H^T 等场景
 */
void alg_kalman_internal_multiply_right_transpose(const float *left, size_t left_rows,
                                                  size_t shared_dimension, const float *right,
                                                  size_t right_rows, float *output);

/**
 * @brief 执行卡尔曼校正（标准 EKF 更新）
 * @param state 状态向量（会被更新）
 * @param covariance 协方差矩阵（会被更新）
 * @param state_dimension 状态维度
 * @param measurement_matrix 观测矩阵 H（m×n）
 * @param measurement_noise 测量噪声矩阵 R（m×m）
 * @param measurement 测量值（m×1）
 * @param predicted_measurement 预测测量值（m×1）
 * @param measurement_dimension 测量维度
 * @param workspace 工作区
 * @param workspace_size 工作区大小
 * @return 执行状态
 * @note 使用 Joseph 形式更新协方差，提高数值稳定性
 *       计算流程：
 *       1. 创新 y = z - h(x)
 *       2. 创新协方差 S = H*P*H^T + R
 *       3. 卡尔曼增益 K = P*H^T * S^-1
 *       4. 状态更新 x = x + K*y
 *       5. 协方差更新 P = (I-KH)*P*(I-KH)^T + K*R*K^T
 */
alg_kalman_status_t
alg_kalman_internal_correct(float *state, float *covariance, size_t state_dimension,
                            const float *measurement_matrix, const float *measurement_noise,
                            const float *measurement, const float *predicted_measurement,
                            size_t measurement_dimension, float *workspace, size_t workspace_size);

/**
 * @brief 对称化矩阵：P = (P + P^T) / 2
 * @param matrix 矩阵指针（会被修改）
 * @param dimension 矩阵维度
 * @note 用于消除数值误差导致的非对称性
 */
void alg_kalman_internal_symmetrize(float *matrix, size_t dimension);

#endif /* ALG_KALMAN_INTERNAL_H */