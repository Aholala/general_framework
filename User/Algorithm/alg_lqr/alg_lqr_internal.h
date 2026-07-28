/**
 * @file alg_lqr_internal.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief LQR 库内部接口头文件（仅供本模块使用）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 声明内部工具函数：数组检查、矩阵运算、求逆、Riccati 步进等。
 *       不对外暴露。
 */

#ifndef ALG_LQR_INTERNAL_H
#define ALG_LQR_INTERNAL_H

#include "alg_lqr.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 检查数组是否全为有限数
     * @param values      数组指针
     * @param value_count 元素个数
     * @return true 表示全部有限
     */
    bool alg_lqr_internal_is_finite_array(const float *values, size_t value_count);

    /**
     * @brief 检查方阵对角线是否全为非负有限数
     * @param matrix    矩阵指针（行优先）
     * @param dimension 矩阵维度
     * @return true 表示对角线合法
     */
    bool alg_lqr_internal_has_nonnegative_diagonal(const float *matrix, size_t dimension);

    /**
     * @brief 拷贝数组
     * @param destination 目标
     * @param source      源
     * @param value_count 元素个数
     */
    void alg_lqr_internal_copy(float *destination, const float *source, size_t value_count);

    /**
     * @brief 矩阵乘法：C = A * B
     * @param left             A（left_rows × shared_dimension）
     * @param left_rows        A 的行数
     * @param shared_dimension A 的列数 = B 的行数
     * @param right            B（shared_dimension × right_columns）
     * @param right_columns    B 的列数
     * @param output           C（left_rows × right_columns）
     */
    void alg_lqr_internal_multiply(const float *left, size_t left_rows, size_t shared_dimension,
                                   const float *right, size_t right_columns, float *output);

    /**
     * @brief 矩阵乘法：C = A^T * B
     * @param left          A（left_rows × left_columns），计算其转置
     * @param left_rows     A 的行数
     * @param left_columns  A 的列数
     * @param right         B（left_rows × right_columns）
     * @param right_columns B 的列数
     * @param output        C（left_columns × right_columns）
     * @note 用于计算 A^T * B，避免显式转置。
     */
    void alg_lqr_internal_multiply_left_transpose(const float *left, size_t left_rows,
                                                  size_t left_columns, const float *right,
                                                  size_t right_columns, float *output);

    /**
     * @brief 对称化矩阵：P = (P + P^T) / 2
     * @param matrix    方阵
     * @param dimension 维度
     */
    void alg_lqr_internal_symmetrize(float *matrix, size_t dimension);

    /**
     * @brief 矩阵求逆（部分主元 Gauss-Jordan）
     * @param matrix    输入矩阵（会被修改）
     * @param inverse   输出逆矩阵
     * @param dimension 维度
     * @return 执行状态
     * @note 若奇异则返回 SINGULAR_MATRIX。
     */
    alg_lqr_status_t alg_lqr_internal_invert(float *matrix, float *inverse, size_t dimension);

    /**
     * @brief 执行一步 Riccati 迭代计算 P_k 和 K
     * @param state_dimension      n
     * @param control_dimension    m
     * @param state_matrix         A（n×n）
     * @param control_matrix       B（n×m）
     * @param state_weight         Q（n×n）
     * @param control_weight       R（m×m）
     * @param cross_weight         N（n×m），可为 NULL
     * @param next_riccati         P_{k+1}（n×n）
     * @param current_riccati      输出 P_k（n×n）
     * @param gain_matrix          输出 K（m×n）
     * @param workspace            工作区
     * @param workspace_size       工作区大小
     * @return 执行状态
     * @note 计算公式见 DARE 说明。
     */
    alg_lqr_status_t alg_lqr_internal_riccati_step(
        size_t state_dimension, size_t control_dimension, const float *state_matrix,
        const float *control_matrix, const float *state_weight, const float *control_weight,
        const float *cross_weight, const float *next_riccati, float *current_riccati,
        float *gain_matrix, float *workspace, size_t workspace_size);

#ifdef __cplusplus
}
#endif

#endif /* ALG_LQR_INTERNAL_H */