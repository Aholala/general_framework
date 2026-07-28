/**
 * @file alg_chassis_motion.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘运动学公共数学内核实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 包含加权最小二乘速度求解（QR 分解）、坐标变换、旋转中心转换、
 *       轮速缩放和里程计积分。
 */

#include "alg_chassis_motion.h"

#include <math.h>   // sqrtf, hypotf, sinf, cosf, fabsf, isfinite
#include <stddef.h> // NULL

/* ======================== 内部常量 ======================== */

/** @brief 速度分量数量（vx, vy, wz） */
#define ALG_CHASSIS_COMPONENT_COUNT (3U)
/** @brief QR 分解对角线容差，用于判断是否奇异 */
#define ALG_CHASSIS_QR_DIAGONAL_TOLERANCE (1.0E-6F)

/* ======================== 内部工具函数 ======================== */

/**
 * @brief 校验速度是否有效
 * @param velocity 速度指针
 * @return true=有效
 * @note 检查指针非空且所有分量为有限数
 */
static bool alg_chassis_velocity_is_valid(const alg_chassis_velocity_t *velocity)
{
    return (velocity != NULL) && isfinite(velocity->velocity_x_m_per_s) &&
           isfinite(velocity->velocity_y_m_per_s) && isfinite(velocity->angular_velocity_rad_per_s);
}

/**
 * @brief 获取速度分量
 * @param velocity 速度指针
 * @param component_index 分量索引（0=vx, 1=vy, 2=wz）
 * @return 分量值
 */
static float alg_chassis_get_velocity_component(const alg_chassis_velocity_t *velocity,
                                                size_t component_index)
{
    if (component_index == 0U)
        return velocity->velocity_x_m_per_s;
    if (component_index == 1U)
        return velocity->velocity_y_m_per_s;
    return velocity->angular_velocity_rad_per_s;
}

/**
 * @brief 设置速度分量
 * @param velocity 速度指针
 * @param component_index 分量索引（0=vx, 1=vy, 2=wz）
 * @param value 要设置的值
 */
static void alg_chassis_set_velocity_component(alg_chassis_velocity_t *velocity,
                                               size_t component_index, float value)
{
    if (component_index == 0U)
        velocity->velocity_x_m_per_s = value;
    else if (component_index == 1U)
        velocity->velocity_y_m_per_s = value;
    else
        velocity->angular_velocity_rad_per_s = value;
}

/**
 * @brief 获取约束系数
 * @param constraint 约束指针
 * @param component_index 分量索引
 * @return 系数值
 */
static float alg_chassis_get_constraint_coefficient(const alg_chassis_constraint_t *constraint,
                                                    size_t component_index)
{
    if (component_index == 0U)
        return constraint->velocity_x_coefficient;
    if (component_index == 1U)
        return constraint->velocity_y_coefficient;
    return constraint->angular_velocity_coefficient_m;
}

/**
 * @brief 校验约束是否有效
 * @param constraint 约束指针
 * @return true=有效
 * @note 检查所有字段为有限数且 weight >= 0
 */
static bool alg_chassis_constraint_is_valid(const alg_chassis_constraint_t *constraint)
{
    return isfinite(constraint->velocity_x_coefficient) &&
           isfinite(constraint->velocity_y_coefficient) &&
           isfinite(constraint->angular_velocity_coefficient_m) &&
           isfinite(constraint->measured_velocity_m_per_s) && isfinite(constraint->weight) &&
           (constraint->weight >= 0.0F);
}

/* ======================== QR 分解相关函数 ======================== */

/**
 * @brief 使用 Givens 旋转向 QR 分解添加一行
 * @param upper_triangular 上三角矩阵（3×3），会被修改
 * @param transformed_vector 变换后的向量，会被修改
 * @param row 要添加的行
 * @param measured_value 测量值
 * @param order 矩阵阶数
 * @return true=成功
 * @note 增量式 QR 分解，逐行构建 R 矩阵和 Q^T b 向量
 *       每次添加一行，通过 Givens 旋转保持上三角形式
 */
static bool alg_chassis_qr_add_row(float upper_triangular[3][3], float transformed_vector[3],
                                   float row[3], float measured_value, size_t order)
{
    size_t diagonal_index;

    for (diagonal_index = 0U; diagonal_index < order; ++diagonal_index)
    {
        // 当前对角线元素（R 矩阵）和传入行对应元素
        const float existing_value = upper_triangular[diagonal_index][diagonal_index];
        const float incoming_value = row[diagonal_index];
        // 计算 Givens 旋转的模长（斜边长度）
        const float hypotenuse = hypotf(existing_value, incoming_value);
        float cosine;
        float sine;
        size_t column_index;

        // 检查模长是否有效
        if (!isfinite(hypotenuse))
            return false;
        if (hypotenuse == 0.0F)
            continue; // 该列为零，跳过（无需旋转）

        // 计算旋转参数：cos = a/h，sin = b/h
        cosine = existing_value / hypotenuse;
        sine = incoming_value / hypotenuse;

        // 应用 Givens 旋转到当前行（R 矩阵）
        // 只处理 diagonal_index 及之后的列
        for (column_index = diagonal_index; column_index < order; ++column_index)
        {
            const float existing_column_value = upper_triangular[diagonal_index][column_index];
            const float incoming_column_value = row[column_index];
            upper_triangular[diagonal_index][column_index] =
                cosine * existing_column_value + sine * incoming_column_value;
            row[column_index] = -sine * existing_column_value + cosine * incoming_column_value;
        }

        // 应用 Givens 旋转到 RHS 向量（Q^T b）
        {
            const float existing_vector_value = transformed_vector[diagonal_index];
            transformed_vector[diagonal_index] =
                cosine * existing_vector_value + sine * measured_value;
            measured_value = -sine * existing_vector_value + cosine * measured_value;
        }
    }
    return true;
}

/**
 * @brief QR 分解回代求解
 * @param upper_triangular 上三角矩阵
 * @param transformed_vector 变换后的 RHS 向量
 * @param order 矩阵阶数
 * @param solution 输出解
 * @return true=成功
 */
static bool alg_chassis_qr_back_substitute(float upper_triangular[3][3],
                                           const float transformed_vector[3], size_t order,
                                           float solution[3])
{
    size_t diagonal_index;

    // 检查对角线是否有效（非零有限数）
    for (diagonal_index = 0U; diagonal_index < order; ++diagonal_index)
    {
        if (!isfinite(upper_triangular[diagonal_index][diagonal_index]) ||
            (fabsf(upper_triangular[diagonal_index][diagonal_index]) <=
             ALG_CHASSIS_QR_DIAGONAL_TOLERANCE))
        {
            return false; // 对角线为零或接近零 → 奇异矩阵
        }
    }

    // 从最后一行开始回代（从下往上）
    for (diagonal_index = order; diagonal_index > 0U; --diagonal_index)
    {
        const size_t row_index = diagonal_index - 1U;
        float value = transformed_vector[row_index];
        size_t column_index;

        // 减去已知项（已求解的变量）
        for (column_index = row_index + 1U; column_index < order; ++column_index)
        {
            value -= upper_triangular[row_index][column_index] * solution[column_index];
        }

        // 除以对角线元素
        solution[row_index] = value / upper_triangular[row_index][row_index];
        if (!isfinite(solution[row_index]))
            return false;
    }
    return true;
}

/* ======================== 公共 API ======================== */

/**
 * @brief 加权最小二乘求解底盘速度（核心函数）
 * @param constraints 约束数组
 * @param constraint_count 约束数量
 * @param known_component_mask 已知分量掩码
 * @param known_velocity 已知速度分量
 * @param nominal_constraint_count 名义约束数
 * @param solution 输出求解结果
 * @return 执行状态
 * @note 算法步骤：
 *       1. 确定未知分量
 *       2. 计算列缩放因子（数值稳定性）
 *       3. 构建加权最小二乘法方程
 *       4. 使用 QR 分解求解
 *       5. 计算残差 RMS
 *       6. 判断是否降级
 */
alg_chassis_status_t alg_chassis_solve_velocity(const alg_chassis_constraint_t *constraints,
                                                size_t constraint_count,
                                                uint8_t known_component_mask,
                                                const alg_chassis_velocity_t *known_velocity,
                                                size_t nominal_constraint_count,
                                                alg_chassis_solution_t *solution)
{
    float upper_triangular[3][3] = {{0.0F}};    // R 矩阵（上三角）
    float transformed_vector[3] = {0.0F};       // Q^T b
    float unknown_solution[3] = {0.0F};         // 归一化解
    float column_scales[3] = {0.0F};            // 列缩放因子
    size_t unknown_component_indices[3] = {0U}; // 未知分量索引
    size_t unknown_component_count = 0U;        // 未知分量数量
    size_t used_constraint_count = 0U;          // 实际使用约束数
    size_t constraint_index;
    size_t component_index;
    float squared_residual_sum = 0.0F;

    // ---- 1. 参数校验 ----
    if ((constraints == NULL) || (constraint_count == 0U) || (solution == NULL) ||
        ((known_component_mask & (uint8_t)(~ALG_CHASSIS_COMPONENT_ALL)) != 0U) ||
        ((known_component_mask != 0U) && !alg_chassis_velocity_is_valid(known_velocity)) ||
        (nominal_constraint_count == 0U))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    // ---- 2. 初始化解结构 ----
    *solution = (alg_chassis_solution_t){0};
    if (known_velocity != NULL)
        solution->velocity = *known_velocity;

    // ---- 3. 确定未知分量 ----
    for (component_index = 0U; component_index < ALG_CHASSIS_COMPONENT_COUNT; ++component_index)
    {
        if ((known_component_mask & (1U << component_index)) == 0U)
            unknown_component_indices[unknown_component_count++] = component_index;
    }
    solution->unknown_component_count = unknown_component_count;
    if (unknown_component_count == 0U)
        return ALG_CHASSIS_STATUS_OK; // 所有分量已知，无需求解

    // ---- 4. 计算列缩放因子（用于数值稳定性） ----
    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *constraint = &constraints[constraint_index];
        size_t unknown_index;

        if (!alg_chassis_constraint_is_valid(constraint))
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        if (!constraint->is_available || (constraint->weight == 0.0F))
            continue; // 跳过不可用或权重为 0 的约束

        // 累加列缩放：weight * coefficient^2
        for (unknown_index = 0U; unknown_index < unknown_component_count; ++unknown_index)
        {
            const float coefficient = alg_chassis_get_constraint_coefficient(
                constraint, unknown_component_indices[unknown_index]);
            column_scales[unknown_index] += constraint->weight * coefficient * coefficient;
        }
        ++used_constraint_count;
    }
    solution->used_constraint_count = used_constraint_count;

    // ---- 5. 检查是否欠定 ----
    if (used_constraint_count < unknown_component_count)
        return ALG_CHASSIS_STATUS_UNDERDETERMINED;

    // ---- 6. 归一化列缩放 ----
    for (component_index = 0U; component_index < unknown_component_count; ++component_index)
    {
        column_scales[component_index] = sqrtf(column_scales[component_index]);
        if (!isfinite(column_scales[component_index]) || (column_scales[component_index] <= 0.0F))
            return ALG_CHASSIS_STATUS_SINGULAR;
    }

    // ---- 7. 构建加权最小二乘矩阵（QR 分解） ----
    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *constraint = &constraints[constraint_index];
        float adjusted_measurement;
        float weighted_row[3] = {0.0F};
        float square_root_weight;
        size_t unknown_index;

        if (!constraint->is_available || (constraint->weight == 0.0F))
            continue;

        // 调整测量值：减去已知分量的贡献
        adjusted_measurement = constraint->measured_velocity_m_per_s;
        for (component_index = 0U; component_index < ALG_CHASSIS_COMPONENT_COUNT; ++component_index)
        {
            if ((known_component_mask & (1U << component_index)) != 0U)
            {
                adjusted_measurement -=
                    alg_chassis_get_constraint_coefficient(constraint, component_index) *
                    alg_chassis_get_velocity_component(known_velocity, component_index);
            }
        }

        // 加权
        square_root_weight = sqrtf(constraint->weight);
        adjusted_measurement *= square_root_weight;

        // 构建加权行（已归一化列缩放）
        for (unknown_index = 0U; unknown_index < unknown_component_count; ++unknown_index)
        {
            weighted_row[unknown_index] =
                square_root_weight *
                alg_chassis_get_constraint_coefficient(constraint,
                                                       unknown_component_indices[unknown_index]) /
                column_scales[unknown_index];
        }

        // 添加到 QR 分解
        if (!alg_chassis_qr_add_row(upper_triangular, transformed_vector, weighted_row,
                                    adjusted_measurement, unknown_component_count))
            return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
    }

    // ---- 8. 回代求解 ----
    if (!alg_chassis_qr_back_substitute(upper_triangular, transformed_vector,
                                        unknown_component_count, unknown_solution))
        return ALG_CHASSIS_STATUS_SINGULAR;

    // ---- 9. 还原缩放并更新速度 ----
    for (component_index = 0U; component_index < unknown_component_count; ++component_index)
    {
        alg_chassis_set_velocity_component(
            &solution->velocity, unknown_component_indices[component_index],
            unknown_solution[component_index] / column_scales[component_index]);
    }

    // ---- 10. 计算残差均方根 ----
    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *constraint = &constraints[constraint_index];
        float predicted_velocity;
        float residual;

        if (!constraint->is_available || (constraint->weight == 0.0F))
            continue;

        // 预测速度 = sum(系数 * 速度分量)
        predicted_velocity =
            constraint->velocity_x_coefficient * solution->velocity.velocity_x_m_per_s +
            constraint->velocity_y_coefficient * solution->velocity.velocity_y_m_per_s +
            constraint->angular_velocity_coefficient_m *
                solution->velocity.angular_velocity_rad_per_s;

        // 残差 = 预测 - 测量
        residual = predicted_velocity - constraint->measured_velocity_m_per_s;
        squared_residual_sum += residual * residual;
    }
    solution->residual_root_mean_square_m_per_s =
        sqrtf(squared_residual_sum / (float)used_constraint_count);

    // ---- 11. 检查结果有效性 ----
    if (!alg_chassis_velocity_is_valid(&solution->velocity) ||
        !isfinite(solution->residual_root_mean_square_m_per_s))
        return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;

    // ---- 12. 判断是否降级 ----
    solution->is_degraded = used_constraint_count < nominal_constraint_count;
    return solution->is_degraded ? ALG_CHASSIS_STATUS_DEGRADED : ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 计算约束残差
 * @param constraints 约束数组
 * @param constraint_count 约束数量
 * @param velocity 速度值
 * @param residuals_m_per_s 输出残差数组（m/s）
 * @param residual_capacity 残差数组容量
 * @return 执行状态
 */
alg_chassis_status_t alg_chassis_calculate_constraint_residuals(
    const alg_chassis_constraint_t *constraints, size_t constraint_count,
    const alg_chassis_velocity_t *velocity, float *residuals_m_per_s, size_t residual_capacity)
{
    size_t constraint_index;

    // ---- 参数校验 ----
    if ((constraints == NULL) || (constraint_count == 0U) ||
        !alg_chassis_velocity_is_valid(velocity) || (residuals_m_per_s == NULL) ||
        (residual_capacity < constraint_count))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *constraint = &constraints[constraint_index];
        float predicted_velocity;

        if (!alg_chassis_constraint_is_valid(constraint))
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

        // 不可用或权重为零的约束残差置零
        if (!constraint->is_available || (constraint->weight == 0.0F))
        {
            residuals_m_per_s[constraint_index] = 0.0F;
            continue;
        }

        // 预测速度
        predicted_velocity =
            constraint->velocity_x_coefficient * velocity->velocity_x_m_per_s +
            constraint->velocity_y_coefficient * velocity->velocity_y_m_per_s +
            constraint->angular_velocity_coefficient_m * velocity->angular_velocity_rad_per_s;

        // 残差 = 预测 - 测量
        residuals_m_per_s[constraint_index] =
            predicted_velocity - constraint->measured_velocity_m_per_s;
        if (!isfinite(residuals_m_per_s[constraint_index]))
            return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
    }
    return ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 将参考坐标系速度变换到车体坐标系
 * @param reference_velocity 参考坐标系速度
 * @param reference_heading_rad 参考航向角
 * @param body_velocity 输出车体速度
 * @return 执行状态
 * @note 使用二维旋转矩阵将速度从参考系旋转到车体系
 *       [vx']   [cos  sin] [vx]
 *       [vy'] = [-sin cos] [vy]
 */
alg_chassis_status_t
alg_chassis_transform_reference_to_body(const alg_chassis_velocity_t *reference_velocity,
                                        float reference_heading_rad,
                                        alg_chassis_velocity_t *body_velocity)
{
    float heading_cosine;
    float heading_sine;

    if (!alg_chassis_velocity_is_valid(reference_velocity) || !isfinite(reference_heading_rad) ||
        (body_velocity == NULL))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    heading_cosine = cosf(reference_heading_rad);
    heading_sine = sinf(reference_heading_rad);

    // 二维旋转：将参考系速度旋转到车体系
    // 注意：旋转方向为从参考系到车体系（逆时针旋转 heading 角）
    body_velocity->velocity_x_m_per_s = heading_cosine * reference_velocity->velocity_x_m_per_s +
                                        heading_sine * reference_velocity->velocity_y_m_per_s;
    body_velocity->velocity_y_m_per_s = -heading_sine * reference_velocity->velocity_x_m_per_s +
                                        heading_cosine * reference_velocity->velocity_y_m_per_s;

    // 角速度保持不变（标量，不受旋转影响）
    body_velocity->angular_velocity_rad_per_s = reference_velocity->angular_velocity_rad_per_s;
    return ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 将旋转中心处的速度转换到车体原点
 * @param center_velocity 旋转中心处速度
 * @param center_of_rotation_x_m 旋转中心 X 坐标
 * @param center_of_rotation_y_m 旋转中心 Y 坐标
 * @param origin_velocity 输出原点速度
 * @return 执行状态
 * @note v_origin = v_center + w × (-r_center)
 *       公式：v_origin.x = v_center.x + w * y_center
 *             v_origin.y = v_center.y - w * x_center
 */
alg_chassis_status_t alg_chassis_convert_center_velocity_to_origin(
    const alg_chassis_velocity_t *center_velocity, float center_of_rotation_x_m,
    float center_of_rotation_y_m, alg_chassis_velocity_t *origin_velocity)
{
    if (!alg_chassis_velocity_is_valid(center_velocity) || !isfinite(center_of_rotation_x_m) ||
        !isfinite(center_of_rotation_y_m) || (origin_velocity == NULL))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    // v_origin = v_center + w × (-r_center)
    // 其中 r_center 是旋转中心相对原点的位置
    // 叉积展开：(w × r) = (-w * y, w * x)
    // 所以 v_origin = v_center + (-w * y, w * x)
    origin_velocity->velocity_x_m_per_s =
        center_velocity->velocity_x_m_per_s +
        center_velocity->angular_velocity_rad_per_s * center_of_rotation_y_m;
    origin_velocity->velocity_y_m_per_s =
        center_velocity->velocity_y_m_per_s -
        center_velocity->angular_velocity_rad_per_s * center_of_rotation_x_m;
    origin_velocity->angular_velocity_rad_per_s = center_velocity->angular_velocity_rad_per_s;

    return alg_chassis_velocity_is_valid(origin_velocity) ? ALG_CHASSIS_STATUS_OK
                                                          : ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
}

/**
 * @brief 统一缩放轮速
 * @param wheel_velocities 轮速数组（会被修改）
 * @param wheel_is_available 轮子可用性数组
 * @param wheel_count 轮子数量
 * @param maximum_absolute_velocity 最大允许绝对速度
 * @param applied_scale 输出实际缩放系数
 * @return 执行状态
 */
alg_chassis_status_t alg_chassis_scale_wheel_velocities(float *wheel_velocities,
                                                        const bool *wheel_is_available,
                                                        size_t wheel_count,
                                                        float maximum_absolute_velocity,
                                                        float *applied_scale)
{
    float maximum_calculated_velocity = 0.0F;
    float scale = 1.0F;
    size_t wheel_index;
    size_t available_wheel_count = 0U;

    // ---- 参数校验 ----
    if ((wheel_velocities == NULL) || (wheel_count == 0U) || !isfinite(maximum_absolute_velocity) ||
        (maximum_absolute_velocity <= 0.0F))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    // ---- 1. 计算最大轮速 ----
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        const bool is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index];
        if (!isfinite(wheel_velocities[wheel_index]))
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

        if (!is_available)
        {
            wheel_velocities[wheel_index] = 0.0F; // 不可用轮子归零
            continue;
        }
        ++available_wheel_count;
        if (fabsf(wheel_velocities[wheel_index]) > maximum_calculated_velocity)
            maximum_calculated_velocity = fabsf(wheel_velocities[wheel_index]);
    }
    if (available_wheel_count == 0U)
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    // ---- 2. 计算缩放系数 ----
    // 如果最大轮速超过限制，统一缩小所有轮速
    if (maximum_calculated_velocity > maximum_absolute_velocity)
        scale = maximum_absolute_velocity / maximum_calculated_velocity;

    // ---- 3. 应用缩放 ----
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
        wheel_velocities[wheel_index] *= scale;

    if (applied_scale != NULL)
        *applied_scale = scale;

    // 有轮子不可用 → 降级
    return (available_wheel_count < wheel_count) ? ALG_CHASSIS_STATUS_DEGRADED
                                                 : ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 里程计积分
 * @param me 位姿对象（会被修改）
 * @param body_velocity 车体速度
 * @param delta_time_s 时间步长（秒）
 * @param integration_method 积分方法
 * @return 执行状态
 * @note Euler：直接使用当前航向
 *       Midpoint：使用半步后的航向（更精确）
 *       Exact：恒定速度模型下的精确圆弧积分
 */
alg_chassis_status_t
alg_chassis_integrate_odometry(alg_chassis_pose_t *me, const alg_chassis_velocity_t *body_velocity,
                               float delta_time_s,
                               alg_chassis_integration_method_t integration_method)
{
    float body_displacement_x_m;
    float body_displacement_y_m;
    float integration_heading_rad;
    float heading_change_rad;
    float heading_cosine;
    float heading_sine;

    // ---- 参数校验 ----
    if ((me == NULL) || !alg_chassis_velocity_is_valid(body_velocity) ||
        !isfinite(me->position_x_m) || !isfinite(me->position_y_m) || !isfinite(me->heading_rad) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F) ||
        (integration_method > ALG_CHASSIS_INTEGRATION_EXACT))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    // ---- 1. 计算航向变化 ----
    heading_change_rad = body_velocity->angular_velocity_rad_per_s * delta_time_s;
    integration_heading_rad = me->heading_rad;

    // ---- 2. 计算车体位移 ----
    if (integration_method == ALG_CHASSIS_INTEGRATION_EXACT)
    {
        // 精确积分：恒定速度模型下的精确解（圆弧轨迹）
        if (fabsf(body_velocity->angular_velocity_rad_per_s) > 1.0E-6F)
        {
            const float inverse_angular_velocity_s =
                1.0F / body_velocity->angular_velocity_rad_per_s;
            const float heading_change_sine = sinf(heading_change_rad);
            const float heading_change_cosine = cosf(heading_change_rad);
            // 圆弧积分公式
            // dx = (sin(Δθ)*vx - (1-cos(Δθ))*vy) / w
            // dy = ((1-cos(Δθ))*vx + sin(Δθ)*vy) / w
            body_displacement_x_m =
                inverse_angular_velocity_s *
                (heading_change_sine * body_velocity->velocity_x_m_per_s -
                 (1.0F - heading_change_cosine) * body_velocity->velocity_y_m_per_s);
            body_displacement_y_m =
                inverse_angular_velocity_s *
                ((1.0F - heading_change_cosine) * body_velocity->velocity_x_m_per_s +
                 heading_change_sine * body_velocity->velocity_y_m_per_s);
        }
        else
        {
            // 角速度接近零时退化为直线
            body_displacement_x_m = body_velocity->velocity_x_m_per_s * delta_time_s;
            body_displacement_y_m = body_velocity->velocity_y_m_per_s * delta_time_s;
        }
    }
    else
    {
        // Euler 或 Midpoint：直线位移近似
        body_displacement_x_m = body_velocity->velocity_x_m_per_s * delta_time_s;
        body_displacement_y_m = body_velocity->velocity_y_m_per_s * delta_time_s;
        if (integration_method == ALG_CHASSIS_INTEGRATION_MIDPOINT)
        {
            // 中点积分：使用半个时间步后的航向
            integration_heading_rad += heading_change_rad * 0.5F;
        }
    }

    // ---- 3. 将车体位移变换到参考坐标系 ----
    // 使用积分航向（Euler 用起始航向，Midpoint 用中点航向）
    heading_cosine = cosf(integration_heading_rad);
    heading_sine = sinf(integration_heading_rad);
    me->position_x_m +=
        heading_cosine * body_displacement_x_m - heading_sine * body_displacement_y_m;
    me->position_y_m +=
        heading_sine * body_displacement_x_m + heading_cosine * body_displacement_y_m;
    me->heading_rad += heading_change_rad;

    // ---- 4. 检查结果有效性 ----
    return (isfinite(me->position_x_m) && isfinite(me->position_y_m) && isfinite(me->heading_rad))
               ? ALG_CHASSIS_STATUS_OK
               : ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
}