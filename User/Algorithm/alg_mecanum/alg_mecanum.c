/**
 * @file alg_mecanum.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 四轮麦克纳姆底盘运动学算法实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现逆解、正解和任意旋转中心支持。
 *       内部使用预计算系数以提高实时性能。
 *       依赖 alg_chassis_motion 中的辅助函数：
 *         - alg_chassis_convert_center_velocity_to_origin()
 *         - alg_chassis_scale_wheel_velocities()
 *         - alg_chassis_solve_velocity()
 */

#include "alg_mecanum.h"

#include <math.h>
#include <stddef.h>

/**
 * @brief 初始化麦克纳姆底盘运动学模型
 * @param me     模型对象
 * @param config 配置
 * @return 执行状态
 */
alg_chassis_status_t alg_mecanum_init(alg_mecanum_t *me, const alg_mecanum_config_t *config)
{
    // ---- X型布局的横向系数（标准麦克纳姆） ----
    static const float x_lateral_coefficients[ALG_MECANUM_WHEEL_COUNT] = {-1.0F, 1.0F, 1.0F, -1.0F};
    // ---- X型布局的角速度符号（由轮子位置和转向决定） ----
    static const float x_angular_signs[ALG_MECANUM_WHEEL_COUNT] = {-1.0F, 1.0F, -1.0F, 1.0F};

    size_t wheel_index;
    float rotation_radius_m;

    // ---- 参数合法性检查 ----
    if ((me == NULL) || (config == NULL) || !isfinite(config->wheel_radius_m) ||
        (config->wheel_radius_m <= 0.0F) || !isfinite(config->half_wheelbase_m) ||
        (config->half_wheelbase_m <= 0.0F) || !isfinite(config->half_track_width_m) ||
        (config->half_track_width_m <= 0.0F) ||
        !isfinite(config->maximum_wheel_angular_velocity_rad_per_s) ||
        (config->maximum_wheel_angular_velocity_rad_per_s <= 0.0F) ||
        (config->roller_arrangement > ALG_MECANUM_ROLLER_O))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }

    me->is_initialized = false;
    me->config = *config;

    // ---- 计算旋转半径（用于角速度系数） ----
    rotation_radius_m = config->half_wheelbase_m + config->half_track_width_m;

    // ---- 预计算每个轮的横向和角速度系数 ----
    for (wheel_index = 0U; wheel_index < ALG_MECANUM_WHEEL_COUNT; ++wheel_index)
    {
        // 获取基础横向系数
        float lateral_coefficient = x_lateral_coefficients[wheel_index];

        // 检查方向符号是否合法
        if (((config->direction_sign[wheel_index] != 1.0F) &&
             (config->direction_sign[wheel_index] != -1.0F)) ||
            !isfinite(config->odometry_weight[wheel_index]) ||
            (config->odometry_weight[wheel_index] <= 0.0F))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }

        // 若为 O 型布局，横向系数取反
        if (config->roller_arrangement == ALG_MECANUM_ROLLER_O)
        {
            lateral_coefficient = -lateral_coefficient;
        }
        me->lateral_coefficient[wheel_index] = lateral_coefficient;

        // 角速度系数 = 符号 * 旋转半径 * 辊子布局因子（X型为 +1，O型为 -1）
        me->angular_coefficient_m[wheel_index] =
            x_angular_signs[wheel_index] * rotation_radius_m *
            ((config->roller_arrangement == ALG_MECANUM_ROLLER_X) ? 1.0F : -1.0F);
    }

    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

/**
 * @brief 逆运动学（绕底盘原点）
 */
alg_chassis_status_t
alg_mecanum_inverse(const alg_mecanum_t *me, const alg_chassis_velocity_t *chassis_velocity,
                    const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                    float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                    float *applied_scale)
{
    // ---- 直接调用带旋转中心的版本，中心为原点 ----
    return alg_mecanum_inverse_with_center_of_rotation(
        me, chassis_velocity, 0.0F, 0.0F, wheel_is_available, wheel_angular_velocities_rad_per_s,
        applied_scale);
}

/**
 * @brief 逆运动学（任意旋转中心）
 */
alg_chassis_status_t alg_mecanum_inverse_with_center_of_rotation(
    const alg_mecanum_t *me, const alg_chassis_velocity_t *center_velocity,
    float center_of_rotation_x_m, float center_of_rotation_y_m,
    const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
    float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT], float *applied_scale)
{
    alg_chassis_velocity_t origin_velocity;
    alg_chassis_status_t status;
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (center_velocity == NULL) || (wheel_angular_velocities_rad_per_s == NULL) ||
        !isfinite(center_velocity->velocity_x_m_per_s) ||
        !isfinite(center_velocity->velocity_y_m_per_s) ||
        !isfinite(center_velocity->angular_velocity_rad_per_s))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }

    // ---- 将旋转中心速度转换到底盘原点速度 ----
    status = alg_chassis_convert_center_velocity_to_origin(
        center_velocity, center_of_rotation_x_m, center_of_rotation_y_m, &origin_velocity);
    if (status != ALG_CHASSIS_STATUS_OK)
    {
        return status;
    }

    // ---- 计算每个轮的线性速度 ----
    // 公式：v_linear = vx + K_ly*vy + K_ang*wz
    for (wheel_index = 0U; wheel_index < ALG_MECANUM_WHEEL_COUNT; ++wheel_index)
    {
        const float wheel_linear_velocity_m_per_s =
            origin_velocity.velocity_x_m_per_s +
            me->lateral_coefficient[wheel_index] * origin_velocity.velocity_y_m_per_s +
            me->angular_coefficient_m[wheel_index] * origin_velocity.angular_velocity_rad_per_s;

        // ---- 转换为角速度并乘以方向符号 ----
        wheel_angular_velocities_rad_per_s[wheel_index] = wheel_linear_velocity_m_per_s /
                                                          me->config.wheel_radius_m *
                                                          me->config.direction_sign[wheel_index];
    }

    // ---- 轮速饱和缩放（保持运动方向） ----
    return alg_chassis_scale_wheel_velocities(
        wheel_angular_velocities_rad_per_s, wheel_is_available, ALG_MECANUM_WHEEL_COUNT,
        me->config.maximum_wheel_angular_velocity_rad_per_s, applied_scale);
}

/**
 * @brief 正运动学：从轮速估计底盘速度（加权最小二乘）
 */
alg_chassis_status_t
alg_mecanum_forward(const alg_mecanum_t *me,
                    const float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                    const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                    uint8_t known_component_mask, const alg_chassis_velocity_t *known_velocity,
                    alg_chassis_solution_t *solution)
{
    alg_chassis_constraint_t constraints[ALG_MECANUM_WHEEL_COUNT];
    size_t wheel_index;

    // ---- 参数检查 ----
    if ((me == NULL) || (wheel_angular_velocities_rad_per_s == NULL) || (solution == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }

    // ---- 构建每个轮的约束方程 ----
    // 约束形式：vx + K_ly*vy + K_ang*wz = measured_v
    // 其中 measured_v = wheel_omega * radius * direction_sign
    for (wheel_index = 0U; wheel_index < ALG_MECANUM_WHEEL_COUNT; ++wheel_index)
    {
        if (!isfinite(wheel_angular_velocities_rad_per_s[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }

        constraints[wheel_index] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 1.0F,
            .velocity_y_coefficient = me->lateral_coefficient[wheel_index],
            .angular_velocity_coefficient_m = me->angular_coefficient_m[wheel_index],
            .measured_velocity_m_per_s = wheel_angular_velocities_rad_per_s[wheel_index] *
                                         me->config.wheel_radius_m *
                                         me->config.direction_sign[wheel_index],
            .weight = me->config.odometry_weight[wheel_index],
            .is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index],
        };
    }

    // ---- 调用通用加权最小二乘求解器 ----
    return alg_chassis_solve_velocity(constraints, ALG_MECANUM_WHEEL_COUNT, known_component_mask,
                                      known_velocity, ALG_MECANUM_WHEEL_COUNT, solution);
}