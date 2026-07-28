/**
 * @file alg_swerve.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 任意数量舵轮模块运动学算法实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 实现任意布局舵轮底盘的逆解、正解、舵角优化和自锁。
 *       逆解支持参考坐标系变换、任意旋转中心和模块失效。
 *       正解将每个模块的轮速分解为 x/y 两个约束，调用加权最小二乘求解器。
 */

#include "alg_swerve.h"

#include <math.h>
#include <stddef.h>

#define ALG_SWERVE_PI (3.14159265358979323846F)
#define ALG_SWERVE_HALF_PI (1.57079632679489661923F)
#define ALG_SWERVE_TWO_PI (6.28318530717958647692F)

/**
 * @brief 角度回绕至 [-π, π)
 */
float alg_swerve_wrap_angle_rad(float angle_rad)
{
    if (!isfinite(angle_rad))
        return 0.0F;
    angle_rad = fmodf(angle_rad + ALG_SWERVE_PI, ALG_SWERVE_TWO_PI);
    if (angle_rad < 0.0F)
        angle_rad += ALG_SWERVE_TWO_PI;
    return angle_rad - ALG_SWERVE_PI;
}

/**
 * @brief 生成标准矩形四轮布局
 */
alg_swerve_status_t alg_swerve_configure_rectangular_layout(
    alg_swerve_module_geometry_t module_geometry[ALG_SWERVE_RECTANGULAR_MODULE_COUNT],
    float half_wheelbase_m, float half_track_width_m)
{
    if ((module_geometry == NULL) || !isfinite(half_wheelbase_m) || (half_wheelbase_m <= 0.0F) ||
        !isfinite(half_track_width_m) || (half_track_width_m <= 0.0F))
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;

    // ---- 左前（+x, +y） ----
    module_geometry[ALG_SWERVE_MODULE_FRONT_LEFT] = (alg_swerve_module_geometry_t){
        .position_x_m = half_wheelbase_m,
        .position_y_m = half_track_width_m,
    };
    // ---- 右前（+x, -y） ----
    module_geometry[ALG_SWERVE_MODULE_FRONT_RIGHT] = (alg_swerve_module_geometry_t){
        .position_x_m = half_wheelbase_m,
        .position_y_m = -half_track_width_m,
    };
    // ---- 左后（-x, +y） ----
    module_geometry[ALG_SWERVE_MODULE_REAR_LEFT] = (alg_swerve_module_geometry_t){
        .position_x_m = -half_wheelbase_m,
        .position_y_m = half_track_width_m,
    };
    // ---- 右后（-x, -y） ----
    module_geometry[ALG_SWERVE_MODULE_REAR_RIGHT] = (alg_swerve_module_geometry_t){
        .position_x_m = -half_wheelbase_m,
        .position_y_m = -half_track_width_m,
    };
    return ALG_SWERVE_STATUS_OK;
}

/**
 * @brief 初始化舵轮模型
 */
alg_swerve_status_t alg_swerve_init(alg_swerve_t *me,
                                    const alg_swerve_module_geometry_t *module_geometry,
                                    size_t module_count, float maximum_wheel_velocity_m_per_s)
{
    size_t i;
    if ((me == NULL) || (module_geometry == NULL) || (module_count == 0U) ||
        !isfinite(maximum_wheel_velocity_m_per_s) || (maximum_wheel_velocity_m_per_s <= 0.0F))
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;

    me->is_initialized = false;
    for (i = 0U; i < module_count; ++i)
    {
        if (!isfinite(module_geometry[i].position_x_m) ||
            !isfinite(module_geometry[i].position_y_m))
            return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    me->module_geometry = module_geometry;
    me->module_count = module_count;
    me->maximum_wheel_velocity_m_per_s = maximum_wheel_velocity_m_per_s;
    me->is_initialized = true;
    return ALG_SWERVE_STATUS_OK;
}

/**
 * @brief 逆解（所有模块可用，绕原点）
 */
alg_swerve_status_t alg_swerve_calculate(const alg_swerve_t *me,
                                         const alg_swerve_command_t *command,
                                         alg_swerve_module_target_t *module_targets,
                                         size_t target_capacity)
{
    return alg_swerve_calculate_with_availability(me, command, NULL, module_targets,
                                                  target_capacity);
}

/**
 * @brief 逆解（支持模块可用性和任意旋转中心）
 */
alg_swerve_status_t alg_swerve_calculate_with_availability(
    const alg_swerve_t *me, const alg_swerve_command_t *command, const bool *module_is_available,
    alg_swerve_module_target_t *module_targets, size_t target_capacity)
{
    float body_vx, body_vy;
    float max_calc_vel = 0.0F;
    float scale = 1.0F;
    size_t idx;
    size_t avail_count = 0U;

    // ---- 参数检查 ----
    if ((me == NULL) || (command == NULL) || (module_targets == NULL) ||
        !isfinite(command->velocity_x_m_per_s) || !isfinite(command->velocity_y_m_per_s) ||
        !isfinite(command->angular_velocity_rad_per_s) ||
        !isfinite(command->reference_heading_rad) || !isfinite(command->center_of_rotation_x_m) ||
        !isfinite(command->center_of_rotation_y_m))
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_SWERVE_STATUS_NOT_INITIALIZED;
    if (target_capacity < me->module_count)
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;

    // ---- 若命令相对参考航向，则旋转到车体系 ----
    body_vx = command->velocity_x_m_per_s;
    body_vy = command->velocity_y_m_per_s;
    if (command->command_is_reference_relative)
    {
        const float c = cosf(command->reference_heading_rad);
        const float s = sinf(command->reference_heading_rad);
        // [vx_body; vy_body] = R * [vx_ref; vy_ref]
        body_vx = c * command->velocity_x_m_per_s + s * command->velocity_y_m_per_s;
        body_vy = -s * command->velocity_x_m_per_s + c * command->velocity_y_m_per_s;
    }

    // ---- 逐模块计算 ----
    for (idx = 0U; idx < me->module_count; ++idx)
    {
        const bool available = (module_is_available == NULL) || module_is_available[idx];
        const alg_swerve_module_geometry_t *geo = &me->module_geometry[idx];

        // 计算模块在车体坐标系中的速度（刚体运动公式）
        // v_module = v_body + ω × (r_module - r_center)
        const float vx = body_vx - command->angular_velocity_rad_per_s *
                                       (geo->position_y_m - command->center_of_rotation_y_m);
        const float vy = body_vy + command->angular_velocity_rad_per_s *
                                       (geo->position_x_m - command->center_of_rotation_x_m);
        const float speed = hypotf(vx, vy);

        // 输出：可用模块输出真实值，不可用置零
        module_targets[idx].wheel_velocity_m_per_s = available ? speed : 0.0F;
        module_targets[idx].steering_angle_rad =
            (available && (speed > 0.0F)) ? atan2f(vy, vx) : 0.0F;

        if (available)
        {
            ++avail_count;
            if (speed > max_calc_vel)
                max_calc_vel = speed;
        }
    }

    // ---- 轮速统一缩放（若超限） ----
    if (max_calc_vel > me->maximum_wheel_velocity_m_per_s)
        scale = me->maximum_wheel_velocity_m_per_s / max_calc_vel;
    for (idx = 0U; idx < me->module_count; ++idx)
        module_targets[idx].wheel_velocity_m_per_s *= scale;

    // ---- 返回状态 ----
    if (avail_count == 0U)
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    return (avail_count < me->module_count) ? ALG_SWERVE_STATUS_DEGRADED : ALG_SWERVE_STATUS_OK;
}

/**
 * @brief 正运动学（加权最小二乘）
 */
alg_chassis_status_t alg_swerve_forward(const alg_swerve_t *me,
                                        const alg_swerve_module_target_t *measured_module_states,
                                        const bool *module_is_available,
                                        const float *odometry_weights, uint8_t known_component_mask,
                                        const alg_chassis_velocity_t *known_velocity,
                                        alg_chassis_constraint_t *constraint_workspace,
                                        size_t workspace_capacity, alg_chassis_solution_t *solution)
{
    size_t idx;

    // ---- 参数检查 ----
    if ((me == NULL) || (measured_module_states == NULL) || (constraint_workspace == NULL) ||
        (solution == NULL))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    if (workspace_capacity < (2U * me->module_count))
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

    // ---- 逐模块构建约束 ----
    for (idx = 0U; idx < me->module_count; ++idx)
    {
        const alg_swerve_module_geometry_t *geo = &me->module_geometry[idx];
        const alg_swerve_module_target_t *state = &measured_module_states[idx];
        const bool available = (module_is_available == NULL) || module_is_available[idx];
        const float weight = (odometry_weights == NULL) ? 1.0F : odometry_weights[idx];

        if (!isfinite(state->wheel_velocity_m_per_s) || !isfinite(state->steering_angle_rad) ||
            !isfinite(weight) || (weight <= 0.0F))
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;

        // 将轮速分解为 x 和 y 两个约束
        const float vx = state->wheel_velocity_m_per_s * cosf(state->steering_angle_rad);
        const float vy = state->wheel_velocity_m_per_s * sinf(state->steering_angle_rad);

        // 约束1：vx 分量
        constraint_workspace[2U * idx] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 1.0F,
            .velocity_y_coefficient = 0.0F,
            .angular_velocity_coefficient_m = -geo->position_y_m,
            .measured_velocity_m_per_s = vx,
            .weight = weight,
            .is_available = available,
        };
        // 约束2：vy 分量
        constraint_workspace[(2U * idx) + 1U] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 0.0F,
            .velocity_y_coefficient = 1.0F,
            .angular_velocity_coefficient_m = geo->position_x_m,
            .measured_velocity_m_per_s = vy,
            .weight = weight,
            .is_available = available,
        };
    }

    // ---- 调用通用求解器 ----
    return alg_chassis_solve_velocity(constraint_workspace, 2U * me->module_count,
                                      known_component_mask, known_velocity, 2U * me->module_count,
                                      solution);
}

/**
 * @brief 舵角最短路径优化
 */
alg_swerve_status_t alg_swerve_optimize_target(float current_steering_angle_rad,
                                               alg_swerve_module_target_t *module_target)
{
    float angle_error;

    if ((module_target == NULL) || !isfinite(current_steering_angle_rad) ||
        !isfinite(module_target->wheel_velocity_m_per_s) ||
        !isfinite(module_target->steering_angle_rad))
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;

    // ---- 计算误差并回绕到 [-π, π) ----
    angle_error =
        alg_swerve_wrap_angle_rad(module_target->steering_angle_rad - current_steering_angle_rad);

    // ---- 若误差超过 ±π/2，则反转舵角并取反轮速 ----
    if (angle_error > ALG_SWERVE_HALF_PI)
    {
        module_target->steering_angle_rad =
            alg_swerve_wrap_angle_rad(module_target->steering_angle_rad - ALG_SWERVE_PI);
        module_target->wheel_velocity_m_per_s = -module_target->wheel_velocity_m_per_s;
    }
    else if (angle_error < -ALG_SWERVE_HALF_PI)
    {
        module_target->steering_angle_rad =
            alg_swerve_wrap_angle_rad(module_target->steering_angle_rad + ALG_SWERVE_PI);
        module_target->wheel_velocity_m_per_s = -module_target->wheel_velocity_m_per_s;
    }
    return ALG_SWERVE_STATUS_OK;
}

/**
 * @brief 静止自锁
 */
alg_swerve_status_t alg_swerve_calculate_self_lock(const alg_swerve_t *me,
                                                   alg_swerve_module_target_t *module_targets,
                                                   size_t target_capacity)
{
    size_t idx;

    if ((me == NULL) || (module_targets == NULL))
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    if (!me->is_initialized)
        return ALG_SWERVE_STATUS_NOT_INITIALIZED;
    if (target_capacity < me->module_count)
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;

    // ---- 每个模块舵角指向车体原点，轮速为零 ----
    for (idx = 0U; idx < me->module_count; ++idx)
    {
        module_targets[idx].wheel_velocity_m_per_s = 0.0F;
        module_targets[idx].steering_angle_rad =
            atan2f(-me->module_geometry[idx].position_y_m, -me->module_geometry[idx].position_x_m);
    }
    return ALG_SWERVE_STATUS_OK;
}