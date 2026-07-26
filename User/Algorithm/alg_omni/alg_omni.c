#include "alg_omni.h"

#include <math.h>
#include <stddef.h>

static bool alg_omni_wheel_config_is_valid(const alg_omni_wheel_config_t *wheel_config)
{
    return isfinite(wheel_config->position_x_m) && isfinite(wheel_config->position_y_m) &&
           isfinite(wheel_config->drive_direction_rad) && isfinite(wheel_config->wheel_radius_m) &&
           (wheel_config->wheel_radius_m > 0.0F) &&
           ((wheel_config->direction_sign == 1.0F) || (wheel_config->direction_sign == -1.0F)) &&
           isfinite(wheel_config->odometry_weight) && (wheel_config->odometry_weight > 0.0F);
}

static void alg_omni_get_constraint_coefficients(const alg_omni_wheel_config_t *wheel_config,
                                                 float *velocity_x_coefficient,
                                                 float *velocity_y_coefficient,
                                                 float *angular_velocity_coefficient_m)
{
    *velocity_x_coefficient = cosf(wheel_config->drive_direction_rad);
    *velocity_y_coefficient = sinf(wheel_config->drive_direction_rad);
    *angular_velocity_coefficient_m = (-*velocity_x_coefficient * wheel_config->position_y_m) +
                                      (*velocity_y_coefficient * wheel_config->position_x_m);
}

alg_chassis_status_t alg_omni_configure_tangential_layout(
    alg_omni_wheel_config_t *wheel_configs, size_t wheel_count, float center_to_wheel_distance_m,
    float wheel_radius_m, float first_wheel_position_angle_rad, float tangential_direction_sign,
    const float *wheel_direction_signs, float odometry_weight)
{
    const float full_circle_rad = 6.28318530717958647692F;
    const float quarter_turn_rad = 1.57079632679489661923F;
    size_t wheel_index;

    if ((wheel_configs == NULL) || (wheel_count < 2U) || !isfinite(center_to_wheel_distance_m) ||
        (center_to_wheel_distance_m <= 0.0F) || !isfinite(wheel_radius_m) ||
        (wheel_radius_m <= 0.0F) || !isfinite(first_wheel_position_angle_rad) ||
        ((tangential_direction_sign != 1.0F) && (tangential_direction_sign != -1.0F)) ||
        !isfinite(odometry_weight) || (odometry_weight <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        const float motor_direction_sign =
            (wheel_direction_signs == NULL) ? 1.0F : wheel_direction_signs[wheel_index];
        const float position_angle_rad = first_wheel_position_angle_rad +
                                         full_circle_rad * (float)wheel_index / (float)wheel_count;
        if ((motor_direction_sign != 1.0F) && (motor_direction_sign != -1.0F))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        wheel_configs[wheel_index] = (alg_omni_wheel_config_t){
            .position_x_m = center_to_wheel_distance_m * cosf(position_angle_rad),
            .position_y_m = center_to_wheel_distance_m * sinf(position_angle_rad),
            .drive_direction_rad =
                position_angle_rad + tangential_direction_sign * quarter_turn_rad,
            .wheel_radius_m = wheel_radius_m,
            .direction_sign = motor_direction_sign,
            .odometry_weight = odometry_weight,
        };
    }
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_omni_init(alg_omni_t *me, const alg_omni_wheel_config_t *wheel_configs,
                                   size_t wheel_count,
                                   float maximum_wheel_angular_velocity_rad_per_s)
{
    size_t wheel_index;

    if ((me == NULL) || (wheel_configs == NULL) || (wheel_count == 0U) ||
        !isfinite(maximum_wheel_angular_velocity_rad_per_s) ||
        (maximum_wheel_angular_velocity_rad_per_s <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        if (!alg_omni_wheel_config_is_valid(&wheel_configs[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
    }
    me->wheel_configs = wheel_configs;
    me->wheel_count = wheel_count;
    me->maximum_wheel_angular_velocity_rad_per_s = maximum_wheel_angular_velocity_rad_per_s;
    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_omni_inverse(const alg_omni_t *me,
                                      const alg_chassis_velocity_t *chassis_velocity,
                                      const bool *wheel_is_available,
                                      float *wheel_angular_velocities_rad_per_s,
                                      size_t output_capacity, float *applied_scale)
{
    return alg_omni_inverse_with_center_of_rotation(
        me, chassis_velocity, 0.0F, 0.0F, wheel_is_available, wheel_angular_velocities_rad_per_s,
        output_capacity, applied_scale);
}

alg_chassis_status_t alg_omni_inverse_with_center_of_rotation(
    const alg_omni_t *me, const alg_chassis_velocity_t *center_velocity,
    float center_of_rotation_x_m, float center_of_rotation_y_m, const bool *wheel_is_available,
    float *wheel_angular_velocities_rad_per_s, size_t output_capacity, float *applied_scale)
{
    alg_chassis_velocity_t origin_velocity;
    alg_chassis_status_t status;
    size_t wheel_index;

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
    if (output_capacity < me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    status = alg_chassis_convert_center_velocity_to_origin(
        center_velocity, center_of_rotation_x_m, center_of_rotation_y_m, &origin_velocity);
    if (status != ALG_CHASSIS_STATUS_OK)
    {
        return status;
    }
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_omni_wheel_config_t *const wheel_config = &me->wheel_configs[wheel_index];
        float velocity_x_coefficient;
        float velocity_y_coefficient;
        float angular_velocity_coefficient_m;
        float wheel_linear_velocity_m_per_s;

        alg_omni_get_constraint_coefficients(wheel_config, &velocity_x_coefficient,
                                             &velocity_y_coefficient,
                                             &angular_velocity_coefficient_m);
        wheel_linear_velocity_m_per_s =
            velocity_x_coefficient * origin_velocity.velocity_x_m_per_s +
            velocity_y_coefficient * origin_velocity.velocity_y_m_per_s +
            angular_velocity_coefficient_m * origin_velocity.angular_velocity_rad_per_s;
        wheel_angular_velocities_rad_per_s[wheel_index] = wheel_linear_velocity_m_per_s /
                                                          wheel_config->wheel_radius_m *
                                                          wheel_config->direction_sign;
    }
    return alg_chassis_scale_wheel_velocities(
        wheel_angular_velocities_rad_per_s, wheel_is_available, me->wheel_count,
        me->maximum_wheel_angular_velocity_rad_per_s, applied_scale);
}

alg_chassis_status_t alg_omni_forward(const alg_omni_t *me,
                                      const float *wheel_angular_velocities_rad_per_s,
                                      const bool *wheel_is_available, uint8_t known_component_mask,
                                      const alg_chassis_velocity_t *known_velocity,
                                      alg_chassis_constraint_t *constraint_workspace,
                                      size_t workspace_capacity, alg_chassis_solution_t *solution)
{
    size_t wheel_index;

    if ((me == NULL) || (wheel_angular_velocities_rad_per_s == NULL) ||
        (constraint_workspace == NULL) || (solution == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (workspace_capacity < me->wheel_count)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_omni_wheel_config_t *const wheel_config = &me->wheel_configs[wheel_index];
        float velocity_x_coefficient;
        float velocity_y_coefficient;
        float angular_velocity_coefficient_m;

        if (!isfinite(wheel_angular_velocities_rad_per_s[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        alg_omni_get_constraint_coefficients(wheel_config, &velocity_x_coefficient,
                                             &velocity_y_coefficient,
                                             &angular_velocity_coefficient_m);
        constraint_workspace[wheel_index] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = velocity_x_coefficient,
            .velocity_y_coefficient = velocity_y_coefficient,
            .angular_velocity_coefficient_m = angular_velocity_coefficient_m,
            .measured_velocity_m_per_s = wheel_angular_velocities_rad_per_s[wheel_index] *
                                         wheel_config->wheel_radius_m *
                                         wheel_config->direction_sign,
            .weight = wheel_config->odometry_weight,
            .is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index],
        };
    }
    return alg_chassis_solve_velocity(constraint_workspace, me->wheel_count, known_component_mask,
                                      known_velocity, me->wheel_count, solution);
}
