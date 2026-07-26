#include "alg_differential.h"

#include <math.h>
#include <stddef.h>

static bool
alg_differential_wheel_config_is_valid(const alg_differential_wheel_config_t *wheel_config)
{
    return (wheel_config->side <= ALG_DIFFERENTIAL_SIDE_RIGHT) &&
           isfinite(wheel_config->wheel_radius_m) && (wheel_config->wheel_radius_m > 0.0F) &&
           ((wheel_config->direction_sign == 1.0F) || (wheel_config->direction_sign == -1.0F)) &&
           isfinite(wheel_config->odometry_weight) && (wheel_config->odometry_weight > 0.0F);
}

alg_chassis_status_t alg_differential_init(alg_differential_t *me,
                                           const alg_differential_wheel_config_t *wheel_configs,
                                           size_t wheel_count, float track_width_m,
                                           float maximum_wheel_angular_velocity_rad_per_s)
{
    bool has_left_wheel = false;
    bool has_right_wheel = false;
    size_t wheel_index;

    if ((me == NULL) || (wheel_configs == NULL) ||
        (wheel_count < ALG_DIFFERENTIAL_TWO_WHEEL_COUNT) || !isfinite(track_width_m) ||
        (track_width_m <= 0.0F) || !isfinite(maximum_wheel_angular_velocity_rad_per_s) ||
        (maximum_wheel_angular_velocity_rad_per_s <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        if (!alg_differential_wheel_config_is_valid(&wheel_configs[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        has_left_wheel =
            has_left_wheel || (wheel_configs[wheel_index].side == ALG_DIFFERENTIAL_SIDE_LEFT);
        has_right_wheel =
            has_right_wheel || (wheel_configs[wheel_index].side == ALG_DIFFERENTIAL_SIDE_RIGHT);
    }
    if (!has_left_wheel || !has_right_wheel)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->wheel_configs = wheel_configs;
    me->wheel_count = wheel_count;
    me->track_width_m = track_width_m;
    me->maximum_wheel_angular_velocity_rad_per_s = maximum_wheel_angular_velocity_rad_per_s;
    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_differential_inverse(const alg_differential_t *me,
                                              const alg_chassis_velocity_t *chassis_velocity,
                                              const bool *wheel_is_available,
                                              float *wheel_angular_velocities_rad_per_s,
                                              size_t output_capacity, float *applied_scale)
{
    return alg_differential_inverse_with_lateral_center_of_rotation(
        me, chassis_velocity, 0.0F, wheel_is_available, wheel_angular_velocities_rad_per_s,
        output_capacity, applied_scale);
}

alg_chassis_status_t alg_differential_inverse_with_lateral_center_of_rotation(
    const alg_differential_t *me, const alg_chassis_velocity_t *center_velocity,
    float center_of_rotation_y_m, const bool *wheel_is_available,
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
    if ((output_capacity < me->wheel_count) ||
        (fabsf(center_velocity->velocity_y_m_per_s) > 1.0E-6F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    status = alg_chassis_convert_center_velocity_to_origin(
        center_velocity, 0.0F, center_of_rotation_y_m, &origin_velocity);
    if (status != ALG_CHASSIS_STATUS_OK)
    {
        return status;
    }
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_differential_wheel_config_t *const wheel_config = &me->wheel_configs[wheel_index];
        const float side_sign = (wheel_config->side == ALG_DIFFERENTIAL_SIDE_LEFT) ? -1.0F : 1.0F;
        const float wheel_linear_velocity_m_per_s =
            origin_velocity.velocity_x_m_per_s +
            side_sign * (me->track_width_m * 0.5F) * origin_velocity.angular_velocity_rad_per_s;
        wheel_angular_velocities_rad_per_s[wheel_index] = wheel_linear_velocity_m_per_s /
                                                          wheel_config->wheel_radius_m *
                                                          wheel_config->direction_sign;
    }
    return alg_chassis_scale_wheel_velocities(
        wheel_angular_velocities_rad_per_s, wheel_is_available, me->wheel_count,
        me->maximum_wheel_angular_velocity_rad_per_s, applied_scale);
}

alg_chassis_status_t alg_differential_forward(
    const alg_differential_t *me, const float *wheel_angular_velocities_rad_per_s,
    const bool *wheel_is_available, uint8_t known_component_mask,
    const alg_chassis_velocity_t *known_velocity, alg_chassis_constraint_t *constraint_workspace,
    size_t workspace_capacity, alg_chassis_solution_t *solution)
{
    alg_chassis_velocity_t effective_known_velocity = {
        .velocity_x_m_per_s = 0.0F,
        .velocity_y_m_per_s = 0.0F,
        .angular_velocity_rad_per_s = 0.0F,
    };
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
    if ((known_velocity == NULL) &&
        ((known_component_mask &
          (ALG_CHASSIS_COMPONENT_VELOCITY_X | ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY)) != 0U))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (known_velocity != NULL)
    {
        effective_known_velocity = *known_velocity;
    }
    effective_known_velocity.velocity_y_m_per_s = 0.0F;
    known_component_mask |= ALG_CHASSIS_COMPONENT_VELOCITY_Y;
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_differential_wheel_config_t *const wheel_config = &me->wheel_configs[wheel_index];
        const float side_sign = (wheel_config->side == ALG_DIFFERENTIAL_SIDE_LEFT) ? -1.0F : 1.0F;
        if (!isfinite(wheel_angular_velocities_rad_per_s[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        constraint_workspace[wheel_index] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 1.0F,
            .velocity_y_coefficient = 0.0F,
            .angular_velocity_coefficient_m = side_sign * me->track_width_m * 0.5F,
            .measured_velocity_m_per_s = wheel_angular_velocities_rad_per_s[wheel_index] *
                                         wheel_config->wheel_radius_m *
                                         wheel_config->direction_sign,
            .weight = wheel_config->odometry_weight,
            .is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index],
        };
    }
    return alg_chassis_solve_velocity(constraint_workspace, me->wheel_count, known_component_mask,
                                      &effective_known_velocity, me->wheel_count, solution);
}
