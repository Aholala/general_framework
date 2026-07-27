#include "alg_swerve.h"

#include <math.h>
#include <stddef.h>

#define ALG_SWERVE_PI (3.14159265358979323846F)
#define ALG_SWERVE_HALF_PI (1.57079632679489661923F)
#define ALG_SWERVE_TWO_PI (6.28318530717958647692F)

float alg_swerve_wrap_angle_rad(float angle_rad)
{
    if (!isfinite(angle_rad))
    {
        return 0.0F;
    }
    angle_rad = fmodf(angle_rad + ALG_SWERVE_PI, ALG_SWERVE_TWO_PI);
    if (angle_rad < 0.0F)
    {
        angle_rad += ALG_SWERVE_TWO_PI;
    }
    return angle_rad - ALG_SWERVE_PI;
}

alg_swerve_status_t alg_swerve_configure_rectangular_layout(
    alg_swerve_module_geometry_t module_geometry[ALG_SWERVE_RECTANGULAR_MODULE_COUNT],
    float half_wheelbase_m, float half_track_width_m)
{
    if ((module_geometry == NULL) || !isfinite(half_wheelbase_m) || (half_wheelbase_m <= 0.0F) ||
        !isfinite(half_track_width_m) || (half_track_width_m <= 0.0F))
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    module_geometry[ALG_SWERVE_MODULE_FRONT_LEFT] = (alg_swerve_module_geometry_t){
        .position_x_m = half_wheelbase_m,
        .position_y_m = half_track_width_m,
    };
    module_geometry[ALG_SWERVE_MODULE_FRONT_RIGHT] = (alg_swerve_module_geometry_t){
        .position_x_m = half_wheelbase_m,
        .position_y_m = -half_track_width_m,
    };
    module_geometry[ALG_SWERVE_MODULE_REAR_LEFT] = (alg_swerve_module_geometry_t){
        .position_x_m = -half_wheelbase_m,
        .position_y_m = half_track_width_m,
    };
    module_geometry[ALG_SWERVE_MODULE_REAR_RIGHT] = (alg_swerve_module_geometry_t){
        .position_x_m = -half_wheelbase_m,
        .position_y_m = -half_track_width_m,
    };
    return ALG_SWERVE_STATUS_OK;
}

alg_swerve_status_t alg_swerve_init(alg_swerve_t *me,
                                    const alg_swerve_module_geometry_t *module_geometry,
                                    size_t module_count, float maximum_wheel_velocity_m_per_s)
{
    size_t module_index;

    if ((me == NULL) || (module_geometry == NULL) || (module_count == 0U) ||
        !isfinite(maximum_wheel_velocity_m_per_s) || (maximum_wheel_velocity_m_per_s <= 0.0F))
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    for (module_index = 0U; module_index < module_count; ++module_index)
    {
        if (!isfinite(module_geometry[module_index].position_x_m) ||
            !isfinite(module_geometry[module_index].position_y_m))
        {
            return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
        }
    }
    me->module_geometry = module_geometry;
    me->module_count = module_count;
    me->maximum_wheel_velocity_m_per_s = maximum_wheel_velocity_m_per_s;
    me->is_initialized = true;
    return ALG_SWERVE_STATUS_OK;
}

alg_swerve_status_t alg_swerve_calculate(const alg_swerve_t *me,
                                         const alg_swerve_command_t *command,
                                         alg_swerve_module_target_t *module_targets,
                                         size_t target_capacity)
{
    return alg_swerve_calculate_with_availability(me, command, NULL, module_targets,
                                                  target_capacity);
}

alg_swerve_status_t alg_swerve_calculate_with_availability(
    const alg_swerve_t *me, const alg_swerve_command_t *command, const bool *module_is_available,
    alg_swerve_module_target_t *module_targets, size_t target_capacity)
{
    float body_velocity_x_m_per_s;
    float body_velocity_y_m_per_s;
    float maximum_calculated_velocity_m_per_s = 0.0F;
    float scale = 1.0F;
    size_t module_index;
    size_t available_module_count = 0U;

    if ((me == NULL) || (command == NULL) || (module_targets == NULL) ||
        !isfinite(command->velocity_x_m_per_s) || !isfinite(command->velocity_y_m_per_s) ||
        !isfinite(command->angular_velocity_rad_per_s) ||
        !isfinite(command->reference_heading_rad) || !isfinite(command->center_of_rotation_x_m) ||
        !isfinite(command->center_of_rotation_y_m))
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_SWERVE_STATUS_NOT_INITIALIZED;
    }
    if (target_capacity < me->module_count)
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }

    body_velocity_x_m_per_s = command->velocity_x_m_per_s;
    body_velocity_y_m_per_s = command->velocity_y_m_per_s;
    if (command->command_is_reference_relative)
    {
        const float heading_cosine = cosf(command->reference_heading_rad);
        const float heading_sine = sinf(command->reference_heading_rad);
        body_velocity_x_m_per_s = (heading_cosine * command->velocity_x_m_per_s) +
                                  (heading_sine * command->velocity_y_m_per_s);
        body_velocity_y_m_per_s = (-heading_sine * command->velocity_x_m_per_s) +
                                  (heading_cosine * command->velocity_y_m_per_s);
    }

    for (module_index = 0U; module_index < me->module_count; ++module_index)
    {
        const bool is_available =
            (module_is_available == NULL) || module_is_available[module_index];
        const float wheel_velocity_x_m_per_s =
            body_velocity_x_m_per_s -
            (command->angular_velocity_rad_per_s *
             (me->module_geometry[module_index].position_y_m - command->center_of_rotation_y_m));
        const float wheel_velocity_y_m_per_s =
            body_velocity_y_m_per_s +
            (command->angular_velocity_rad_per_s *
             (me->module_geometry[module_index].position_x_m - command->center_of_rotation_x_m));
        const float wheel_velocity_m_per_s =
            hypotf(wheel_velocity_x_m_per_s, wheel_velocity_y_m_per_s);

        module_targets[module_index].wheel_velocity_m_per_s =
            is_available ? wheel_velocity_m_per_s : 0.0F;
        module_targets[module_index].steering_angle_rad =
            (is_available && (wheel_velocity_m_per_s > 0.0F))
                ? atan2f(wheel_velocity_y_m_per_s, wheel_velocity_x_m_per_s)
                : 0.0F;
        if (is_available)
        {
            ++available_module_count;
        }
        if (is_available && (wheel_velocity_m_per_s > maximum_calculated_velocity_m_per_s))
        {
            maximum_calculated_velocity_m_per_s = wheel_velocity_m_per_s;
        }
    }
    if (maximum_calculated_velocity_m_per_s > me->maximum_wheel_velocity_m_per_s)
    {
        scale = me->maximum_wheel_velocity_m_per_s / maximum_calculated_velocity_m_per_s;
    }
    for (module_index = 0U; module_index < me->module_count; ++module_index)
    {
        module_targets[module_index].wheel_velocity_m_per_s *= scale;
    }
    if (available_module_count == 0U)
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    return (available_module_count < me->module_count) ? ALG_SWERVE_STATUS_DEGRADED
                                                       : ALG_SWERVE_STATUS_OK;
}

alg_chassis_status_t alg_swerve_forward(const alg_swerve_t *me,
                                        const alg_swerve_module_target_t *measured_module_states,
                                        const bool *module_is_available,
                                        const float *odometry_weights, uint8_t known_component_mask,
                                        const alg_chassis_velocity_t *known_velocity,
                                        alg_chassis_constraint_t *constraint_workspace,
                                        size_t workspace_capacity, alg_chassis_solution_t *solution)
{
    size_t module_index;

    if ((me == NULL) || (measured_module_states == NULL) || (constraint_workspace == NULL) ||
        (solution == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (workspace_capacity < (2U * me->module_count))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (module_index = 0U; module_index < me->module_count; ++module_index)
    {
        const alg_swerve_module_geometry_t *const geometry = &me->module_geometry[module_index];
        const alg_swerve_module_target_t *const measured_state =
            &measured_module_states[module_index];
        const bool is_available =
            (module_is_available == NULL) || module_is_available[module_index];
        const float weight = (odometry_weights == NULL) ? 1.0F : odometry_weights[module_index];
        float wheel_velocity_x_m_per_s;
        float wheel_velocity_y_m_per_s;

        if (!isfinite(measured_state->wheel_velocity_m_per_s) ||
            !isfinite(measured_state->steering_angle_rad) || !isfinite(weight) || (weight <= 0.0F))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        wheel_velocity_x_m_per_s =
            measured_state->wheel_velocity_m_per_s * cosf(measured_state->steering_angle_rad);
        wheel_velocity_y_m_per_s =
            measured_state->wheel_velocity_m_per_s * sinf(measured_state->steering_angle_rad);
        constraint_workspace[2U * module_index] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 1.0F,
            .velocity_y_coefficient = 0.0F,
            .angular_velocity_coefficient_m = -geometry->position_y_m,
            .measured_velocity_m_per_s = wheel_velocity_x_m_per_s,
            .weight = weight,
            .is_available = is_available,
        };
        constraint_workspace[(2U * module_index) + 1U] = (alg_chassis_constraint_t){
            .velocity_x_coefficient = 0.0F,
            .velocity_y_coefficient = 1.0F,
            .angular_velocity_coefficient_m = geometry->position_x_m,
            .measured_velocity_m_per_s = wheel_velocity_y_m_per_s,
            .weight = weight,
            .is_available = is_available,
        };
    }
    return alg_chassis_solve_velocity(constraint_workspace, 2U * me->module_count,
                                      known_component_mask, known_velocity, 2U * me->module_count,
                                      solution);
}

alg_swerve_status_t alg_swerve_optimize_target(float current_steering_angle_rad,
                                               alg_swerve_module_target_t *module_target)
{
    float angle_error_rad;

    if ((module_target == NULL) || !isfinite(current_steering_angle_rad) ||
        !isfinite(module_target->wheel_velocity_m_per_s) ||
        !isfinite(module_target->steering_angle_rad))
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    angle_error_rad =
        alg_swerve_wrap_angle_rad(module_target->steering_angle_rad - current_steering_angle_rad);
    if (angle_error_rad > ALG_SWERVE_HALF_PI)
    {
        module_target->steering_angle_rad =
            alg_swerve_wrap_angle_rad(module_target->steering_angle_rad - ALG_SWERVE_PI);
        module_target->wheel_velocity_m_per_s = -module_target->wheel_velocity_m_per_s;
    }
    else if (angle_error_rad < -ALG_SWERVE_HALF_PI)
    {
        module_target->steering_angle_rad =
            alg_swerve_wrap_angle_rad(module_target->steering_angle_rad + ALG_SWERVE_PI);
        module_target->wheel_velocity_m_per_s = -module_target->wheel_velocity_m_per_s;
    }
    return ALG_SWERVE_STATUS_OK;
}

alg_swerve_status_t alg_swerve_calculate_self_lock(const alg_swerve_t *me,
                                                   alg_swerve_module_target_t *module_targets,
                                                   size_t target_capacity)
{
    size_t module_index;

    if ((me == NULL) || (module_targets == NULL))
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_SWERVE_STATUS_NOT_INITIALIZED;
    }
    if (target_capacity < me->module_count)
    {
        return ALG_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    for (module_index = 0U; module_index < me->module_count; ++module_index)
    {
        module_targets[module_index].wheel_velocity_m_per_s = 0.0F;
        module_targets[module_index].steering_angle_rad =
            atan2f(-me->module_geometry[module_index].position_y_m,
                   -me->module_geometry[module_index].position_x_m);
    }
    return ALG_SWERVE_STATUS_OK;
}
