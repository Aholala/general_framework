#include "alg_ackermann.h"

#include <math.h>
#include <stddef.h>

#define ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE (1.0E-6F)

static bool alg_ackermann_config_is_valid(
    const alg_ackermann_config_t *config)
{
    size_t wheel_index;

    if (!isfinite(config->wheelbase_m) ||
        (config->wheelbase_m <= 0.0F) ||
        !isfinite(config->track_width_m) ||
        (config->track_width_m <= 0.0F) ||
        !isfinite(config->lateral_constraint_weight) ||
        (config->lateral_constraint_weight <= 0.0F) ||
        !isfinite(config->maximum_steering_angle_rad) ||
        (config->maximum_steering_angle_rad <= 0.0F) ||
        (config->maximum_steering_angle_rad >= 1.5707963267948966F) ||
        !isfinite(config->maximum_wheel_angular_velocity_rad_per_s) ||
        (config->maximum_wheel_angular_velocity_rad_per_s <= 0.0F))
    {
        return false;
    }
    for (wheel_index = 0U; wheel_index < ALG_ACKERMANN_WHEEL_COUNT;
         ++wheel_index)
    {
        if (!isfinite(config->wheel_radius_m[wheel_index]) ||
            (config->wheel_radius_m[wheel_index] <= 0.0F) ||
            ((config->direction_sign[wheel_index] != 1.0F) &&
             (config->direction_sign[wheel_index] != -1.0F)) ||
            !isfinite(config->odometry_weight[wheel_index]) ||
            (config->odometry_weight[wheel_index] <= 0.0F))
        {
            return false;
        }
    }
    return true;
}

alg_chassis_status_t alg_ackermann_init(
    alg_ackermann_t *me, const alg_ackermann_config_t *config)
{
    const float half_track_width_m =
        (config != NULL) ? config->track_width_m * 0.5F : 0.0F;

    if ((me == NULL) || (config == NULL) ||
        !alg_ackermann_config_is_valid(config))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    me->is_initialized = false;
    me->config = *config;
    me->wheel_position_x_m[ALG_ACKERMANN_WHEEL_FRONT_LEFT] =
        config->wheelbase_m;
    me->wheel_position_x_m[ALG_ACKERMANN_WHEEL_FRONT_RIGHT] =
        config->wheelbase_m;
    me->wheel_position_x_m[ALG_ACKERMANN_WHEEL_REAR_LEFT] = 0.0F;
    me->wheel_position_x_m[ALG_ACKERMANN_WHEEL_REAR_RIGHT] = 0.0F;
    me->wheel_position_y_m[ALG_ACKERMANN_WHEEL_FRONT_LEFT] =
        half_track_width_m;
    me->wheel_position_y_m[ALG_ACKERMANN_WHEEL_REAR_LEFT] =
        half_track_width_m;
    me->wheel_position_y_m[ALG_ACKERMANN_WHEEL_FRONT_RIGHT] =
        -half_track_width_m;
    me->wheel_position_y_m[ALG_ACKERMANN_WHEEL_REAR_RIGHT] =
        -half_track_width_m;
    me->is_initialized = true;
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_ackermann_inverse(
    const alg_ackermann_t *me,
    const alg_chassis_velocity_t *chassis_velocity,
    const bool wheel_is_available[ALG_ACKERMANN_WHEEL_COUNT],
    alg_ackermann_wheel_state_t
        wheel_states[ALG_ACKERMANN_WHEEL_COUNT],
    float *applied_scale)
{
    float wheel_angular_velocities[ALG_ACKERMANN_WHEEL_COUNT];
    float curvature_per_m;
    float half_track_width_m;
    float left_denominator;
    float right_denominator;
    float left_steering_angle_rad;
    float right_steering_angle_rad;
    float left_rear_velocity_m_per_s;
    float right_rear_velocity_m_per_s;
    size_t wheel_index;
    alg_chassis_status_t status;

    if ((me == NULL) || (chassis_velocity == NULL) ||
        (wheel_states == NULL) ||
        !isfinite(chassis_velocity->velocity_x_m_per_s) ||
        !isfinite(chassis_velocity->velocity_y_m_per_s) ||
        !isfinite(chassis_velocity->angular_velocity_rad_per_s))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    if (fabsf(chassis_velocity->velocity_y_m_per_s) >
        ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (fabsf(chassis_velocity->velocity_x_m_per_s) <=
        ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE)
    {
        if (fabsf(chassis_velocity->angular_velocity_rad_per_s) >
            ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE)
        {
            return ALG_CHASSIS_STATUS_UNDERDETERMINED;
        }
        for (wheel_index = 0U;
             wheel_index < ALG_ACKERMANN_WHEEL_COUNT; ++wheel_index)
        {
            wheel_states[wheel_index] =
                (alg_ackermann_wheel_state_t){0};
        }
        if (applied_scale != NULL)
        {
            *applied_scale = 1.0F;
        }
        return ALG_CHASSIS_STATUS_OK;
    }

    curvature_per_m =
        chassis_velocity->angular_velocity_rad_per_s /
        chassis_velocity->velocity_x_m_per_s;
    half_track_width_m = me->config.track_width_m * 0.5F;
    left_denominator = 1.0F -
                       half_track_width_m * curvature_per_m;
    right_denominator = 1.0F +
                        half_track_width_m * curvature_per_m;
    if ((left_denominator <= ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE) ||
        (right_denominator <= ALG_ACKERMANN_ZERO_VELOCITY_TOLERANCE))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    left_steering_angle_rad = atanf(
        me->config.wheelbase_m * curvature_per_m /
        left_denominator);
    right_steering_angle_rad = atanf(
        me->config.wheelbase_m * curvature_per_m /
        right_denominator);
    if (!isfinite(left_steering_angle_rad) ||
        !isfinite(right_steering_angle_rad) ||
        (fabsf(left_steering_angle_rad) >
         me->config.maximum_steering_angle_rad) ||
        (fabsf(right_steering_angle_rad) >
         me->config.maximum_steering_angle_rad))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    left_rear_velocity_m_per_s =
        chassis_velocity->velocity_x_m_per_s * left_denominator;
    right_rear_velocity_m_per_s =
        chassis_velocity->velocity_x_m_per_s * right_denominator;
    wheel_states[ALG_ACKERMANN_WHEEL_FRONT_LEFT].steering_angle_rad =
        left_steering_angle_rad;
    wheel_states[ALG_ACKERMANN_WHEEL_FRONT_RIGHT].steering_angle_rad =
        right_steering_angle_rad;
    wheel_states[ALG_ACKERMANN_WHEEL_REAR_LEFT].steering_angle_rad =
        0.0F;
    wheel_states[ALG_ACKERMANN_WHEEL_REAR_RIGHT].steering_angle_rad =
        0.0F;
    wheel_angular_velocities[ALG_ACKERMANN_WHEEL_FRONT_LEFT] =
        left_rear_velocity_m_per_s /
        cosf(left_steering_angle_rad) /
        me->config.wheel_radius_m[ALG_ACKERMANN_WHEEL_FRONT_LEFT] *
        me->config.direction_sign[ALG_ACKERMANN_WHEEL_FRONT_LEFT];
    wheel_angular_velocities[ALG_ACKERMANN_WHEEL_FRONT_RIGHT] =
        right_rear_velocity_m_per_s /
        cosf(right_steering_angle_rad) /
        me->config.wheel_radius_m[ALG_ACKERMANN_WHEEL_FRONT_RIGHT] *
        me->config.direction_sign[ALG_ACKERMANN_WHEEL_FRONT_RIGHT];
    wheel_angular_velocities[ALG_ACKERMANN_WHEEL_REAR_LEFT] =
        left_rear_velocity_m_per_s /
        me->config.wheel_radius_m[ALG_ACKERMANN_WHEEL_REAR_LEFT] *
        me->config.direction_sign[ALG_ACKERMANN_WHEEL_REAR_LEFT];
    wheel_angular_velocities[ALG_ACKERMANN_WHEEL_REAR_RIGHT] =
        right_rear_velocity_m_per_s /
        me->config.wheel_radius_m[ALG_ACKERMANN_WHEEL_REAR_RIGHT] *
        me->config.direction_sign[ALG_ACKERMANN_WHEEL_REAR_RIGHT];
    status = alg_chassis_scale_wheel_velocities(
        wheel_angular_velocities, wheel_is_available,
        ALG_ACKERMANN_WHEEL_COUNT,
        me->config.maximum_wheel_angular_velocity_rad_per_s,
        applied_scale);
    if ((status != ALG_CHASSIS_STATUS_OK) &&
        (status != ALG_CHASSIS_STATUS_DEGRADED))
    {
        return status;
    }
    for (wheel_index = 0U; wheel_index < ALG_ACKERMANN_WHEEL_COUNT;
         ++wheel_index)
    {
        wheel_states[wheel_index].wheel_angular_velocity_rad_per_s =
            wheel_angular_velocities[wheel_index];
        if ((wheel_is_available != NULL) &&
            !wheel_is_available[wheel_index])
        {
            wheel_states[wheel_index].steering_angle_rad = 0.0F;
        }
    }
    return status;
}

alg_chassis_status_t alg_ackermann_forward(
    const alg_ackermann_t *me,
    const alg_ackermann_wheel_state_t
        wheel_states[ALG_ACKERMANN_WHEEL_COUNT],
    const bool wheel_is_available[ALG_ACKERMANN_WHEEL_COUNT],
    uint8_t known_component_mask,
    const alg_chassis_velocity_t *known_velocity,
    alg_chassis_constraint_t
        constraint_workspace[ALG_ACKERMANN_CONSTRAINT_COUNT],
    alg_chassis_solution_t *solution)
{
    size_t wheel_index;

    if ((me == NULL) || (wheel_states == NULL) ||
        (constraint_workspace == NULL) || (solution == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_STATUS_NOT_INITIALIZED;
    }
    for (wheel_index = 0U; wheel_index < ALG_ACKERMANN_WHEEL_COUNT;
         ++wheel_index)
    {
        const float steering_angle_rad =
            wheel_states[wheel_index].steering_angle_rad;
        const float direction_x = cosf(steering_angle_rad);
        const float direction_y = sinf(steering_angle_rad);
        const float normal_x = -direction_y;
        const float normal_y = direction_x;
        const float position_x_m = me->wheel_position_x_m[wheel_index];
        const float position_y_m = me->wheel_position_y_m[wheel_index];
        const bool is_available =
            (wheel_is_available == NULL) ||
            wheel_is_available[wheel_index];

        if (!isfinite(steering_angle_rad) ||
            !isfinite(
                wheel_states[wheel_index]
                    .wheel_angular_velocity_rad_per_s) ||
            (fabsf(steering_angle_rad) >
             me->config.maximum_steering_angle_rad))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        constraint_workspace[2U * wheel_index] =
            (alg_chassis_constraint_t){
                .velocity_x_coefficient = direction_x,
                .velocity_y_coefficient = direction_y,
                .angular_velocity_coefficient_m =
                    (-direction_x * position_y_m) +
                    (direction_y * position_x_m),
                .measured_velocity_m_per_s =
                    wheel_states[wheel_index]
                        .wheel_angular_velocity_rad_per_s *
                    me->config.wheel_radius_m[wheel_index] *
                    me->config.direction_sign[wheel_index],
                .weight = me->config.odometry_weight[wheel_index],
                .is_available = is_available,
            };
        constraint_workspace[(2U * wheel_index) + 1U] =
            (alg_chassis_constraint_t){
                .velocity_x_coefficient = normal_x,
                .velocity_y_coefficient = normal_y,
                .angular_velocity_coefficient_m =
                    (-normal_x * position_y_m) +
                    (normal_y * position_x_m),
                .measured_velocity_m_per_s = 0.0F,
                .weight = me->config.lateral_constraint_weight,
                .is_available = is_available,
            };
    }
    return alg_chassis_solve_velocity(
        constraint_workspace, ALG_ACKERMANN_CONSTRAINT_COUNT,
        known_component_mask, known_velocity,
        ALG_ACKERMANN_CONSTRAINT_COUNT, solution);
}
