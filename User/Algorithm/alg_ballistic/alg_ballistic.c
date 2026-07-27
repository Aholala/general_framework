#include "alg_ballistic.h"

#include <math.h>

static bool alg_ballistic_simulate(const alg_ballistic_t *me, float speed_m_per_s, float pitch_rad,
                                   float horizontal_distance_m, float *height_m,
                                   float *flight_time_s)
{
    float position_x_m = 0.0F;
    float position_z_m = 0.0F;
    float velocity_x_m_per_s = speed_m_per_s * cosf(pitch_rad);
    float velocity_z_m_per_s = speed_m_per_s * sinf(pitch_rad);
    float elapsed_time_s = 0.0F;

    while ((position_x_m < horizontal_distance_m) &&
           (elapsed_time_s < me->config.maximum_flight_time_s) && (velocity_x_m_per_s > 0.01F))
    {
        const float step_s = me->config.integration_step_s;
        const float velocity_magnitude = hypotf(velocity_x_m_per_s, velocity_z_m_per_s);
        const float drag_x =
            me->config.drag_coefficient_per_m * velocity_magnitude * velocity_x_m_per_s;
        const float drag_z =
            me->config.drag_coefficient_per_m * velocity_magnitude * velocity_z_m_per_s;
        velocity_x_m_per_s -= drag_x * step_s;
        velocity_z_m_per_s -= (me->config.gravity_m_per_s2 + drag_z) * step_s;
        position_x_m += velocity_x_m_per_s * step_s;
        position_z_m += velocity_z_m_per_s * step_s;
        elapsed_time_s += step_s;
    }
    if (position_x_m < horizontal_distance_m)
    {
        return false;
    }
    *height_m = position_z_m;
    *flight_time_s = elapsed_time_s;
    return true;
}

alg_ballistic_status_t alg_ballistic_init(alg_ballistic_t *me, const alg_ballistic_config_t *config)
{
    if ((me == NULL) || (config == NULL) || !isfinite(config->gravity_m_per_s2) ||
        !isfinite(config->drag_coefficient_per_m) || !isfinite(config->integration_step_s) ||
        !isfinite(config->maximum_flight_time_s) || !isfinite(config->pitch_min_rad) ||
        !isfinite(config->pitch_max_rad) || !isfinite(config->height_tolerance_m) ||
        (config->gravity_m_per_s2 <= 0.0F) || (config->drag_coefficient_per_m < 0.0F) ||
        (config->integration_step_s <= 0.0F) || (config->maximum_flight_time_s <= 0.0F) ||
        (config->pitch_max_rad <= config->pitch_min_rad) || (config->maximum_iterations == 0U) ||
        (config->height_tolerance_m <= 0.0F))
    {
        return ALG_BALLISTIC_STATUS_INVALID_ARGUMENT;
    }
    me->config = *config;
    me->is_initialized = true;
    return ALG_BALLISTIC_STATUS_OK;
}

alg_ballistic_status_t alg_ballistic_solve(const alg_ballistic_t *me,
                                           const alg_ballistic_target_t *target,
                                           alg_ballistic_solution_t *solution)
{
    float lower_pitch;
    float upper_pitch;
    float flight_time_s = 0.0F;
    float simulated_height_m = 0.0F;
    float desired_height_m;
    float effective_horizontal_distance_m;
    unsigned int iteration;

    if ((me == NULL) || (target == NULL) || (solution == NULL) ||
        !isfinite(target->horizontal_distance_m) || !isfinite(target->vertical_distance_m) ||
        !isfinite(target->target_velocity_x_m_per_s) ||
        !isfinite(target->target_velocity_y_m_per_s) ||
        !isfinite(target->target_velocity_z_m_per_s) ||
        !isfinite(target->projectile_speed_m_per_s) || !isfinite(target->system_delay_s) ||
        (target->horizontal_distance_m <= 0.0F) || (target->projectile_speed_m_per_s <= 0.0F) ||
        (target->system_delay_s < 0.0F))
    {
        return ALG_BALLISTIC_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_BALLISTIC_STATUS_NOT_INITIALIZED;
    }
    lower_pitch = me->config.pitch_min_rad;
    upper_pitch = me->config.pitch_max_rad;
    effective_horizontal_distance_m =
        target->horizontal_distance_m +
        target->target_velocity_x_m_per_s *
            (target->horizontal_distance_m / target->projectile_speed_m_per_s +
             target->system_delay_s);
    if (effective_horizontal_distance_m <= 0.0F)
    {
        return ALG_BALLISTIC_STATUS_NO_SOLUTION;
    }
    for (iteration = 0U; iteration < me->config.maximum_iterations; ++iteration)
    {
        const float pitch_rad = 0.5F * (lower_pitch + upper_pitch);
        if (!alg_ballistic_simulate(me, target->projectile_speed_m_per_s, pitch_rad,
                                    effective_horizontal_distance_m, &simulated_height_m,
                                    &flight_time_s))
        {
            return ALG_BALLISTIC_STATUS_NO_SOLUTION;
        }
        desired_height_m =
            target->vertical_distance_m +
            target->target_velocity_z_m_per_s * (flight_time_s + target->system_delay_s);
        if (fabsf(simulated_height_m - desired_height_m) <= me->config.height_tolerance_m)
        {
            const float total_delay_s = flight_time_s + target->system_delay_s;
            const float lead_lateral_m = target->target_velocity_y_m_per_s * total_delay_s;
            solution->pitch_rad = pitch_rad;
            solution->yaw_lead_rad = atan2f(lead_lateral_m, effective_horizontal_distance_m);
            solution->flight_time_s = flight_time_s;
            solution->predicted_drop_m = desired_height_m - simulated_height_m;
            return ALG_BALLISTIC_STATUS_OK;
        }
        if (simulated_height_m < desired_height_m)
        {
            lower_pitch = pitch_rad;
        }
        else
        {
            upper_pitch = pitch_rad;
        }
    }
    return ALG_BALLISTIC_STATUS_NO_SOLUTION;
}
