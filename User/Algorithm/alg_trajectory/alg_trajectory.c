#include "alg_trajectory.h"

#include <math.h>
#include <stddef.h>

static float alg_trajectory_clamp(float value, float minimum_value, float maximum_value)
{
    if (value < minimum_value)
    {
        return minimum_value;
    }
    if (value > maximum_value)
    {
        return maximum_value;
    }
    return value;
}

static float alg_trajectory_sign(float value)
{
    if (value > 0.0F)
    {
        return 1.0F;
    }
    if (value < 0.0F)
    {
        return -1.0F;
    }
    return 0.0F;
}

static bool alg_trajectory_state_is_valid(const alg_trajectory_state_t *state)
{
    return (state != NULL) && isfinite(state->position) && isfinite(state->velocity_per_s) &&
           isfinite(state->acceleration_per_s2);
}

static bool alg_trajectory_config_is_valid(const alg_trajectory_config_t *config)
{
    return (config != NULL) && isfinite(config->maximum_velocity_per_s) &&
           (config->maximum_velocity_per_s > 0.0F) &&
           isfinite(config->maximum_acceleration_per_s2) &&
           (config->maximum_acceleration_per_s2 > 0.0F) &&
           isfinite(config->maximum_deceleration_per_s2) &&
           (config->maximum_deceleration_per_s2 > 0.0F) && isfinite(config->maximum_jerk_per_s3) &&
           (config->maximum_jerk_per_s3 > 0.0F) && isfinite(config->position_tolerance) &&
           (config->position_tolerance >= 0.0F) && isfinite(config->velocity_tolerance_per_s) &&
           (config->velocity_tolerance_per_s >= 0.0F);
}

static float alg_trajectory_calculate_position_acceleration(const alg_trajectory_t *me)
{
    const float position_error = me->target_position - me->state.position;
    const float direction = alg_trajectory_sign(position_error);
    const float directed_velocity = me->state.velocity_per_s * direction;
    const float terminal_velocity = me->target_velocity_per_s * direction;
    float braking_distance;

    if (direction == 0.0F)
    {
        return -alg_trajectory_sign(me->state.velocity_per_s) *
               me->config.maximum_deceleration_per_s2;
    }
    if (directed_velocity < 0.0F)
    {
        return direction * me->config.maximum_deceleration_per_s2;
    }
    braking_distance =
        ((directed_velocity * directed_velocity) - (terminal_velocity * terminal_velocity)) /
        (2.0F * me->config.maximum_deceleration_per_s2);
    if (braking_distance < 0.0F)
    {
        braking_distance = 0.0F;
    }
    if (braking_distance >= fabsf(position_error))
    {
        return -direction * me->config.maximum_deceleration_per_s2;
    }
    if (directed_velocity < me->config.maximum_velocity_per_s)
    {
        return direction * me->config.maximum_acceleration_per_s2;
    }
    return 0.0F;
}

static float alg_trajectory_calculate_velocity_acceleration(const alg_trajectory_t *me)
{
    const float velocity_error = me->target_velocity_per_s - me->state.velocity_per_s;
    const float acceleration_limit = (me->state.velocity_per_s * velocity_error < 0.0F)
                                         ? me->config.maximum_deceleration_per_s2
                                         : me->config.maximum_acceleration_per_s2;
    return alg_trajectory_sign(velocity_error) * acceleration_limit;
}

static float alg_trajectory_apply_jerk_limit(const alg_trajectory_t *me, float target_acceleration,
                                             float delta_time_s)
{
    const float maximum_change = me->config.maximum_jerk_per_s3 * delta_time_s;
    return me->state.acceleration_per_s2 +
           alg_trajectory_clamp(target_acceleration - me->state.acceleration_per_s2,
                                -maximum_change, maximum_change);
}

float alg_trajectory_calculate_stopping_distance(float velocity_per_s, float deceleration_per_s2)
{
    if (!isfinite(velocity_per_s) || !isfinite(deceleration_per_s2) ||
        (deceleration_per_s2 <= 0.0F))
    {
        return NAN;
    }
    return (velocity_per_s * fabsf(velocity_per_s)) / (2.0F * deceleration_per_s2);
}

alg_trajectory_status_t alg_trajectory_init(alg_trajectory_t *me,
                                            const alg_trajectory_config_t *config,
                                            alg_trajectory_profile_t profile,
                                            const alg_trajectory_state_t *initial_state)
{
    if ((me == NULL) || !alg_trajectory_config_is_valid(config) ||
        !alg_trajectory_state_is_valid(initial_state) || (profile > ALG_TRAJECTORY_PROFILE_S_CURVE))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    *me = (alg_trajectory_t){0};
    me->config = *config;
    me->state = *initial_state;
    me->target_position = initial_state->position;
    me->target_velocity_per_s = initial_state->velocity_per_s;
    me->profile = profile;
    me->target_type = ALG_TRAJECTORY_TARGET_POSITION;
    me->is_finished = true;
    me->is_initialized = true;
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_reset(alg_trajectory_t *me,
                                             const alg_trajectory_state_t *state)
{
    if ((me == NULL) || !alg_trajectory_state_is_valid(state))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    me->state = *state;
    me->target_position = state->position;
    me->target_velocity_per_s = state->velocity_per_s;
    me->target_type = ALG_TRAJECTORY_TARGET_POSITION;
    me->is_finished = true;
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_set_position_target(alg_trajectory_t *me,
                                                           float target_position,
                                                           float terminal_velocity_per_s)
{
    if ((me == NULL) || !isfinite(target_position) || !isfinite(terminal_velocity_per_s))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    if (fabsf(terminal_velocity_per_s) > me->config.maximum_velocity_per_s)
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    me->target_position = target_position;
    me->target_velocity_per_s = terminal_velocity_per_s;
    me->target_type = ALG_TRAJECTORY_TARGET_POSITION;
    me->is_finished = false;
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_set_velocity_target(alg_trajectory_t *me,
                                                           float target_velocity_per_s)
{
    if ((me == NULL) || !isfinite(target_velocity_per_s))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    if (fabsf(target_velocity_per_s) > me->config.maximum_velocity_per_s)
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    me->target_velocity_per_s = target_velocity_per_s;
    me->target_type = ALG_TRAJECTORY_TARGET_VELOCITY;
    me->is_finished = false;
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_set_profile(alg_trajectory_t *me,
                                                   alg_trajectory_profile_t profile)
{
    if ((me == NULL) || (profile > ALG_TRAJECTORY_PROFILE_S_CURVE))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    me->profile = profile;
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_update(alg_trajectory_t *me, float delta_time_s,
                                              alg_trajectory_state_t *output_state)
{
    float target_acceleration;
    float new_acceleration;
    float new_velocity;
    float new_position;

    if ((me == NULL) || (output_state == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    target_acceleration = (me->target_type == ALG_TRAJECTORY_TARGET_POSITION)
                              ? alg_trajectory_calculate_position_acceleration(me)
                              : alg_trajectory_calculate_velocity_acceleration(me);
    new_acceleration = (me->profile == ALG_TRAJECTORY_PROFILE_S_CURVE)
                           ? alg_trajectory_apply_jerk_limit(me, target_acceleration, delta_time_s)
                           : target_acceleration;
    new_acceleration =
        alg_trajectory_clamp(new_acceleration, -me->config.maximum_deceleration_per_s2,
                             me->config.maximum_acceleration_per_s2);
    new_velocity = me->state.velocity_per_s +
                   0.5F * (me->state.acceleration_per_s2 + new_acceleration) * delta_time_s;
    new_velocity = alg_trajectory_clamp(new_velocity, -me->config.maximum_velocity_per_s,
                                        me->config.maximum_velocity_per_s);
    new_position =
        me->state.position + 0.5F * (me->state.velocity_per_s + new_velocity) * delta_time_s;

    me->state.position = new_position;
    me->state.velocity_per_s = new_velocity;
    me->state.acceleration_per_s2 = new_acceleration;
    if (!alg_trajectory_state_is_valid(&me->state))
    {
        return ALG_TRAJECTORY_STATUS_NUMERICAL_ERROR;
    }

    if (me->target_type == ALG_TRAJECTORY_TARGET_POSITION)
    {
        const bool position_reached =
            fabsf(me->target_position - me->state.position) <= me->config.position_tolerance;
        const bool velocity_reached = fabsf(me->target_velocity_per_s - me->state.velocity_per_s) <=
                                      me->config.velocity_tolerance_per_s;
        if (position_reached && velocity_reached &&
            (fabsf(me->state.acceleration_per_s2) <= me->config.maximum_jerk_per_s3 * delta_time_s))
        {
            me->state.position = me->target_position;
            me->state.velocity_per_s = me->target_velocity_per_s;
            me->state.acceleration_per_s2 = 0.0F;
            me->is_finished = true;
        }
    }
    else if (fabsf(me->target_velocity_per_s - me->state.velocity_per_s) <=
             me->config.velocity_tolerance_per_s)
    {
        me->state.velocity_per_s = me->target_velocity_per_s;
        if ((me->profile == ALG_TRAJECTORY_PROFILE_TRAPEZOIDAL) ||
            (fabsf(me->state.acceleration_per_s2) <= me->config.maximum_jerk_per_s3 * delta_time_s))
        {
            me->state.acceleration_per_s2 = 0.0F;
            me->is_finished = true;
        }
    }
    *output_state = me->state;
    return me->is_finished ? ALG_TRAJECTORY_STATUS_FINISHED : ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_get_state(const alg_trajectory_t *me,
                                                 alg_trajectory_state_t *state)
{
    if ((me == NULL) || (state == NULL))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    *state = me->state;
    return ALG_TRAJECTORY_STATUS_OK;
}

bool alg_trajectory_is_finished(const alg_trajectory_t *me)
{
    return (me != NULL) && me->is_initialized && me->is_finished;
}
