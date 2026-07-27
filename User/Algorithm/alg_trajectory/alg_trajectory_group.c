#include "alg_trajectory_group.h"

#include <math.h>

static float alg_trajectory_group_maximum(float left, float right)
{
    return (left > right) ? left : right;
}

static float alg_trajectory_group_axis_duration(float distance,
                                                const alg_trajectory_config_t *config)
{
    const float absolute_distance = fabsf(distance);
    const float velocity_duration = 1.875F * absolute_distance / config->maximum_velocity_per_s;
    const float acceleration_duration =
        sqrtf(5.773503F * absolute_distance / config->maximum_acceleration_per_s2);
    const float jerk_duration = cbrtf(60.0F * absolute_distance / config->maximum_jerk_per_s3);
    return alg_trajectory_group_maximum(
        velocity_duration, alg_trajectory_group_maximum(acceleration_duration, jerk_duration));
}

alg_trajectory_status_t alg_trajectory_group_init(alg_trajectory_group_t *me,
                                                  alg_trajectory_config_t *axis_config_storage,
                                                  alg_trajectory_state_t *axis_state_storage,
                                                  float *start_position_storage,
                                                  float *target_position_storage, size_t axis_count,
                                                  const alg_trajectory_state_t *initial_states,
                                                  const alg_trajectory_config_t *axis_configs)
{
    size_t axis_index;
    if ((me == NULL) || (axis_config_storage == NULL) || (axis_state_storage == NULL) ||
        (start_position_storage == NULL) || (target_position_storage == NULL) ||
        (axis_count == 0U) || (initial_states == NULL) || (axis_configs == NULL))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    for (axis_index = 0U; axis_index < axis_count; ++axis_index)
    {
        if (!isfinite(initial_states[axis_index].position) ||
            (axis_configs[axis_index].maximum_velocity_per_s <= 0.0F) ||
            (axis_configs[axis_index].maximum_acceleration_per_s2 <= 0.0F) ||
            (axis_configs[axis_index].maximum_jerk_per_s3 <= 0.0F))
        {
            return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
        }
        axis_config_storage[axis_index] = axis_configs[axis_index];
        axis_state_storage[axis_index] = initial_states[axis_index];
        start_position_storage[axis_index] = initial_states[axis_index].position;
        target_position_storage[axis_index] = initial_states[axis_index].position;
    }
    *me = (alg_trajectory_group_t){
        .axis_configs = axis_config_storage,
        .axis_states = axis_state_storage,
        .start_positions = start_position_storage,
        .target_positions = target_position_storage,
        .axis_count = axis_count,
        .is_finished = true,
        .is_initialized = true,
    };
    return ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_group_set_target(alg_trajectory_group_t *me,
                                                        const float *target_positions)
{
    size_t axis_index;
    float duration_s = 0.0F;
    if ((me == NULL) || (target_positions == NULL))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    for (axis_index = 0U; axis_index < me->axis_count; ++axis_index)
    {
        float axis_duration_s;
        if (!isfinite(target_positions[axis_index]))
        {
            return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
        }
        me->start_positions[axis_index] = me->axis_states[axis_index].position;
        me->target_positions[axis_index] = target_positions[axis_index];
        axis_duration_s = alg_trajectory_group_axis_duration(target_positions[axis_index] -
                                                                 me->start_positions[axis_index],
                                                             &me->axis_configs[axis_index]);
        duration_s = alg_trajectory_group_maximum(duration_s, axis_duration_s);
    }
    me->elapsed_time_s = 0.0F;
    me->duration_s = duration_s;
    me->is_finished = duration_s <= 1.0e-6F;
    return me->is_finished ? ALG_TRAJECTORY_STATUS_FINISHED : ALG_TRAJECTORY_STATUS_OK;
}

alg_trajectory_status_t alg_trajectory_group_update(alg_trajectory_group_t *me, float delta_time_s)
{
    float normalized_time;
    float normalized_time2;
    float normalized_time3;
    float normalized_time4;
    float normalized_time5;
    float position_scale;
    float velocity_scale;
    float acceleration_scale;
    size_t axis_index;
    if ((me == NULL) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return ALG_TRAJECTORY_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_TRAJECTORY_STATUS_NOT_INITIALIZED;
    }
    if (me->is_finished)
    {
        return ALG_TRAJECTORY_STATUS_FINISHED;
    }
    me->elapsed_time_s = fminf(me->elapsed_time_s + delta_time_s, me->duration_s);
    normalized_time = me->elapsed_time_s / me->duration_s;
    normalized_time2 = normalized_time * normalized_time;
    normalized_time3 = normalized_time2 * normalized_time;
    normalized_time4 = normalized_time3 * normalized_time;
    normalized_time5 = normalized_time4 * normalized_time;
    position_scale = 10.0F * normalized_time3 - 15.0F * normalized_time4 + 6.0F * normalized_time5;
    velocity_scale =
        (30.0F * normalized_time2 - 60.0F * normalized_time3 + 30.0F * normalized_time4) /
        me->duration_s;
    acceleration_scale =
        (60.0F * normalized_time - 180.0F * normalized_time2 + 120.0F * normalized_time3) /
        (me->duration_s * me->duration_s);
    for (axis_index = 0U; axis_index < me->axis_count; ++axis_index)
    {
        const float distance = me->target_positions[axis_index] - me->start_positions[axis_index];
        me->axis_states[axis_index].position =
            me->start_positions[axis_index] + distance * position_scale;
        me->axis_states[axis_index].velocity_per_s = distance * velocity_scale;
        me->axis_states[axis_index].acceleration_per_s2 = distance * acceleration_scale;
    }
    me->is_finished = me->elapsed_time_s >= me->duration_s;
    return me->is_finished ? ALG_TRAJECTORY_STATUS_FINISHED : ALG_TRAJECTORY_STATUS_OK;
}

const alg_trajectory_state_t *alg_trajectory_group_get_state(const alg_trajectory_group_t *me,
                                                             size_t axis_index)
{
    return ((me != NULL) && me->is_initialized && (axis_index < me->axis_count))
               ? &me->axis_states[axis_index]
               : NULL;
}

bool alg_trajectory_group_is_finished(const alg_trajectory_group_t *me)
{
    return (me != NULL) && me->is_initialized && me->is_finished;
}
