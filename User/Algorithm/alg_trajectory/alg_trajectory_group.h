#ifndef ALG_TRAJECTORY_GROUP_H
#define ALG_TRAJECTORY_GROUP_H

#include "alg_trajectory.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        alg_trajectory_config_t *axis_configs;
        alg_trajectory_state_t *axis_states;
        float *start_positions;
        float *target_positions;
        size_t axis_count;
        float elapsed_time_s;
        float duration_s;
        bool is_finished;
        bool is_initialized;
    } alg_trajectory_group_t;

    alg_trajectory_status_t alg_trajectory_group_init(
        alg_trajectory_group_t *me, alg_trajectory_config_t *axis_config_storage,
        alg_trajectory_state_t *axis_state_storage, float *start_position_storage,
        float *target_position_storage, size_t axis_count,
        const alg_trajectory_state_t *initial_states, const alg_trajectory_config_t *axis_configs);
    alg_trajectory_status_t alg_trajectory_group_set_target(alg_trajectory_group_t *me,
                                                            const float *target_positions);
    alg_trajectory_status_t alg_trajectory_group_update(alg_trajectory_group_t *me,
                                                        float delta_time_s);
    const alg_trajectory_state_t *alg_trajectory_group_get_state(const alg_trajectory_group_t *me,
                                                                 size_t axis_index);
    bool alg_trajectory_group_is_finished(const alg_trajectory_group_t *me);

#ifdef __cplusplus
}
#endif

#endif
