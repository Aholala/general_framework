#ifndef ALG_CHASSIS_FAULT_H
#define ALG_CHASSIS_FAULT_H

#include "alg_chassis.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        float residual_m_per_s;
        uint32_t fault_confirmation_count;
        uint32_t recovery_confirmation_count;
        bool is_faulted;
    } alg_chassis_fault_wheel_state_t;

    typedef struct
    {
        size_t wheel_count;
        float fault_residual_threshold_m_per_s;
        float recovery_residual_threshold_m_per_s;
        uint32_t fault_confirmation_samples;
        uint32_t recovery_confirmation_samples;
        alg_chassis_fault_wheel_state_t *wheel_state_storage;
    } alg_chassis_fault_config_t;

    typedef struct
    {
        size_t wheel_count;
        float fault_residual_threshold_m_per_s;
        float recovery_residual_threshold_m_per_s;
        uint32_t fault_confirmation_samples;
        uint32_t recovery_confirmation_samples;
        alg_chassis_fault_wheel_state_t *wheel_states;
        bool is_initialized;
    } alg_chassis_fault_t;

    alg_chassis_status_t alg_chassis_fault_init(
        alg_chassis_fault_t *me,
        const alg_chassis_fault_config_t *config);
    alg_chassis_status_t alg_chassis_fault_update(
        alg_chassis_fault_t *me,
        const float *wheel_residuals_m_per_s,
        const bool *sensor_is_available,
        bool *wheel_is_available, size_t output_capacity);
    alg_chassis_status_t alg_chassis_fault_reset_wheel(
        alg_chassis_fault_t *me, size_t wheel_index,
        bool assume_available);
    const alg_chassis_fault_wheel_state_t *
    alg_chassis_fault_get_wheel_state(
        const alg_chassis_fault_t *me, size_t wheel_index);

#ifdef __cplusplus
}
#endif

#endif /* ALG_CHASSIS_FAULT_H */
