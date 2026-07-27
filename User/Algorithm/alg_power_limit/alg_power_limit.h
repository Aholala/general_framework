#ifndef ALG_POWER_LIMIT_H
#define ALG_POWER_LIMIT_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_POWER_LIMIT_STATUS_OK = 0,
        ALG_POWER_LIMIT_STATUS_LIMITED,
        ALG_POWER_LIMIT_STATUS_INVALID_ARGUMENT,
        ALG_POWER_LIMIT_STATUS_NOT_INITIALIZED
    } alg_power_limit_status_t;

    typedef struct
    {
        float continuous_power_w;
        float peak_power_w;
        float buffer_energy_j;
        float buffer_energy_low_j;
        float buffer_energy_high_j;
        float minimum_output_scale;
        float smoothing_time_constant_s;
    } alg_power_limit_config_t;

    typedef struct
    {
        float requested_output;
        float estimated_power_w;
        float priority;
        bool is_enabled;
    } alg_power_limit_channel_input_t;

    typedef struct
    {
        float limited_output;
        float allocated_power_w;
        float scale;
    } alg_power_limit_channel_output_t;

    typedef struct
    {
        alg_power_limit_config_t config;
        float available_power_w;
        float filtered_scale;
        bool is_initialized;
    } alg_power_limit_t;

    alg_power_limit_status_t alg_power_limit_init(alg_power_limit_t *me,
                                                  const alg_power_limit_config_t *config);
    alg_power_limit_status_t alg_power_limit_reset(alg_power_limit_t *me);
    alg_power_limit_status_t alg_power_limit_update(alg_power_limit_t *me,
                                                    float referee_power_limit_w,
                                                    float buffer_energy_j,
                                                    const alg_power_limit_channel_input_t *inputs,
                                                    alg_power_limit_channel_output_t *outputs,
                                                    size_t channel_count, float delta_time_s);
    float alg_power_limit_estimate_motor_power(float torque_nm, float velocity_rad_per_s,
                                               float current_a, float winding_resistance_ohm,
                                               float idle_power_w);

#ifdef __cplusplus
}
#endif

#endif
