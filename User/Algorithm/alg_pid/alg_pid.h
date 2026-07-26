#ifndef ALG_PID_H
#define ALG_PID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        ALG_PID_STATUS_OK = 0,
        ALG_PID_STATUS_INVALID_ARGUMENT,
        ALG_PID_STATUS_OUT_OF_RANGE,
        ALG_PID_STATUS_NOT_INITIALIZED,
        ALG_PID_STATUS_NUMERICAL_ERROR
    } alg_pid_status_t;

    typedef enum
    {
        ALG_PID_ANTI_WINDUP_NONE = 0,
        ALG_PID_ANTI_WINDUP_CLAMPING,
        ALG_PID_ANTI_WINDUP_BACK_CALCULATION
    } alg_pid_anti_windup_t;

    typedef enum
    {
        ALG_PID_DERIVATIVE_ON_ERROR = 0,
        ALG_PID_DERIVATIVE_ON_MEASUREMENT
    } alg_pid_derivative_mode_t;

    typedef struct
    {
        float proportional_gain;
        float integral_gain;
        float derivative_gain;
        float setpoint_weight;
        float derivative_setpoint_weight;
        float velocity_feedforward_gain;
        float acceleration_feedforward_gain;
        float derivative_filter_cutoff_hz;
        float error_deadband;
        float integral_separation_threshold;
        float integral_min;
        float integral_max;
        float output_min;
        float output_max;
        float back_calculation_gain;
        alg_pid_anti_windup_t anti_windup_mode;
        alg_pid_derivative_mode_t derivative_mode;
    } alg_pid_config_t;

    typedef struct
    {
        float setpoint;
        float measurement;
        float setpoint_rate_per_s;
        float setpoint_acceleration_per_s2;
        float additional_feedforward;
        float delta_time_s;
    } alg_pid_input_t;

    typedef struct
    {
        float proportional;
        float integral;
        float derivative;
        float feedforward;
        float unsaturated_output;
        float output;
    } alg_pid_terms_t;

    typedef struct
    {
        alg_pid_config_t config;
        alg_pid_terms_t terms;
        float previous_error;
        float previous_setpoint;
        float previous_measurement;
        float filtered_derivative;
        bool has_previous_sample;
        bool is_initialized;
    } alg_pid_t;

    typedef alg_pid_t alg_pid_position_t;
    typedef alg_pid_t alg_pid_velocity_t;

    alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config);
    alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config);
    alg_pid_status_t alg_pid_position_init(alg_pid_position_t *me, const alg_pid_config_t *config);
    alg_pid_status_t alg_pid_velocity_init(alg_pid_velocity_t *me, const alg_pid_config_t *config);
    alg_pid_status_t alg_pid_set_config(alg_pid_t *me, const alg_pid_config_t *config);
    alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output);
    alg_pid_status_t alg_pid_track_output(alg_pid_t *me, float setpoint, float measurement,
                                          float feedforward, float tracked_output);
    alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                    float delta_time_s, float *output);
    alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                             float *output);
    const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me);

    typedef struct
    {
        float proportional_gain;
        float integral_gain;
        float derivative_gain;
        float derivative_filter_cutoff_hz;
        float error_deadband;
        float delta_output_min;
        float delta_output_max;
        float output_min;
        float output_max;
    } alg_pid_incremental_config_t;

    typedef struct
    {
        alg_pid_incremental_config_t config;
        alg_pid_terms_t terms;
        float previous_error;
        float second_previous_error;
        float filtered_derivative_delta;
        bool has_previous_sample;
        bool is_initialized;
    } alg_pid_incremental_t;

    alg_pid_status_t alg_pid_incremental_config_init(alg_pid_incremental_config_t *config);
    alg_pid_status_t alg_pid_incremental_init(alg_pid_incremental_t *me,
                                              const alg_pid_incremental_config_t *config);
    alg_pid_status_t alg_pid_incremental_set_config(alg_pid_incremental_t *me,
                                                    const alg_pid_incremental_config_t *config);
    alg_pid_status_t alg_pid_incremental_reset(alg_pid_incremental_t *me, float initial_output);
    alg_pid_status_t alg_pid_incremental_update(alg_pid_incremental_t *me, float setpoint,
                                                float measurement, float feedforward_delta,
                                                float delta_time_s, float *output);
    const alg_pid_terms_t *alg_pid_incremental_get_terms(const alg_pid_incremental_t *me);

    typedef struct
    {
        float operating_point;
        float proportional_gain;
        float integral_gain;
        float derivative_gain;
    } alg_pid_gain_point_t;

    typedef struct
    {
        alg_pid_t controller;
        const alg_pid_gain_point_t *gain_points;
        size_t gain_point_count;
        bool is_initialized;
    } alg_pid_gain_schedule_t;

    alg_pid_status_t alg_pid_gain_schedule_init(alg_pid_gain_schedule_t *me,
                                                const alg_pid_config_t *base_config,
                                                const alg_pid_gain_point_t *gain_points,
                                                size_t gain_point_count);
    alg_pid_status_t alg_pid_gain_schedule_update(alg_pid_gain_schedule_t *me,
                                                  float operating_point,
                                                  const alg_pid_input_t *input, float *output);
    alg_pid_status_t alg_pid_gain_schedule_reset(alg_pid_gain_schedule_t *me, float measurement,
                                                 float initial_output);

    typedef struct
    {
        alg_pid_config_t base_config;
        const float *proportional_adjustment_table;
        const float *integral_adjustment_table;
        const float *derivative_adjustment_table;
        size_t axis_point_count;
        float error_normalization;
        float error_rate_normalization;
    } alg_pid_fuzzy_config_t;

    typedef struct
    {
        alg_pid_t controller;
        alg_pid_fuzzy_config_t config;
        float previous_error;
        bool has_previous_sample;
        bool is_initialized;
    } alg_pid_fuzzy_t;

    alg_pid_status_t alg_pid_fuzzy_init(alg_pid_fuzzy_t *me, const alg_pid_fuzzy_config_t *config);
    alg_pid_status_t alg_pid_fuzzy_reset(alg_pid_fuzzy_t *me, float measurement,
                                         float initial_output);
    alg_pid_status_t alg_pid_fuzzy_update(alg_pid_fuzzy_t *me, const alg_pid_input_t *input,
                                          float *output);

    typedef struct
    {
        alg_pid_config_t position_config;
        alg_pid_config_t velocity_config;
        uint32_t position_loop_divider;
        float velocity_setpoint_min;
        float velocity_setpoint_max;
    } alg_pid_cascade_config_t;

    typedef struct
    {
        float position_setpoint;
        float position_measurement;
        float velocity_measurement;
        float velocity_feedforward;
        float actuator_feedforward;
        float delta_time_s;
    } alg_pid_cascade_input_t;

    typedef struct
    {
        alg_pid_position_t position_controller;
        alg_pid_velocity_t velocity_controller;
        uint32_t position_loop_divider;
        uint32_t position_loop_counter;
        float position_elapsed_time_s;
        float velocity_setpoint_min;
        float velocity_setpoint_max;
        float velocity_setpoint;
        bool is_initialized;
    } alg_pid_cascade_t;

    alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me,
                                          const alg_pid_cascade_config_t *config);
    alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                           float velocity_measurement, float initial_output);
    alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me,
                                            const alg_pid_cascade_input_t *input, float *output);
    float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PID_H */
