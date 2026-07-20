#ifndef ALG_PID_H
#define ALG_PID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    ALG_PID_STATUS_OK = 0,
    ALG_PID_STATUS_INVALID_ARGUMENT,
    ALG_PID_STATUS_OUT_OF_RANGE,
    ALG_PID_STATUS_NOT_INITIALIZED,
    ALG_PID_STATUS_NUMERICAL_ERROR
} AlgPidStatus_t;

typedef enum
{
    ALG_PID_ANTI_WINDUP_NONE = 0,
    ALG_PID_ANTI_WINDUP_CLAMPING,
    ALG_PID_ANTI_WINDUP_BACK_CALCULATION
} AlgPidAntiWindup_t;

typedef enum
{
    ALG_PID_DERIVATIVE_ON_ERROR = 0,
    ALG_PID_DERIVATIVE_ON_MEASUREMENT
} AlgPidDerivativeMode_t;

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
    AlgPidAntiWindup_t anti_windup_mode;
    AlgPidDerivativeMode_t derivative_mode;
} AlgPidConfig_t;

typedef struct
{
    float setpoint;
    float measurement;
    float setpoint_rate_per_s;
    float setpoint_acceleration_per_s2;
    float additional_feedforward;
    float delta_time_s;
} AlgPidInput_t;

typedef struct
{
    float proportional;
    float integral;
    float derivative;
    float feedforward;
    float unsaturated_output;
    float output;
} AlgPidTerms_t;

typedef struct
{
    AlgPidConfig_t config;
    AlgPidTerms_t terms;
    float previous_error;
    float previous_setpoint;
    float previous_measurement;
    float filtered_derivative;
    bool has_previous_sample;
    bool is_initialized;
} AlgPid_t;

typedef AlgPid_t AlgPidPosition_t;
typedef AlgPid_t AlgPidVelocity_t;

AlgPidStatus_t AlgPidConfig_Init(AlgPidConfig_t *config);
AlgPidStatus_t AlgPid_Init(AlgPid_t *self, const AlgPidConfig_t *config);
AlgPidStatus_t AlgPidPosition_Init(AlgPidPosition_t *self,
                                  const AlgPidConfig_t *config);
AlgPidStatus_t AlgPidVelocity_Init(AlgPidVelocity_t *self,
                                  const AlgPidConfig_t *config);
AlgPidStatus_t AlgPid_SetConfig(AlgPid_t *self,
                               const AlgPidConfig_t *config);
AlgPidStatus_t AlgPid_Reset(AlgPid_t *self,
                            float measurement,
                            float initial_output);
AlgPidStatus_t AlgPid_TrackOutput(AlgPid_t *self,
                                  float setpoint,
                                  float measurement,
                                  float feedforward,
                                  float tracked_output);
AlgPidStatus_t AlgPid_Update(AlgPid_t *self,
                             float setpoint,
                             float measurement,
                             float delta_time_s,
                             float *output);
AlgPidStatus_t AlgPid_UpdateAdvanced(AlgPid_t *self,
                                     const AlgPidInput_t *input,
                                     float *output);
const AlgPidTerms_t *AlgPid_GetTerms(const AlgPid_t *self);

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
} AlgPidIncrementalConfig_t;

typedef struct
{
    AlgPidIncrementalConfig_t config;
    AlgPidTerms_t terms;
    float previous_error;
    float second_previous_error;
    float filtered_derivative_delta;
    bool has_previous_sample;
    bool is_initialized;
} AlgPidIncremental_t;

AlgPidStatus_t AlgPidIncrementalConfig_Init(
    AlgPidIncrementalConfig_t *config);
AlgPidStatus_t AlgPidIncremental_Init(
    AlgPidIncremental_t *self,
    const AlgPidIncrementalConfig_t *config);
AlgPidStatus_t AlgPidIncremental_SetConfig(
    AlgPidIncremental_t *self,
    const AlgPidIncrementalConfig_t *config);
AlgPidStatus_t AlgPidIncremental_Reset(AlgPidIncremental_t *self,
                                       float initial_output);
AlgPidStatus_t AlgPidIncremental_Update(AlgPidIncremental_t *self,
                                        float setpoint,
                                        float measurement,
                                        float feedforward_delta,
                                        float delta_time_s,
                                        float *output);
const AlgPidTerms_t *AlgPidIncremental_GetTerms(
    const AlgPidIncremental_t *self);

typedef struct
{
    float operating_point;
    float proportional_gain;
    float integral_gain;
    float derivative_gain;
} AlgPidGainPoint_t;

typedef struct
{
    AlgPid_t controller;
    const AlgPidGainPoint_t *gain_points;
    size_t gain_point_count;
    bool is_initialized;
} AlgPidGainSchedule_t;

AlgPidStatus_t AlgPidGainSchedule_Init(AlgPidGainSchedule_t *self,
                                       const AlgPidConfig_t *base_config,
                                       const AlgPidGainPoint_t *gain_points,
                                       size_t gain_point_count);
AlgPidStatus_t AlgPidGainSchedule_Update(AlgPidGainSchedule_t *self,
                                         float operating_point,
                                         const AlgPidInput_t *input,
                                         float *output);
AlgPidStatus_t AlgPidGainSchedule_Reset(AlgPidGainSchedule_t *self,
                                        float measurement,
                                        float initial_output);

typedef struct
{
    AlgPidConfig_t base_config;
    const float *proportional_adjustment_table;
    const float *integral_adjustment_table;
    const float *derivative_adjustment_table;
    size_t axis_point_count;
    float error_normalization;
    float error_rate_normalization;
} AlgPidFuzzyConfig_t;

typedef struct
{
    AlgPid_t controller;
    AlgPidFuzzyConfig_t config;
    float previous_error;
    bool has_previous_sample;
    bool is_initialized;
} AlgPidFuzzy_t;

AlgPidStatus_t AlgPidFuzzy_Init(AlgPidFuzzy_t *self,
                                const AlgPidFuzzyConfig_t *config);
AlgPidStatus_t AlgPidFuzzy_Reset(AlgPidFuzzy_t *self,
                                 float measurement,
                                 float initial_output);
AlgPidStatus_t AlgPidFuzzy_Update(AlgPidFuzzy_t *self,
                                  const AlgPidInput_t *input,
                                  float *output);

typedef struct
{
    AlgPidConfig_t position_config;
    AlgPidConfig_t velocity_config;
    uint32_t position_loop_divider;
    float velocity_setpoint_min;
    float velocity_setpoint_max;
} AlgPidCascadeConfig_t;

typedef struct
{
    float position_setpoint;
    float position_measurement;
    float velocity_measurement;
    float velocity_feedforward;
    float actuator_feedforward;
    float delta_time_s;
} AlgPidCascadeInput_t;

typedef struct
{
    AlgPidPosition_t position_controller;
    AlgPidVelocity_t velocity_controller;
    uint32_t position_loop_divider;
    uint32_t position_loop_counter;
    float position_elapsed_time_s;
    float velocity_setpoint_min;
    float velocity_setpoint_max;
    float velocity_setpoint;
    bool is_initialized;
} AlgPidCascade_t;

AlgPidStatus_t AlgPidCascade_Init(AlgPidCascade_t *self,
                                  const AlgPidCascadeConfig_t *config);
AlgPidStatus_t AlgPidCascade_Reset(AlgPidCascade_t *self,
                                   float position_measurement,
                                   float velocity_measurement,
                                   float initial_output);
AlgPidStatus_t AlgPidCascade_Update(AlgPidCascade_t *self,
                                    const AlgPidCascadeInput_t *input,
                                    float *output);
float AlgPidCascade_GetVelocitySetpoint(const AlgPidCascade_t *self);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PID_H */
