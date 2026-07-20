#ifndef ALG_FILTER_H
#define ALG_FILTER_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief Common status returned by the filter library. */
typedef enum
{
    ALG_FILTER_STATUS_OK = 0,
    ALG_FILTER_STATUS_INVALID_ARGUMENT,
    ALG_FILTER_STATUS_OUT_OF_RANGE,
    ALG_FILTER_STATUS_NOT_INITIALIZED,
    ALG_FILTER_STATUS_NUMERICAL_ERROR
} AlgFilterStatus_t;

/** @brief First-order RC low-pass filter instance. */
typedef struct
{
    float cutoff_frequency_hz;
    float output;
    bool is_initialized;
    bool has_previous_sample;
} AlgFilterLowPass_t;

/** @brief First-order RC high-pass filter instance. */
typedef struct
{
    float cutoff_frequency_hz;
    float previous_input;
    float output;
    bool is_initialized;
    bool has_previous_sample;
} AlgFilterHighPass_t;

/** @brief Exponential moving-average filter instance. */
typedef struct
{
    float smoothing_factor;
    float output;
    bool is_initialized;
    bool has_previous_sample;
} AlgFilterExponential_t;

/** @brief Moving-average filter using caller-owned storage. */
typedef struct
{
    float *sample_buffer;
    size_t capacity;
    size_t sample_count;
    size_t write_index;
    float sum;
    bool is_initialized;
} AlgFilterMovingAverage_t;

/** @brief Median filter using caller-owned sample and workspace buffers. */
typedef struct
{
    float *sample_buffer;
    float *sort_buffer;
    size_t capacity;
    size_t sample_count;
    size_t write_index;
    bool is_initialized;
} AlgFilterMedian_t;

/** @brief Generic finite impulse response filter instance. */
typedef struct
{
    const float *coefficients;
    float *state_buffer;
    size_t tap_count;
    size_t write_index;
    bool is_initialized;
} AlgFilterFir_t;

/** @brief Supported second-order Biquad response types. */
typedef enum
{
    ALG_FILTER_BIQUAD_LOW_PASS = 0,
    ALG_FILTER_BIQUAD_HIGH_PASS,
    ALG_FILTER_BIQUAD_BAND_PASS,
    ALG_FILTER_BIQUAD_NOTCH
} AlgFilterBiquadType_t;

/** @brief Direct-form-II-transposed Biquad filter instance. */
typedef struct
{
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float state_1;
    float state_2;
    bool is_initialized;
} AlgFilterBiquad_t;

/** @brief Complementary filter for a measured value and its measured rate. */
typedef struct
{
    float prediction_weight;
    float output;
    bool is_initialized;
} AlgFilterComplementary_t;

AlgFilterStatus_t AlgFilterLowPass_Init(AlgFilterLowPass_t *self,
                                        float cutoff_frequency_hz);
AlgFilterStatus_t AlgFilterLowPass_SetCutoff(AlgFilterLowPass_t *self,
                                             float cutoff_frequency_hz);
AlgFilterStatus_t AlgFilterLowPass_Reset(AlgFilterLowPass_t *self,
                                         float initial_output);
AlgFilterStatus_t AlgFilterLowPass_Update(AlgFilterLowPass_t *self,
                                          float input,
                                          float delta_time_s,
                                          float *output);

AlgFilterStatus_t AlgFilterHighPass_Init(AlgFilterHighPass_t *self,
                                         float cutoff_frequency_hz);
AlgFilterStatus_t AlgFilterHighPass_SetCutoff(AlgFilterHighPass_t *self,
                                              float cutoff_frequency_hz);
AlgFilterStatus_t AlgFilterHighPass_Reset(AlgFilterHighPass_t *self,
                                          float initial_input);
AlgFilterStatus_t AlgFilterHighPass_Update(AlgFilterHighPass_t *self,
                                           float input,
                                           float delta_time_s,
                                           float *output);

AlgFilterStatus_t AlgFilterExponential_Init(AlgFilterExponential_t *self,
                                            float smoothing_factor);
AlgFilterStatus_t AlgFilterExponential_SetFactor(AlgFilterExponential_t *self,
                                                 float smoothing_factor);
AlgFilterStatus_t AlgFilterExponential_Reset(AlgFilterExponential_t *self,
                                             float initial_output);
AlgFilterStatus_t AlgFilterExponential_Update(AlgFilterExponential_t *self,
                                              float input,
                                              float *output);

AlgFilterStatus_t AlgFilterMovingAverage_Init(AlgFilterMovingAverage_t *self,
                                              float *sample_buffer,
                                              size_t capacity);
AlgFilterStatus_t AlgFilterMovingAverage_Reset(AlgFilterMovingAverage_t *self);
AlgFilterStatus_t AlgFilterMovingAverage_Update(AlgFilterMovingAverage_t *self,
                                                float input,
                                                float *output);

AlgFilterStatus_t AlgFilterMedian_Init(AlgFilterMedian_t *self,
                                       float *sample_buffer,
                                       float *sort_buffer,
                                       size_t capacity);
AlgFilterStatus_t AlgFilterMedian_Reset(AlgFilterMedian_t *self);
AlgFilterStatus_t AlgFilterMedian_Update(AlgFilterMedian_t *self,
                                         float input,
                                         float *output);

AlgFilterStatus_t AlgFilterFir_Init(AlgFilterFir_t *self,
                                    const float *coefficients,
                                    float *state_buffer,
                                    size_t tap_count);
AlgFilterStatus_t AlgFilterFir_Reset(AlgFilterFir_t *self);
AlgFilterStatus_t AlgFilterFir_Update(AlgFilterFir_t *self,
                                      float input,
                                      float *output);

AlgFilterStatus_t AlgFilterBiquad_Init(AlgFilterBiquad_t *self,
                                       AlgFilterBiquadType_t type,
                                       float sample_frequency_hz,
                                       float center_frequency_hz,
                                       float quality_factor);
AlgFilterStatus_t AlgFilterBiquad_SetCoefficients(AlgFilterBiquad_t *self,
                                                  float b0,
                                                  float b1,
                                                  float b2,
                                                  float a1,
                                                  float a2);
AlgFilterStatus_t AlgFilterBiquad_Reset(AlgFilterBiquad_t *self);
AlgFilterStatus_t AlgFilterBiquad_Update(AlgFilterBiquad_t *self,
                                         float input,
                                         float *output);

AlgFilterStatus_t AlgFilterComplementary_Init(AlgFilterComplementary_t *self,
                                              float prediction_weight,
                                              float initial_output);
AlgFilterStatus_t AlgFilterComplementary_SetWeight(AlgFilterComplementary_t *self,
                                                   float prediction_weight);
AlgFilterStatus_t AlgFilterComplementary_Reset(AlgFilterComplementary_t *self,
                                               float initial_output);
AlgFilterStatus_t AlgFilterComplementary_Update(AlgFilterComplementary_t *self,
                                                float measured_value,
                                                float measured_rate_per_s,
                                                float delta_time_s,
                                                float *output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_FILTER_H */
