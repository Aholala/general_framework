#ifndef ALG_FILTER_H
#define ALG_FILTER_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /** @brief Common status returned by the filter library. */
    typedef enum
    {
        ALG_FILTER_STATUS_OK = 0,
        ALG_FILTER_STATUS_INVALID_ARGUMENT,
        ALG_FILTER_STATUS_OUT_OF_RANGE,
        ALG_FILTER_STATUS_NOT_INITIALIZED,
        ALG_FILTER_STATUS_NUMERICAL_ERROR
    } alg_filter_status_t;

    /** @brief First-order RC low-pass filter instance. */
    typedef struct
    {
        float cutoff_frequency_hz;
        float output;
        bool is_initialized;
        bool has_previous_sample;
    } alg_filter_low_pass_t;

    /** @brief First-order RC high-pass filter instance. */
    typedef struct
    {
        float cutoff_frequency_hz;
        float previous_input;
        float output;
        bool is_initialized;
        bool has_previous_sample;
    } alg_filter_high_pass_t;

    /** @brief Exponential moving-average filter instance. */
    typedef struct
    {
        float smoothing_factor;
        float output;
        bool is_initialized;
        bool has_previous_sample;
    } alg_filter_exponential_t;

    /** @brief Moving-average filter using caller-owned storage. */
    typedef struct
    {
        float *sample_buffer;
        size_t capacity;
        size_t sample_count;
        size_t write_index;
        float sum;
        bool is_initialized;
    } alg_filter_moving_average_t;

    /** @brief Median filter using caller-owned sample and workspace buffers. */
    typedef struct
    {
        float *sample_buffer;
        float *sort_buffer;
        size_t capacity;
        size_t sample_count;
        size_t write_index;
        bool is_initialized;
    } alg_filter_median_t;

    /** @brief Generic finite impulse response filter instance. */
    typedef struct
    {
        const float *coefficients;
        float *state_buffer;
        size_t tap_count;
        size_t write_index;
        bool is_initialized;
    } alg_filter_fir_t;

    /** @brief Supported second-order Biquad response types. */
    typedef enum
    {
        ALG_FILTER_BIQUAD_LOW_PASS = 0,
        ALG_FILTER_BIQUAD_HIGH_PASS,
        ALG_FILTER_BIQUAD_BAND_PASS,
        ALG_FILTER_BIQUAD_NOTCH
    } alg_filter_biquad_type_t;

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
    } alg_filter_biquad_t;

    /** @brief Complementary filter for a measured value and its measured rate. */
    typedef struct
    {
        float prediction_weight;
        float output;
        bool is_initialized;
    } alg_filter_complementary_t;

    alg_filter_status_t alg_filter_low_pass_init(alg_filter_low_pass_t *me,
                                                 float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_low_pass_set_cutoff(alg_filter_low_pass_t *me,
                                                       float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_low_pass_reset(alg_filter_low_pass_t *me, float initial_output);
    alg_filter_status_t alg_filter_low_pass_update(alg_filter_low_pass_t *me, float input,
                                                   float delta_time_s, float *output);

    alg_filter_status_t alg_filter_high_pass_init(alg_filter_high_pass_t *me,
                                                  float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_high_pass_set_cutoff(alg_filter_high_pass_t *me,
                                                        float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_high_pass_reset(alg_filter_high_pass_t *me, float initial_input);
    alg_filter_status_t alg_filter_high_pass_update(alg_filter_high_pass_t *me, float input,
                                                    float delta_time_s, float *output);

    alg_filter_status_t alg_filter_exponential_init(alg_filter_exponential_t *me,
                                                    float smoothing_factor);
    alg_filter_status_t alg_filter_exponential_set_factor(alg_filter_exponential_t *me,
                                                          float smoothing_factor);
    alg_filter_status_t alg_filter_exponential_reset(alg_filter_exponential_t *me,
                                                     float initial_output);
    alg_filter_status_t alg_filter_exponential_update(alg_filter_exponential_t *me, float input,
                                                      float *output);

    alg_filter_status_t alg_filter_moving_average_init(alg_filter_moving_average_t *me,
                                                       float *sample_buffer, size_t capacity);
    alg_filter_status_t alg_filter_moving_average_reset(alg_filter_moving_average_t *me);
    alg_filter_status_t alg_filter_moving_average_update(alg_filter_moving_average_t *me,
                                                         float input, float *output);

    alg_filter_status_t alg_filter_median_init(alg_filter_median_t *me, float *sample_buffer,
                                               float *sort_buffer, size_t capacity);
    alg_filter_status_t alg_filter_median_reset(alg_filter_median_t *me);
    alg_filter_status_t alg_filter_median_update(alg_filter_median_t *me, float input,
                                                 float *output);

    alg_filter_status_t alg_filter_fir_init(alg_filter_fir_t *me, const float *coefficients,
                                            float *state_buffer, size_t tap_count);
    alg_filter_status_t alg_filter_fir_reset(alg_filter_fir_t *me);
    alg_filter_status_t alg_filter_fir_update(alg_filter_fir_t *me, float input, float *output);

    alg_filter_status_t alg_filter_biquad_init(alg_filter_biquad_t *me,
                                               alg_filter_biquad_type_t type,
                                               float sample_frequency_hz, float center_frequency_hz,
                                               float quality_factor);
    alg_filter_status_t alg_filter_biquad_set_coefficients(alg_filter_biquad_t *me, float b0,
                                                           float b1, float b2, float a1, float a2);
    alg_filter_status_t alg_filter_biquad_reset(alg_filter_biquad_t *me);
    alg_filter_status_t alg_filter_biquad_update(alg_filter_biquad_t *me, float input,
                                                 float *output);

    alg_filter_status_t alg_filter_complementary_init(alg_filter_complementary_t *me,
                                                      float prediction_weight,
                                                      float initial_output);
    alg_filter_status_t alg_filter_complementary_set_weight(alg_filter_complementary_t *me,
                                                            float prediction_weight);
    alg_filter_status_t alg_filter_complementary_reset(alg_filter_complementary_t *me,
                                                       float initial_output);
    alg_filter_status_t alg_filter_complementary_update(alg_filter_complementary_t *me,
                                                        float measured_value,
                                                        float measured_rate_per_s,
                                                        float delta_time_s, float *output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_FILTER_H */
