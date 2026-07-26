#ifndef ALG_MATH_H
#define ALG_MATH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define ALG_MATH_PI_F (3.14159265358979323846F)
#define ALG_MATH_TWO_PI_F (6.28318530717958647692F)
#define ALG_MATH_HALF_PI_F (1.57079632679489661923F)
#define ALG_MATH_DEG_TO_RAD_F (ALG_MATH_PI_F / 180.0F)
#define ALG_MATH_RAD_TO_DEG_F (180.0F / ALG_MATH_PI_F)
#define ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(order) (2U * (size_t)(order) * (size_t)(order))
#define ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(order) ((size_t)(order) * ((size_t)(order) + 1U))

    typedef enum
    {
        ALG_MATH_STATUS_OK = 0,
        ALG_MATH_STATUS_INVALID_ARGUMENT,
        ALG_MATH_STATUS_OUT_OF_RANGE,
        ALG_MATH_STATUS_SIZE_MISMATCH,
        ALG_MATH_STATUS_SINGULAR,
        ALG_MATH_STATUS_NUMERICAL_ERROR
    } alg_math_status_t;

    typedef struct
    {
        float x;
        float y;
    } alg_math_vector2_t;

    typedef struct
    {
        float x;
        float y;
        float z;
    } alg_math_vector3_t;

    typedef struct
    {
        float w;
        float x;
        float y;
        float z;
    } alg_math_quaternion_t;

    typedef struct
    {
        size_t rows;
        size_t columns;
        float *data;
    } alg_math_matrix_t;

    typedef struct
    {
        uint32_t sample_count;
        float mean;
        float sum_of_squared_deviations;
        float minimum;
        float maximum;
    } alg_math_statistics_t;

    bool alg_math_is_finite_array(const float *values, size_t value_count);
    alg_math_status_t alg_math_clamp(float value, float lower_limit, float upper_limit,
                                     float *result);
    alg_math_status_t alg_math_lerp(float start, float end, float ratio, float *result);
    alg_math_status_t alg_math_map_range(float value, float input_minimum, float input_maximum,
                                         float output_minimum, float output_maximum,
                                         bool clamp_output, float *result);
    alg_math_status_t alg_math_apply_deadband(float value, float deadband, bool rescale_output,
                                              float *result);
    alg_math_status_t alg_math_wrap(float value, float lower_bound, float upper_bound,
                                    float *result);
    alg_math_status_t alg_math_wrap_angle_pi(float angle_rad, float *result_rad);
    alg_math_status_t alg_math_angle_difference(float target_rad, float current_rad,
                                                float *difference_rad);
    float alg_math_degrees_to_radians(float angle_deg);
    float alg_math_radians_to_degrees(float angle_rad);
    alg_math_status_t alg_math_safe_sqrt(float value, float *result);
    alg_math_status_t alg_math_safe_divide(float numerator, float denominator,
                                           float minimum_denominator, float *result);

    alg_math_status_t alg_math_statistics_init(alg_math_statistics_t *me);
    alg_math_status_t alg_math_statistics_update(alg_math_statistics_t *me, float sample);
    alg_math_status_t alg_math_statistics_get_population_variance(const alg_math_statistics_t *me,
                                                                  float *variance);
    alg_math_status_t alg_math_statistics_get_sample_variance(const alg_math_statistics_t *me,
                                                              float *variance);
    alg_math_status_t alg_math_statistics_get_standard_deviation(const alg_math_statistics_t *me,
                                                                 bool sample_standard_deviation,
                                                                 float *standard_deviation);
    alg_math_status_t alg_math_array_mean(const float *values, size_t value_count, float *mean);
    alg_math_status_t alg_math_array_rms(const float *values, size_t value_count, float *rms);
    alg_math_status_t alg_math_interpolate_linear1_d(const float *x_values, const float *y_values,
                                                     size_t point_count, float input,
                                                     bool clamp_to_table, float *output);
    alg_math_status_t alg_math_interpolate_bilinear(float x_ratio, float y_ratio, float value_00,
                                                    float value_10, float value_01, float value_11,
                                                    float *output);

    alg_math_status_t alg_math_vector2_add(const alg_math_vector2_t *left,
                                           const alg_math_vector2_t *right,
                                           alg_math_vector2_t *result);
    alg_math_status_t alg_math_vector2_subtract(const alg_math_vector2_t *left,
                                                const alg_math_vector2_t *right,
                                                alg_math_vector2_t *result);
    alg_math_status_t alg_math_vector2_scale(const alg_math_vector2_t *vector, float scale,
                                             alg_math_vector2_t *result);
    alg_math_status_t alg_math_vector2_dot(const alg_math_vector2_t *left,
                                           const alg_math_vector2_t *right, float *result);
    alg_math_status_t alg_math_vector2_norm(const alg_math_vector2_t *vector, float *norm);
    alg_math_status_t alg_math_vector2_normalize(const alg_math_vector2_t *vector,
                                                 alg_math_vector2_t *result);

    alg_math_status_t alg_math_vector3_add(const alg_math_vector3_t *left,
                                           const alg_math_vector3_t *right,
                                           alg_math_vector3_t *result);
    alg_math_status_t alg_math_vector3_subtract(const alg_math_vector3_t *left,
                                                const alg_math_vector3_t *right,
                                                alg_math_vector3_t *result);
    alg_math_status_t alg_math_vector3_scale(const alg_math_vector3_t *vector, float scale,
                                             alg_math_vector3_t *result);
    alg_math_status_t alg_math_vector3_dot(const alg_math_vector3_t *left,
                                           const alg_math_vector3_t *right, float *result);
    alg_math_status_t alg_math_vector3_cross(const alg_math_vector3_t *left,
                                             const alg_math_vector3_t *right,
                                             alg_math_vector3_t *result);
    alg_math_status_t alg_math_vector3_norm(const alg_math_vector3_t *vector, float *norm);
    alg_math_status_t alg_math_vector3_normalize(const alg_math_vector3_t *vector,
                                                 alg_math_vector3_t *result);

    alg_math_status_t alg_math_quaternion_identity(alg_math_quaternion_t *result);
    alg_math_status_t alg_math_quaternion_normalize(const alg_math_quaternion_t *quaternion,
                                                    alg_math_quaternion_t *result);
    alg_math_status_t alg_math_quaternion_conjugate(const alg_math_quaternion_t *quaternion,
                                                    alg_math_quaternion_t *result);
    alg_math_status_t alg_math_quaternion_multiply(const alg_math_quaternion_t *left,
                                                   const alg_math_quaternion_t *right,
                                                   alg_math_quaternion_t *result);
    alg_math_status_t alg_math_quaternion_from_euler(float roll_rad, float pitch_rad, float yaw_rad,
                                                     alg_math_quaternion_t *result);
    alg_math_status_t alg_math_quaternion_to_euler(const alg_math_quaternion_t *quaternion,
                                                   alg_math_vector3_t *euler_rad);
    alg_math_status_t alg_math_quaternion_rotate_vector(const alg_math_quaternion_t *quaternion,
                                                        const alg_math_vector3_t *vector,
                                                        alg_math_vector3_t *result);
    alg_math_status_t alg_math_quaternion_slerp(const alg_math_quaternion_t *start,
                                                const alg_math_quaternion_t *end, float ratio,
                                                alg_math_quaternion_t *result);

    alg_math_status_t alg_math_matrix_init(alg_math_matrix_t *matrix, float *data, size_t rows,
                                           size_t columns);
    alg_math_status_t alg_math_matrix_zero(alg_math_matrix_t *matrix);
    alg_math_status_t alg_math_matrix_identity(alg_math_matrix_t *matrix);
    alg_math_status_t alg_math_matrix_copy(const alg_math_matrix_t *source,
                                           alg_math_matrix_t *destination);
    alg_math_status_t alg_math_matrix_add(const alg_math_matrix_t *left,
                                          const alg_math_matrix_t *right,
                                          alg_math_matrix_t *result);
    alg_math_status_t alg_math_matrix_subtract(const alg_math_matrix_t *left,
                                               const alg_math_matrix_t *right,
                                               alg_math_matrix_t *result);
    alg_math_status_t alg_math_matrix_scale(const alg_math_matrix_t *input, float scale,
                                            alg_math_matrix_t *result);
    alg_math_status_t alg_math_matrix_multiply(const alg_math_matrix_t *left,
                                               const alg_math_matrix_t *right,
                                               alg_math_matrix_t *result);
    alg_math_status_t alg_math_matrix_transpose(const alg_math_matrix_t *input,
                                                alg_math_matrix_t *result);
    alg_math_status_t alg_math_matrix_multiply_vector(const alg_math_matrix_t *matrix,
                                                      const float *vector, size_t vector_length,
                                                      float *result, size_t result_length);
    alg_math_status_t alg_math_matrix_invert(const alg_math_matrix_t *input,
                                             alg_math_matrix_t *inverse, float *workspace,
                                             size_t workspace_size);
    alg_math_status_t alg_math_matrix_solve(const alg_math_matrix_t *coefficients,
                                            const float *right_hand_side, float *solution,
                                            float *workspace, size_t workspace_size);
    alg_math_status_t alg_math_matrix_cholesky(const alg_math_matrix_t *input,
                                               alg_math_matrix_t *lower_triangular);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MATH_H */
