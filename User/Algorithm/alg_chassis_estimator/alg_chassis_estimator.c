#include "alg_chassis_estimator.h"

#include <math.h>
#include <string.h>

static bool alg_chassis_estimator_solve_3x3(float matrix[3][3], float vector[3], float result[3])
{
    size_t pivot_index;

    for (pivot_index = 0U; pivot_index < 3U; ++pivot_index)
    {
        size_t best_row = pivot_index;
        size_t row_index;
        for (row_index = pivot_index + 1U; row_index < 3U; ++row_index)
        {
            if (fabsf(matrix[row_index][pivot_index]) > fabsf(matrix[best_row][pivot_index]))
            {
                best_row = row_index;
            }
        }
        if (fabsf(matrix[best_row][pivot_index]) < 1.0e-7F)
        {
            return false;
        }
        if (best_row != pivot_index)
        {
            size_t column_index;
            for (column_index = 0U; column_index < 3U; ++column_index)
            {
                const float temporary = matrix[pivot_index][column_index];
                matrix[pivot_index][column_index] = matrix[best_row][column_index];
                matrix[best_row][column_index] = temporary;
            }
            {
                const float temporary = vector[pivot_index];
                vector[pivot_index] = vector[best_row];
                vector[best_row] = temporary;
            }
        }
        {
            const float divisor = matrix[pivot_index][pivot_index];
            size_t column_index;
            for (column_index = pivot_index; column_index < 3U; ++column_index)
            {
                matrix[pivot_index][column_index] /= divisor;
            }
            vector[pivot_index] /= divisor;
        }
        for (row_index = 0U; row_index < 3U; ++row_index)
        {
            if (row_index != pivot_index)
            {
                const float factor = matrix[row_index][pivot_index];
                size_t column_index;
                for (column_index = pivot_index; column_index < 3U; ++column_index)
                {
                    matrix[row_index][column_index] -= factor * matrix[pivot_index][column_index];
                }
                vector[row_index] -= factor * vector[pivot_index];
            }
        }
    }
    result[0] = vector[0];
    result[1] = vector[1];
    result[2] = vector[2];
    return true;
}

alg_chassis_estimator_status_t
alg_chassis_estimator_init(alg_chassis_estimator_t *me, const alg_chassis_estimator_wheel_t *wheels,
                           size_t wheel_count, float velocity_filter_time_constant_s)
{
    size_t wheel_index;
    if ((me == NULL) || (wheels == NULL) || (wheel_count < 2U) ||
        !isfinite(velocity_filter_time_constant_s) || (velocity_filter_time_constant_s < 0.0F))
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        const float norm = hypotf(wheels[wheel_index].direction_x, wheels[wheel_index].direction_y);
        if (!isfinite(norm) || (norm < 1.0e-6F))
        {
            return ALG_CHASSIS_ESTIMATOR_STATUS_INVALID_ARGUMENT;
        }
    }
    *me = (alg_chassis_estimator_t){
        .wheels = wheels,
        .wheel_count = wheel_count,
        .velocity_filter_time_constant_s = velocity_filter_time_constant_s,
        .is_initialized = true,
    };
    return ALG_CHASSIS_ESTIMATOR_STATUS_OK;
}

alg_chassis_estimator_status_t alg_chassis_estimator_update(
    alg_chassis_estimator_t *me, const alg_chassis_estimator_measurement_t *measurements,
    float external_yaw_rate_rad_per_s, bool use_external_yaw_rate, float delta_time_s)
{
    float normal_matrix[3][3] = {{0.0F}};
    float normal_vector[3] = {0.0F};
    float solution[3];
    float residual_sum = 0.0F;
    float filter_alpha;
    size_t wheel_index;

    if ((me == NULL) || (measurements == NULL) || !isfinite(delta_time_s) ||
        (delta_time_s <= 0.0F) || (use_external_yaw_rate && !isfinite(external_yaw_rate_rad_per_s)))
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_NOT_INITIALIZED;
    }
    me->state.valid_wheel_count = 0U;
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        const alg_chassis_estimator_wheel_t *const wheel = &me->wheels[wheel_index];
        const alg_chassis_estimator_measurement_t *const measurement = &measurements[wheel_index];
        float row[3];
        const float direction_norm = hypotf(wheel->direction_x, wheel->direction_y);
        float weight;
        size_t row_index;
        size_t column_index;

        if (!measurement->is_valid || !isfinite(measurement->linear_velocity_m_per_s) ||
            !isfinite(measurement->weight) || (measurement->weight <= 0.0F))
        {
            continue;
        }
        row[0] = wheel->direction_x / direction_norm;
        row[1] = wheel->direction_y / direction_norm;
        row[2] = -row[0] * wheel->position_y_m + row[1] * wheel->position_x_m;
        weight = measurement->weight;
        for (row_index = 0U; row_index < 3U; ++row_index)
        {
            normal_vector[row_index] +=
                weight * row[row_index] * measurement->linear_velocity_m_per_s;
            for (column_index = 0U; column_index < 3U; ++column_index)
            {
                normal_matrix[row_index][column_index] +=
                    weight * row[row_index] * row[column_index];
            }
        }
        ++me->state.valid_wheel_count;
    }
    if (use_external_yaw_rate)
    {
        const float yaw_weight = 1000.0F;
        normal_matrix[2][2] += yaw_weight;
        normal_vector[2] += yaw_weight * external_yaw_rate_rad_per_s;
    }
    if ((me->state.valid_wheel_count < 2U) ||
        !alg_chassis_estimator_solve_3x3(normal_matrix, normal_vector, solution))
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_SINGULAR;
    }
    filter_alpha = (me->velocity_filter_time_constant_s <= 0.0F)
                       ? 1.0F
                       : delta_time_s / (me->velocity_filter_time_constant_s + delta_time_s);
    me->state.velocity_x_m_per_s += filter_alpha * (solution[0] - me->state.velocity_x_m_per_s);
    me->state.velocity_y_m_per_s += filter_alpha * (solution[1] - me->state.velocity_y_m_per_s);
    me->state.angular_velocity_rad_per_s +=
        filter_alpha * (solution[2] - me->state.angular_velocity_rad_per_s);
    for (wheel_index = 0U; wheel_index < me->wheel_count; ++wheel_index)
    {
        if (measurements[wheel_index].is_valid)
        {
            const alg_chassis_estimator_wheel_t *const wheel = &me->wheels[wheel_index];
            const float direction_norm = hypotf(wheel->direction_x, wheel->direction_y);
            const float direction_x = wheel->direction_x / direction_norm;
            const float direction_y = wheel->direction_y / direction_norm;
            const float prediction =
                direction_x * solution[0] + direction_y * solution[1] +
                (-direction_x * wheel->position_y_m + direction_y * wheel->position_x_m) *
                    solution[2];
            const float error = measurements[wheel_index].linear_velocity_m_per_s - prediction;
            residual_sum += error * error;
        }
    }
    me->state.residual_rms_m_per_s = sqrtf(residual_sum / (float)me->state.valid_wheel_count);
    me->state.heading_rad += me->state.angular_velocity_rad_per_s * delta_time_s;
    me->state.position_x_m += (cosf(me->state.heading_rad) * me->state.velocity_x_m_per_s -
                               sinf(me->state.heading_rad) * me->state.velocity_y_m_per_s) *
                              delta_time_s;
    me->state.position_y_m += (sinf(me->state.heading_rad) * me->state.velocity_x_m_per_s +
                               cosf(me->state.heading_rad) * me->state.velocity_y_m_per_s) *
                              delta_time_s;
    return (me->state.valid_wheel_count < me->wheel_count) ? ALG_CHASSIS_ESTIMATOR_STATUS_DEGRADED
                                                           : ALG_CHASSIS_ESTIMATOR_STATUS_OK;
}

alg_chassis_estimator_status_t alg_chassis_estimator_reset_pose(alg_chassis_estimator_t *me,
                                                                float position_x_m,
                                                                float position_y_m,
                                                                float heading_rad)
{
    if ((me == NULL) || !isfinite(position_x_m) || !isfinite(position_y_m) ||
        !isfinite(heading_rad))
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return ALG_CHASSIS_ESTIMATOR_STATUS_NOT_INITIALIZED;
    }
    me->state.position_x_m = position_x_m;
    me->state.position_y_m = position_y_m;
    me->state.heading_rad = heading_rad;
    return ALG_CHASSIS_ESTIMATOR_STATUS_OK;
}

const alg_chassis_estimator_state_t *
alg_chassis_estimator_get_state(const alg_chassis_estimator_t *me)
{
    return ((me != NULL) && me->is_initialized) ? &me->state : NULL;
}
