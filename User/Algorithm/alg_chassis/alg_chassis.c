#include "alg_chassis.h"

#include <math.h>
#include <stddef.h>

#define ALG_CHASSIS_COMPONENT_COUNT (3U)
#define ALG_CHASSIS_QR_DIAGONAL_TOLERANCE (1.0E-6F)

static bool alg_chassis_velocity_is_valid(const alg_chassis_velocity_t *velocity)
{
    return (velocity != NULL) && isfinite(velocity->velocity_x_m_per_s) &&
           isfinite(velocity->velocity_y_m_per_s) && isfinite(velocity->angular_velocity_rad_per_s);
}

static float alg_chassis_get_velocity_component(const alg_chassis_velocity_t *velocity,
                                                size_t component_index)
{
    if (component_index == 0U)
    {
        return velocity->velocity_x_m_per_s;
    }
    if (component_index == 1U)
    {
        return velocity->velocity_y_m_per_s;
    }
    return velocity->angular_velocity_rad_per_s;
}

static void alg_chassis_set_velocity_component(alg_chassis_velocity_t *velocity,
                                               size_t component_index, float value)
{
    if (component_index == 0U)
    {
        velocity->velocity_x_m_per_s = value;
    }
    else if (component_index == 1U)
    {
        velocity->velocity_y_m_per_s = value;
    }
    else
    {
        velocity->angular_velocity_rad_per_s = value;
    }
}

static float alg_chassis_get_constraint_coefficient(const alg_chassis_constraint_t *constraint,
                                                    size_t component_index)
{
    if (component_index == 0U)
    {
        return constraint->velocity_x_coefficient;
    }
    if (component_index == 1U)
    {
        return constraint->velocity_y_coefficient;
    }
    return constraint->angular_velocity_coefficient_m;
}

static bool alg_chassis_constraint_is_valid(const alg_chassis_constraint_t *constraint)
{
    return isfinite(constraint->velocity_x_coefficient) &&
           isfinite(constraint->velocity_y_coefficient) &&
           isfinite(constraint->angular_velocity_coefficient_m) &&
           isfinite(constraint->measured_velocity_m_per_s) && isfinite(constraint->weight) &&
           (constraint->weight >= 0.0F);
}

static bool alg_chassis_qr_add_row(float upper_triangular[3][3], float transformed_vector[3],
                                   float row[3], float measured_value, size_t order)
{
    size_t diagonal_index;

    for (diagonal_index = 0U; diagonal_index < order; ++diagonal_index)
    {
        const float existing_value = upper_triangular[diagonal_index][diagonal_index];
        const float incoming_value = row[diagonal_index];
        const float hypotenuse = hypotf(existing_value, incoming_value);
        float cosine;
        float sine;
        size_t column_index;

        if (!isfinite(hypotenuse))
        {
            return false;
        }
        if (hypotenuse == 0.0F)
        {
            continue;
        }
        cosine = existing_value / hypotenuse;
        sine = incoming_value / hypotenuse;
        for (column_index = diagonal_index; column_index < order; ++column_index)
        {
            const float existing_column_value = upper_triangular[diagonal_index][column_index];
            const float incoming_column_value = row[column_index];
            upper_triangular[diagonal_index][column_index] =
                cosine * existing_column_value + sine * incoming_column_value;
            row[column_index] = -sine * existing_column_value + cosine * incoming_column_value;
        }
        {
            const float existing_vector_value = transformed_vector[diagonal_index];
            transformed_vector[diagonal_index] =
                cosine * existing_vector_value + sine * measured_value;
            measured_value = -sine * existing_vector_value + cosine * measured_value;
        }
    }
    return true;
}

static bool alg_chassis_qr_back_substitute(float upper_triangular[3][3],
                                           const float transformed_vector[3], size_t order,
                                           float solution[3])
{
    size_t diagonal_index;

    for (diagonal_index = 0U; diagonal_index < order; ++diagonal_index)
    {
        if (!isfinite(upper_triangular[diagonal_index][diagonal_index]) ||
            (fabsf(upper_triangular[diagonal_index][diagonal_index]) <=
             ALG_CHASSIS_QR_DIAGONAL_TOLERANCE))
        {
            return false;
        }
    }
    for (diagonal_index = order; diagonal_index > 0U; --diagonal_index)
    {
        const size_t row_index = diagonal_index - 1U;
        float value = transformed_vector[row_index];
        size_t column_index;
        for (column_index = row_index + 1U; column_index < order; ++column_index)
        {
            value -= upper_triangular[row_index][column_index] * solution[column_index];
        }
        solution[row_index] = value / upper_triangular[row_index][row_index];
        if (!isfinite(solution[row_index]))
        {
            return false;
        }
    }
    return true;
}

alg_chassis_status_t alg_chassis_solve_velocity(const alg_chassis_constraint_t *constraints,
                                                size_t constraint_count,
                                                uint8_t known_component_mask,
                                                const alg_chassis_velocity_t *known_velocity,
                                                size_t nominal_constraint_count,
                                                alg_chassis_solution_t *solution)
{
    float upper_triangular[3][3] = {{0.0F}};
    float transformed_vector[3] = {0.0F};
    float unknown_solution[3] = {0.0F};
    float column_scales[3] = {0.0F};
    size_t unknown_component_indices[3] = {0U};
    size_t unknown_component_count = 0U;
    size_t used_constraint_count = 0U;
    size_t constraint_index;
    size_t component_index;
    float squared_residual_sum = 0.0F;

    if ((constraints == NULL) || (constraint_count == 0U) || (solution == NULL) ||
        ((known_component_mask & (uint8_t)(~ALG_CHASSIS_COMPONENT_ALL)) != 0U) ||
        ((known_component_mask != 0U) && !alg_chassis_velocity_is_valid(known_velocity)) ||
        (nominal_constraint_count == 0U))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    *solution = (alg_chassis_solution_t){0};
    if (known_velocity != NULL)
    {
        solution->velocity = *known_velocity;
    }
    for (component_index = 0U; component_index < ALG_CHASSIS_COMPONENT_COUNT; ++component_index)
    {
        if ((known_component_mask & (1U << component_index)) == 0U)
        {
            unknown_component_indices[unknown_component_count++] = component_index;
        }
    }
    solution->unknown_component_count = unknown_component_count;
    if (unknown_component_count == 0U)
    {
        return ALG_CHASSIS_STATUS_OK;
    }

    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *const constraint = &constraints[constraint_index];
        size_t unknown_index;

        if (!alg_chassis_constraint_is_valid(constraint))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        if (!constraint->is_available || (constraint->weight == 0.0F))
        {
            continue;
        }
        for (unknown_index = 0U; unknown_index < unknown_component_count; ++unknown_index)
        {
            const float coefficient = alg_chassis_get_constraint_coefficient(
                constraint, unknown_component_indices[unknown_index]);
            column_scales[unknown_index] += constraint->weight * coefficient * coefficient;
        }
        ++used_constraint_count;
    }
    solution->used_constraint_count = used_constraint_count;
    if (used_constraint_count < unknown_component_count)
    {
        return ALG_CHASSIS_STATUS_UNDERDETERMINED;
    }
    for (component_index = 0U; component_index < unknown_component_count; ++component_index)
    {
        column_scales[component_index] = sqrtf(column_scales[component_index]);
        if (!isfinite(column_scales[component_index]) || (column_scales[component_index] <= 0.0F))
        {
            return ALG_CHASSIS_STATUS_SINGULAR;
        }
    }

    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *const constraint = &constraints[constraint_index];
        float adjusted_measurement;
        float weighted_row[3] = {0.0F};
        float square_root_weight;
        size_t unknown_index;

        if (!constraint->is_available || (constraint->weight == 0.0F))
        {
            continue;
        }
        adjusted_measurement = constraint->measured_velocity_m_per_s;
        for (component_index = 0U; component_index < ALG_CHASSIS_COMPONENT_COUNT; ++component_index)
        {
            if ((known_component_mask & (1U << component_index)) != 0U)
            {
                adjusted_measurement -=
                    alg_chassis_get_constraint_coefficient(constraint, component_index) *
                    alg_chassis_get_velocity_component(known_velocity, component_index);
            }
        }
        square_root_weight = sqrtf(constraint->weight);
        adjusted_measurement *= square_root_weight;
        for (unknown_index = 0U; unknown_index < unknown_component_count; ++unknown_index)
        {
            weighted_row[unknown_index] =
                square_root_weight *
                alg_chassis_get_constraint_coefficient(constraint,
                                                       unknown_component_indices[unknown_index]) /
                column_scales[unknown_index];
        }
        if (!alg_chassis_qr_add_row(upper_triangular, transformed_vector, weighted_row,
                                    adjusted_measurement, unknown_component_count))
        {
            return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
        }
    }
    if (!alg_chassis_qr_back_substitute(upper_triangular, transformed_vector,
                                        unknown_component_count, unknown_solution))
    {
        return ALG_CHASSIS_STATUS_SINGULAR;
    }
    for (component_index = 0U; component_index < unknown_component_count; ++component_index)
    {
        alg_chassis_set_velocity_component(
            &solution->velocity, unknown_component_indices[component_index],
            unknown_solution[component_index] / column_scales[component_index]);
    }
    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *const constraint = &constraints[constraint_index];
        float predicted_velocity;
        float residual;
        if (!constraint->is_available || (constraint->weight == 0.0F))
        {
            continue;
        }
        predicted_velocity =
            constraint->velocity_x_coefficient * solution->velocity.velocity_x_m_per_s +
            constraint->velocity_y_coefficient * solution->velocity.velocity_y_m_per_s +
            constraint->angular_velocity_coefficient_m *
                solution->velocity.angular_velocity_rad_per_s;
        residual = predicted_velocity - constraint->measured_velocity_m_per_s;
        squared_residual_sum += residual * residual;
    }
    solution->residual_root_mean_square_m_per_s =
        sqrtf(squared_residual_sum / (float)used_constraint_count);
    if (!alg_chassis_velocity_is_valid(&solution->velocity) ||
        !isfinite(solution->residual_root_mean_square_m_per_s))
    {
        return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
    }
    solution->is_degraded = used_constraint_count < nominal_constraint_count;
    return solution->is_degraded ? ALG_CHASSIS_STATUS_DEGRADED : ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_chassis_calculate_constraint_residuals(
    const alg_chassis_constraint_t *constraints, size_t constraint_count,
    const alg_chassis_velocity_t *velocity, float *residuals_m_per_s, size_t residual_capacity)
{
    size_t constraint_index;

    if ((constraints == NULL) || (constraint_count == 0U) ||
        !alg_chassis_velocity_is_valid(velocity) || (residuals_m_per_s == NULL) ||
        (residual_capacity < constraint_count))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (constraint_index = 0U; constraint_index < constraint_count; ++constraint_index)
    {
        const alg_chassis_constraint_t *const constraint = &constraints[constraint_index];
        float predicted_velocity;

        if (!alg_chassis_constraint_is_valid(constraint))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        if (!constraint->is_available || (constraint->weight == 0.0F))
        {
            residuals_m_per_s[constraint_index] = 0.0F;
            continue;
        }
        predicted_velocity =
            constraint->velocity_x_coefficient * velocity->velocity_x_m_per_s +
            constraint->velocity_y_coefficient * velocity->velocity_y_m_per_s +
            constraint->angular_velocity_coefficient_m * velocity->angular_velocity_rad_per_s;
        residuals_m_per_s[constraint_index] =
            predicted_velocity - constraint->measured_velocity_m_per_s;
        if (!isfinite(residuals_m_per_s[constraint_index]))
        {
            return ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
        }
    }
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t
alg_chassis_transform_reference_to_body(const alg_chassis_velocity_t *reference_velocity,
                                        float reference_heading_rad,
                                        alg_chassis_velocity_t *body_velocity)
{
    float heading_cosine;
    float heading_sine;

    if (!alg_chassis_velocity_is_valid(reference_velocity) || !isfinite(reference_heading_rad) ||
        (body_velocity == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    heading_cosine = cosf(reference_heading_rad);
    heading_sine = sinf(reference_heading_rad);
    body_velocity->velocity_x_m_per_s = heading_cosine * reference_velocity->velocity_x_m_per_s +
                                        heading_sine * reference_velocity->velocity_y_m_per_s;
    body_velocity->velocity_y_m_per_s = -heading_sine * reference_velocity->velocity_x_m_per_s +
                                        heading_cosine * reference_velocity->velocity_y_m_per_s;
    body_velocity->angular_velocity_rad_per_s = reference_velocity->angular_velocity_rad_per_s;
    return ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t alg_chassis_convert_center_velocity_to_origin(
    const alg_chassis_velocity_t *center_velocity, float center_of_rotation_x_m,
    float center_of_rotation_y_m, alg_chassis_velocity_t *origin_velocity)
{
    if (!alg_chassis_velocity_is_valid(center_velocity) || !isfinite(center_of_rotation_x_m) ||
        !isfinite(center_of_rotation_y_m) || (origin_velocity == NULL))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    origin_velocity->velocity_x_m_per_s =
        center_velocity->velocity_x_m_per_s +
        center_velocity->angular_velocity_rad_per_s * center_of_rotation_y_m;
    origin_velocity->velocity_y_m_per_s =
        center_velocity->velocity_y_m_per_s -
        center_velocity->angular_velocity_rad_per_s * center_of_rotation_x_m;
    origin_velocity->angular_velocity_rad_per_s = center_velocity->angular_velocity_rad_per_s;
    return alg_chassis_velocity_is_valid(origin_velocity) ? ALG_CHASSIS_STATUS_OK
                                                          : ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
}

alg_chassis_status_t alg_chassis_scale_wheel_velocities(float *wheel_velocities,
                                                        const bool *wheel_is_available,
                                                        size_t wheel_count,
                                                        float maximum_absolute_velocity,
                                                        float *applied_scale)
{
    float maximum_calculated_velocity = 0.0F;
    float scale = 1.0F;
    size_t wheel_index;
    size_t available_wheel_count = 0U;

    if ((wheel_velocities == NULL) || (wheel_count == 0U) || !isfinite(maximum_absolute_velocity) ||
        (maximum_absolute_velocity <= 0.0F))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        const bool is_available = (wheel_is_available == NULL) || wheel_is_available[wheel_index];
        if (!isfinite(wheel_velocities[wheel_index]))
        {
            return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
        }
        if (!is_available)
        {
            wheel_velocities[wheel_index] = 0.0F;
            continue;
        }
        ++available_wheel_count;
        if (fabsf(wheel_velocities[wheel_index]) > maximum_calculated_velocity)
        {
            maximum_calculated_velocity = fabsf(wheel_velocities[wheel_index]);
        }
    }
    if (available_wheel_count == 0U)
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    if (maximum_calculated_velocity > maximum_absolute_velocity)
    {
        scale = maximum_absolute_velocity / maximum_calculated_velocity;
    }
    for (wheel_index = 0U; wheel_index < wheel_count; ++wheel_index)
    {
        wheel_velocities[wheel_index] *= scale;
    }
    if (applied_scale != NULL)
    {
        *applied_scale = scale;
    }
    return (available_wheel_count < wheel_count) ? ALG_CHASSIS_STATUS_DEGRADED
                                                 : ALG_CHASSIS_STATUS_OK;
}

alg_chassis_status_t
alg_chassis_integrate_odometry(alg_chassis_pose_t *me, const alg_chassis_velocity_t *body_velocity,
                               float delta_time_s,
                               alg_chassis_integration_method_t integration_method)
{
    float body_displacement_x_m;
    float body_displacement_y_m;
    float integration_heading_rad;
    float heading_change_rad;
    float heading_cosine;
    float heading_sine;

    if ((me == NULL) || !alg_chassis_velocity_is_valid(body_velocity) ||
        !isfinite(me->position_x_m) || !isfinite(me->position_y_m) || !isfinite(me->heading_rad) ||
        !isfinite(delta_time_s) || (delta_time_s <= 0.0F) ||
        (integration_method > ALG_CHASSIS_INTEGRATION_EXACT))
    {
        return ALG_CHASSIS_STATUS_INVALID_ARGUMENT;
    }
    heading_change_rad = body_velocity->angular_velocity_rad_per_s * delta_time_s;
    integration_heading_rad = me->heading_rad;
    if (integration_method == ALG_CHASSIS_INTEGRATION_EXACT)
    {
        if (fabsf(body_velocity->angular_velocity_rad_per_s) > 1.0E-6F)
        {
            const float inverse_angular_velocity_s =
                1.0F / body_velocity->angular_velocity_rad_per_s;
            const float heading_change_sine = sinf(heading_change_rad);
            const float heading_change_cosine = cosf(heading_change_rad);
            body_displacement_x_m =
                inverse_angular_velocity_s *
                (heading_change_sine * body_velocity->velocity_x_m_per_s -
                 (1.0F - heading_change_cosine) * body_velocity->velocity_y_m_per_s);
            body_displacement_y_m =
                inverse_angular_velocity_s *
                ((1.0F - heading_change_cosine) * body_velocity->velocity_x_m_per_s +
                 heading_change_sine * body_velocity->velocity_y_m_per_s);
        }
        else
        {
            body_displacement_x_m = body_velocity->velocity_x_m_per_s * delta_time_s;
            body_displacement_y_m = body_velocity->velocity_y_m_per_s * delta_time_s;
        }
    }
    else
    {
        body_displacement_x_m = body_velocity->velocity_x_m_per_s * delta_time_s;
        body_displacement_y_m = body_velocity->velocity_y_m_per_s * delta_time_s;
        if (integration_method == ALG_CHASSIS_INTEGRATION_MIDPOINT)
        {
            integration_heading_rad += heading_change_rad * 0.5F;
        }
    }
    heading_cosine = cosf(integration_heading_rad);
    heading_sine = sinf(integration_heading_rad);
    me->position_x_m +=
        heading_cosine * body_displacement_x_m - heading_sine * body_displacement_y_m;
    me->position_y_m +=
        heading_sine * body_displacement_x_m + heading_cosine * body_displacement_y_m;
    me->heading_rad += heading_change_rad;
    return (isfinite(me->position_x_m) && isfinite(me->position_y_m) && isfinite(me->heading_rad))
               ? ALG_CHASSIS_STATUS_OK
               : ALG_CHASSIS_STATUS_NUMERICAL_ERROR;
}
