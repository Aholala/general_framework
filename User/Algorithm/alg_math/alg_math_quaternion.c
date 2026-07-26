#include "alg_math.h"

#include <math.h>

#define ALG_MATH_QUATERNION_MINIMUM_NORM (1.0e-12F)
#define ALG_MATH_SLERP_LINEAR_THRESHOLD (0.9995F)

static bool alg_math_quaternion_is_finite(const alg_math_quaternion_t *quaternion)
{
    return (quaternion != NULL) && isfinite(quaternion->w) && isfinite(quaternion->x) &&
           isfinite(quaternion->y) && isfinite(quaternion->z);
}

alg_math_status_t alg_math_quaternion_identity(alg_math_quaternion_t *result)
{
    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    *result = (alg_math_quaternion_t){.w = 1.0F, .x = 0.0F, .y = 0.0F, .z = 0.0F};
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_normalize(const alg_math_quaternion_t *quaternion,
                                                alg_math_quaternion_t *result)
{
    float norm;
    alg_math_quaternion_t temporary;

    if ((quaternion == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_quaternion_is_finite(quaternion))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    norm = sqrtf((quaternion->w * quaternion->w) + (quaternion->x * quaternion->x) +
                 (quaternion->y * quaternion->y) + (quaternion->z * quaternion->z));
    if (!isfinite(norm) || (norm <= ALG_MATH_QUATERNION_MINIMUM_NORM))
    {
        return ALG_MATH_STATUS_SINGULAR;
    }
    temporary.w = quaternion->w / norm;
    temporary.x = quaternion->x / norm;
    temporary.y = quaternion->y / norm;
    temporary.z = quaternion->z / norm;
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_conjugate(const alg_math_quaternion_t *quaternion,
                                                alg_math_quaternion_t *result)
{
    alg_math_quaternion_t temporary;

    if ((quaternion == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_quaternion_is_finite(quaternion))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.w = quaternion->w;
    temporary.x = -quaternion->x;
    temporary.y = -quaternion->y;
    temporary.z = -quaternion->z;
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_multiply(const alg_math_quaternion_t *left,
                                               const alg_math_quaternion_t *right,
                                               alg_math_quaternion_t *result)
{
    alg_math_quaternion_t temporary;

    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_quaternion_is_finite(left) || !alg_math_quaternion_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.w =
        (left->w * right->w) - (left->x * right->x) - (left->y * right->y) - (left->z * right->z);
    temporary.x =
        (left->w * right->x) + (left->x * right->w) + (left->y * right->z) - (left->z * right->y);
    temporary.y =
        (left->w * right->y) - (left->x * right->z) + (left->y * right->w) + (left->z * right->x);
    temporary.z =
        (left->w * right->z) + (left->x * right->y) - (left->y * right->x) + (left->z * right->w);
    if (!alg_math_quaternion_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_from_euler(float roll_rad, float pitch_rad, float yaw_rad,
                                                 alg_math_quaternion_t *result)
{
    float half_roll;
    float half_pitch;
    float half_yaw;
    float cosine_roll;
    float sine_roll;
    float cosine_pitch;
    float sine_pitch;
    float cosine_yaw;
    float sine_yaw;
    alg_math_quaternion_t temporary;

    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(roll_rad) || !isfinite(pitch_rad) || !isfinite(yaw_rad))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    half_roll = 0.5F * roll_rad;
    half_pitch = 0.5F * pitch_rad;
    half_yaw = 0.5F * yaw_rad;
    cosine_roll = cosf(half_roll);
    sine_roll = sinf(half_roll);
    cosine_pitch = cosf(half_pitch);
    sine_pitch = sinf(half_pitch);
    cosine_yaw = cosf(half_yaw);
    sine_yaw = sinf(half_yaw);
    temporary.w = (cosine_roll * cosine_pitch * cosine_yaw) + (sine_roll * sine_pitch * sine_yaw);
    temporary.x = (sine_roll * cosine_pitch * cosine_yaw) - (cosine_roll * sine_pitch * sine_yaw);
    temporary.y = (cosine_roll * sine_pitch * cosine_yaw) + (sine_roll * cosine_pitch * sine_yaw);
    temporary.z = (cosine_roll * cosine_pitch * sine_yaw) - (sine_roll * sine_pitch * cosine_yaw);
    return alg_math_quaternion_normalize(&temporary, result);
}

alg_math_status_t alg_math_quaternion_to_euler(const alg_math_quaternion_t *quaternion,
                                               alg_math_vector3_t *euler_rad)
{
    alg_math_quaternion_t normalized;
    float pitch_sine;
    alg_math_status_t status;

    if (euler_rad == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    status = alg_math_quaternion_normalize(quaternion, &normalized);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    euler_rad->x =
        atan2f(2.0F * ((normalized.w * normalized.x) + (normalized.y * normalized.z)),
               1.0F - (2.0F * ((normalized.x * normalized.x) + (normalized.y * normalized.y))));
    pitch_sine = 2.0F * ((normalized.w * normalized.y) - (normalized.z * normalized.x));
    if (fabsf(pitch_sine) >= 1.0F)
    {
        euler_rad->y = copysignf(ALG_MATH_HALF_PI_F, pitch_sine);
    }
    else
    {
        euler_rad->y = asinf(pitch_sine);
    }
    euler_rad->z =
        atan2f(2.0F * ((normalized.w * normalized.z) + (normalized.x * normalized.y)),
               1.0F - (2.0F * ((normalized.y * normalized.y) + (normalized.z * normalized.z))));
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_rotate_vector(const alg_math_quaternion_t *quaternion,
                                                    const alg_math_vector3_t *vector,
                                                    alg_math_vector3_t *result)
{
    alg_math_quaternion_t normalized;
    alg_math_vector3_t quaternion_vector;
    alg_math_vector3_t first_cross;
    alg_math_vector3_t second_cross;
    alg_math_vector3_t temporary;
    alg_math_status_t status;

    if ((vector == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    status = alg_math_quaternion_normalize(quaternion, &normalized);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    quaternion_vector = (alg_math_vector3_t){normalized.x, normalized.y, normalized.z};
    status = alg_math_vector3_cross(&quaternion_vector, vector, &first_cross);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    status = alg_math_vector3_cross(&quaternion_vector, &first_cross, &second_cross);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    temporary.x = vector->x + (2.0F * ((normalized.w * first_cross.x) + second_cross.x));
    temporary.y = vector->y + (2.0F * ((normalized.w * first_cross.y) + second_cross.y));
    temporary.z = vector->z + (2.0F * ((normalized.w * first_cross.z) + second_cross.z));
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_quaternion_slerp(const alg_math_quaternion_t *start,
                                            const alg_math_quaternion_t *end, float ratio,
                                            alg_math_quaternion_t *result)
{
    alg_math_quaternion_t normalized_start;
    alg_math_quaternion_t normalized_end;
    alg_math_quaternion_t temporary;
    float dot_product;
    float angle;
    float sine_angle;
    float start_weight;
    float end_weight;
    alg_math_status_t status;

    if (result == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!isfinite(ratio) || (ratio < 0.0F) || (ratio > 1.0F))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    status = alg_math_quaternion_normalize(start, &normalized_start);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    status = alg_math_quaternion_normalize(end, &normalized_end);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    dot_product = (normalized_start.w * normalized_end.w) +
                  (normalized_start.x * normalized_end.x) +
                  (normalized_start.y * normalized_end.y) + (normalized_start.z * normalized_end.z);
    if (dot_product < 0.0F)
    {
        dot_product = -dot_product;
        normalized_end.w = -normalized_end.w;
        normalized_end.x = -normalized_end.x;
        normalized_end.y = -normalized_end.y;
        normalized_end.z = -normalized_end.z;
    }
    dot_product = fminf(fmaxf(dot_product, -1.0F), 1.0F);
    if (dot_product > ALG_MATH_SLERP_LINEAR_THRESHOLD)
    {
        temporary.w = normalized_start.w + (ratio * (normalized_end.w - normalized_start.w));
        temporary.x = normalized_start.x + (ratio * (normalized_end.x - normalized_start.x));
        temporary.y = normalized_start.y + (ratio * (normalized_end.y - normalized_start.y));
        temporary.z = normalized_start.z + (ratio * (normalized_end.z - normalized_start.z));
        return alg_math_quaternion_normalize(&temporary, result);
    }
    angle = acosf(dot_product);
    sine_angle = sinf(angle);
    if (fabsf(sine_angle) <= ALG_MATH_QUATERNION_MINIMUM_NORM)
    {
        return ALG_MATH_STATUS_SINGULAR;
    }
    start_weight = sinf((1.0F - ratio) * angle) / sine_angle;
    end_weight = sinf(ratio * angle) / sine_angle;
    temporary.w = (start_weight * normalized_start.w) + (end_weight * normalized_end.w);
    temporary.x = (start_weight * normalized_start.x) + (end_weight * normalized_end.x);
    temporary.y = (start_weight * normalized_start.y) + (end_weight * normalized_end.y);
    temporary.z = (start_weight * normalized_start.z) + (end_weight * normalized_end.z);
    return alg_math_quaternion_normalize(&temporary, result);
}
