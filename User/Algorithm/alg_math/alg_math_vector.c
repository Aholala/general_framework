#include "alg_math.h"

#include <math.h>

#define ALG_MATH_MINIMUM_NORM (1.0e-12F)

static bool alg_math_vector2_is_finite(const alg_math_vector2_t *vector)
{
    return (vector != NULL) && isfinite(vector->x) && isfinite(vector->y);
}

static bool alg_math_vector3_is_finite(const alg_math_vector3_t *vector)
{
    return (vector != NULL) && isfinite(vector->x) && isfinite(vector->y) && isfinite(vector->z);
}

alg_math_status_t alg_math_vector2_add(const alg_math_vector2_t *left,
                                       const alg_math_vector2_t *right, alg_math_vector2_t *result)
{
    alg_math_vector2_t temporary;

    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector2_is_finite(left) || !alg_math_vector2_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.x = left->x + right->x;
    temporary.y = left->y + right->y;
    if (!alg_math_vector2_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector2_subtract(const alg_math_vector2_t *left,
                                            const alg_math_vector2_t *right,
                                            alg_math_vector2_t *result)
{
    alg_math_vector2_t negative;

    if (right == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    negative.x = -right->x;
    negative.y = -right->y;
    return alg_math_vector2_add(left, &negative, result);
}

alg_math_status_t alg_math_vector2_scale(const alg_math_vector2_t *vector, float scale,
                                         alg_math_vector2_t *result)
{
    alg_math_vector2_t temporary;

    if ((result == NULL) || (vector == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector2_is_finite(vector) || !isfinite(scale))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.x = vector->x * scale;
    temporary.y = vector->y * scale;
    if (!alg_math_vector2_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector2_dot(const alg_math_vector2_t *left,
                                       const alg_math_vector2_t *right, float *result)
{
    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector2_is_finite(left) || !alg_math_vector2_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *result = (left->x * right->x) + (left->y * right->y);
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_vector2_norm(const alg_math_vector2_t *vector, float *norm)
{
    float squared_norm;
    alg_math_status_t status;

    status = alg_math_vector2_dot(vector, vector, &squared_norm);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    *norm = sqrtf(squared_norm);
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector2_normalize(const alg_math_vector2_t *vector,
                                             alg_math_vector2_t *result)
{
    float norm;
    alg_math_status_t status;

    status = alg_math_vector2_norm(vector, &norm);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    if (norm <= ALG_MATH_MINIMUM_NORM)
    {
        return ALG_MATH_STATUS_SINGULAR;
    }
    return alg_math_vector2_scale(vector, 1.0F / norm, result);
}

alg_math_status_t alg_math_vector3_add(const alg_math_vector3_t *left,
                                       const alg_math_vector3_t *right, alg_math_vector3_t *result)
{
    alg_math_vector3_t temporary;

    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector3_is_finite(left) || !alg_math_vector3_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.x = left->x + right->x;
    temporary.y = left->y + right->y;
    temporary.z = left->z + right->z;
    if (!alg_math_vector3_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector3_subtract(const alg_math_vector3_t *left,
                                            const alg_math_vector3_t *right,
                                            alg_math_vector3_t *result)
{
    alg_math_vector3_t negative;

    if (right == NULL)
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    negative.x = -right->x;
    negative.y = -right->y;
    negative.z = -right->z;
    return alg_math_vector3_add(left, &negative, result);
}

alg_math_status_t alg_math_vector3_scale(const alg_math_vector3_t *vector, float scale,
                                         alg_math_vector3_t *result)
{
    alg_math_vector3_t temporary;

    if ((vector == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector3_is_finite(vector) || !isfinite(scale))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.x = vector->x * scale;
    temporary.y = vector->y * scale;
    temporary.z = vector->z * scale;
    if (!alg_math_vector3_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector3_dot(const alg_math_vector3_t *left,
                                       const alg_math_vector3_t *right, float *result)
{
    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector3_is_finite(left) || !alg_math_vector3_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    *result = (left->x * right->x) + (left->y * right->y) + (left->z * right->z);
    return isfinite(*result) ? ALG_MATH_STATUS_OK : ALG_MATH_STATUS_NUMERICAL_ERROR;
}

alg_math_status_t alg_math_vector3_cross(const alg_math_vector3_t *left,
                                         const alg_math_vector3_t *right,
                                         alg_math_vector3_t *result)
{
    alg_math_vector3_t temporary;

    if ((left == NULL) || (right == NULL) || (result == NULL))
    {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
    }
    if (!alg_math_vector3_is_finite(left) || !alg_math_vector3_is_finite(right))
    {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
    }
    temporary.x = (left->y * right->z) - (left->z * right->y);
    temporary.y = (left->z * right->x) - (left->x * right->z);
    temporary.z = (left->x * right->y) - (left->y * right->x);
    if (!alg_math_vector3_is_finite(&temporary))
    {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
    }
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector3_norm(const alg_math_vector3_t *vector, float *norm)
{
    float squared_norm;
    alg_math_status_t status;

    status = alg_math_vector3_dot(vector, vector, &squared_norm);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    *norm = sqrtf(squared_norm);
    return ALG_MATH_STATUS_OK;
}

alg_math_status_t alg_math_vector3_normalize(const alg_math_vector3_t *vector,
                                             alg_math_vector3_t *result)
{
    float norm;
    alg_math_status_t status;

    status = alg_math_vector3_norm(vector, &norm);
    if (status != ALG_MATH_STATUS_OK)
    {
        return status;
    }
    if (norm <= ALG_MATH_MINIMUM_NORM)
    {
        return ALG_MATH_STATUS_SINGULAR;
    }
    return alg_math_vector3_scale(vector, 1.0F / norm, result);
}
