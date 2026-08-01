/**
 * @file alg_math_quaternion.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 四元数运算实现（ZYX 欧拉角、旋转、SLERP）
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 四元数格式 (w, x, y, z)，表示主动旋转。
 *       SLERP 包含共轭选择（最短路径）和线性插值退化处理。
 */

#include "alg_math.h"
#include <math.h>

/** @brief 四元数范数最小阈值 */
#define ALG_MATH_QUATERNION_MINIMUM_NORM (1.0e-12F)
/** @brief SLERP 线性插值切换阈值（接近 1 时避免除零） */
#define ALG_MATH_SLERP_LINEAR_THRESHOLD (0.9995F)

/**
 * @brief 检查四元数是否全为有限数
 */
static bool alg_math_quaternion_is_finite(const alg_math_quaternion_t *quaternion)
{
    return (quaternion != NULL) && isfinite(quaternion->w) && isfinite(quaternion->x) &&
           isfinite(quaternion->y) && isfinite(quaternion->z);
}

/**
 * @brief 单位四元数（恒等旋转）
 */
alg_math_status_t alg_math_quaternion_identity(alg_math_quaternion_t *result)
{
    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    *result = (alg_math_quaternion_t){.w = 1.0F, .x = 0.0F, .y = 0.0F, .z = 0.0F};
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 归一化四元数
 */
alg_math_status_t alg_math_quaternion_normalize(const alg_math_quaternion_t *quaternion,
                                                alg_math_quaternion_t *result)
{
    float norm;
    alg_math_quaternion_t temporary;
    if ((quaternion == NULL) || (result == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_quaternion_is_finite(quaternion)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    norm = sqrtf((quaternion->w * quaternion->w) + (quaternion->x * quaternion->x) +
                 (quaternion->y * quaternion->y) + (quaternion->z * quaternion->z));
    if (!isfinite(norm) || (norm <= ALG_MATH_QUATERNION_MINIMUM_NORM)) {
        return ALG_MATH_STATUS_SINGULAR;
}

    temporary.w = quaternion->w / norm;
    temporary.x = quaternion->x / norm;
    temporary.y = quaternion->y / norm;
    temporary.z = quaternion->z / norm;
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 四元数共轭
 */
alg_math_status_t alg_math_quaternion_conjugate(const alg_math_quaternion_t *quaternion,
                                                alg_math_quaternion_t *result)
{
    alg_math_quaternion_t temporary;
    if ((quaternion == NULL) || (result == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_quaternion_is_finite(quaternion)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}
    temporary.w = quaternion->w;
    temporary.x = -quaternion->x;
    temporary.y = -quaternion->y;
    temporary.z = -quaternion->z;
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 四元数乘法（左乘）
 */
alg_math_status_t alg_math_quaternion_multiply(const alg_math_quaternion_t *left,
                                               const alg_math_quaternion_t *right,
                                               alg_math_quaternion_t *result)
{
    alg_math_quaternion_t temporary;
    if ((left == NULL) || (right == NULL) || (result == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!alg_math_quaternion_is_finite(left) || !alg_math_quaternion_is_finite(right)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    // ---- 四元数乘法公式 ----
    temporary.w =
        (left->w * right->w) - (left->x * right->x) - (left->y * right->y) - (left->z * right->z);
    temporary.x =
        (left->w * right->x) + (left->x * right->w) + (left->y * right->z) - (left->z * right->y);
    temporary.y =
        (left->w * right->y) - (left->x * right->z) + (left->y * right->w) + (left->z * right->x);
    temporary.z =
        (left->w * right->z) + (left->x * right->y) - (left->y * right->x) + (left->z * right->w);

    if (!alg_math_quaternion_is_finite(&temporary)) {
        return ALG_MATH_STATUS_NUMERICAL_ERROR;
}
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 欧拉角（ZYX）转四元数
 */
alg_math_status_t alg_math_quaternion_from_euler(float roll_rad, float pitch_rad, float yaw_rad,
                                                 alg_math_quaternion_t *result)
{
    float half_roll, half_pitch, half_yaw;
    float cr, sr, cp, sp, cy, sy;
    alg_math_quaternion_t temporary;

    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(roll_rad) || !isfinite(pitch_rad) || !isfinite(yaw_rad)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    half_roll = 0.5F * roll_rad;
    half_pitch = 0.5F * pitch_rad;
    half_yaw = 0.5F * yaw_rad;
    cr = cosf(half_roll);
    sr = sinf(half_roll);
    cp = cosf(half_pitch);
    sp = sinf(half_pitch);
    cy = cosf(half_yaw);
    sy = sinf(half_yaw);

    // ---- ZYX 顺序（先 yaw，再 pitch，最后 roll） ----
    temporary.w = (cr * cp * cy) + (sr * sp * sy);
    temporary.x = (sr * cp * cy) - (cr * sp * sy);
    temporary.y = (cr * sp * cy) + (sr * cp * sy);
    temporary.z = (cr * cp * sy) - (sr * sp * cy);

    return alg_math_quaternion_normalize(&temporary, result);
}

/**
 * @brief 四元数转 ZYX 欧拉角
 */
alg_math_status_t alg_math_quaternion_to_euler(const alg_math_quaternion_t *quaternion,
                                               alg_math_vector3_t *euler_rad)
{
    alg_math_quaternion_t normalized;
    float pitch_sine;
    alg_math_status_t status;

    if (euler_rad == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    status = alg_math_quaternion_normalize(quaternion, &normalized);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}

    // ---- Roll (x) ----
    euler_rad->x =
        atan2f(2.0F * ((normalized.w * normalized.x) + (normalized.y * normalized.z)),
               1.0F - (2.0F * ((normalized.x * normalized.x) + (normalized.y * normalized.y))));

    // ---- Pitch (y) ----
    pitch_sine = 2.0F * ((normalized.w * normalized.y) - (normalized.z * normalized.x));
    if (fabsf(pitch_sine) >= 1.0F) {
        euler_rad->y = copysignf(ALG_MATH_HALF_PI_F, pitch_sine);
    } else {
        euler_rad->y = asinf(pitch_sine);
}

    // ---- Yaw (z) ----
    euler_rad->z =
        atan2f(2.0F * ((normalized.w * normalized.z) + (normalized.x * normalized.y)),
               1.0F - (2.0F * ((normalized.y * normalized.y) + (normalized.z * normalized.z))));
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 四元数旋转向量：v' = q * v * q^{-1}
 */
alg_math_status_t alg_math_quaternion_rotate_vector(const alg_math_quaternion_t *quaternion,
                                                    const alg_math_vector3_t *vector,
                                                    alg_math_vector3_t *result)
{
    alg_math_quaternion_t normalized;
    alg_math_vector3_t q_vec, first_cross, second_cross, temporary;
    alg_math_status_t status;

    if ((vector == NULL) || (result == NULL)) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    status = alg_math_quaternion_normalize(quaternion, &normalized);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}

    q_vec = (alg_math_vector3_t){normalized.x, normalized.y, normalized.z};
    status = alg_math_vector3_cross(&q_vec, vector, &first_cross);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}
    status = alg_math_vector3_cross(&q_vec, &first_cross, &second_cross);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}

    // ---- v' = v + 2*w*(q_vec × v) + 2*(q_vec × (q_vec × v)) ----
    temporary.x = vector->x + (2.0F * ((normalized.w * first_cross.x) + second_cross.x));
    temporary.y = vector->y + (2.0F * ((normalized.w * first_cross.y) + second_cross.y));
    temporary.z = vector->z + (2.0F * ((normalized.w * first_cross.z) + second_cross.z));
    *result = temporary;
    return ALG_MATH_STATUS_OK;
}

/**
 * @brief 四元数球面线性插值（SLERP）
 */
alg_math_status_t alg_math_quaternion_slerp(const alg_math_quaternion_t *start,
                                            const alg_math_quaternion_t *end, float ratio,
                                            alg_math_quaternion_t *result)
{
    alg_math_quaternion_t ns, ne, temp;
    float dot, angle, sin_angle, sw, ew;
    alg_math_status_t status;

    if (result == NULL) {
        return ALG_MATH_STATUS_INVALID_ARGUMENT;
}
    if (!isfinite(ratio) || (ratio < 0.0F) || (ratio > 1.0F)) {
        return ALG_MATH_STATUS_OUT_OF_RANGE;
}

    status = alg_math_quaternion_normalize(start, &ns);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}
    status = alg_math_quaternion_normalize(end, &ne);
    if (status != ALG_MATH_STATUS_OK) {
        return status;
}

    // ---- 计算点积，若为负则翻转 end 以走最短路径 ----
    dot = (ns.w * ne.w) + (ns.x * ne.x) + (ns.y * ne.y) + (ns.z * ne.z);
    if (dot < 0.0F)
    {
        dot = -dot;
        ne.w = -ne.w;
        ne.x = -ne.x;
        ne.y = -ne.y;
        ne.z = -ne.z;
    }
    dot = fminf(fmaxf(dot, -1.0F), 1.0F);

    // ---- 若接近 1，使用线性插值避免除零 ----
    if (dot > ALG_MATH_SLERP_LINEAR_THRESHOLD)
    {
        temp.w = ns.w + (ratio * (ne.w - ns.w));
        temp.x = ns.x + (ratio * (ne.x - ns.x));
        temp.y = ns.y + (ratio * (ne.y - ns.y));
        temp.z = ns.z + (ratio * (ne.z - ns.z));
        return alg_math_quaternion_normalize(&temp, result);
    }

    // ---- 球面插值 ----
    angle = acosf(dot);
    sin_angle = sinf(angle);
    if (fabsf(sin_angle) <= ALG_MATH_QUATERNION_MINIMUM_NORM) {
        return ALG_MATH_STATUS_SINGULAR;
}

    sw = sinf((1.0F - ratio) * angle) / sin_angle;
    ew = sinf(ratio * angle) / sin_angle;
    temp.w = (sw * ns.w) + (ew * ne.w);
    temp.x = (sw * ns.x) + (ew * ne.x);
    temp.y = (sw * ns.y) + (ew * ne.y);
    temp.z = (sw * ns.z) + (ew * ne.z);
    return alg_math_quaternion_normalize(&temp, result);
}