/**
 * @file module_swerve.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 单个舵轮执行模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 把 alg_swerve_module_target_t 转换成驱动电机速度目标和舵向电机位置目标。
 *       底盘几何解算留在 alg_swerve，本模块只处理一个物理舵轮。
 */

#include "module_swerve.h"

#include <math.h>   // isfinite
#include <stddef.h> // NULL

/**
 * @brief 读取舵向电机的当前物理角度（内部函数）
 * @param me 舵轮对象
 * @param[out] steering_angle_rad 输出舵向角度（弧度）
 * @return 执行状态
 * @note 从舵向电机反馈中读取位置，扣除零位偏移并应用方向符号
 */
static module_swerve_status_t module_swerve_read_steering_angle(const module_swerve_t *me,
                                                                float *steering_angle_rad)
{
    // 获取舵向电机反馈数据
    const module_motor_feedback_t *const steering_feedback =
        module_motor_get_feedback(me->steering_motor);

    // 反馈无效或离线则返回 NOT_READY
    if ((steering_feedback == NULL) || !steering_feedback->is_online)
    {
        return MODULE_SWERVE_STATUS_NOT_READY;
    }
    // 物理舵角 = (电机位置 × 方向符号) - 零位偏移
    *steering_angle_rad = (steering_feedback->position_rad * me->steering_direction_sign) -
                          me->steering_zero_offset_rad;
    return MODULE_SWERVE_STATUS_OK;
}

/**
 * @brief 初始化舵轮模块
 * @param me 舵轮对象
 * @param config 配置参数
 * @return 执行状态
 */
module_swerve_status_t module_swerve_init(module_swerve_t *me, const module_swerve_config_t *config)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->drive_motor == NULL) ||
        (config->steering_motor == NULL) || !config->drive_motor->is_initialized ||
        !config->steering_motor->is_initialized || !isfinite(config->wheel_radius_m) ||
        (config->wheel_radius_m <= 0.0F) || !isfinite(config->drive_reduction_ratio) ||
        (config->drive_reduction_ratio <= 0.0F) || !isfinite(config->steering_zero_offset_rad) ||
        ((config->drive_direction_sign != 1.0F) && (config->drive_direction_sign != -1.0F)) ||
        ((config->steering_direction_sign != 1.0F) && (config->steering_direction_sign != -1.0F)))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (module_swerve_t){
        .drive_motor = config->drive_motor,
        .steering_motor = config->steering_motor,
        .wheel_radius_m = config->wheel_radius_m,
        .drive_reduction_ratio = config->drive_reduction_ratio,
        .steering_zero_offset_rad = config->steering_zero_offset_rad,
        .drive_direction_sign = config->drive_direction_sign,
        .steering_direction_sign = config->steering_direction_sign,
        .is_initialized = true,
    };
    return MODULE_SWERVE_STATUS_OK;
}

/**
 * @brief 使能舵轮（按顺序使能驱动电机和舵向电机）
 * @param me 舵轮对象
 * @return 执行状态
 * @note 若任一电机使能失败，已使能的电机将被禁用（回滚）
 */
module_swerve_status_t module_swerve_enable(module_swerve_t *me)
{
    if (me == NULL)
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }

    // 1. 使能驱动电机
    if (module_motor_enable(me->drive_motor) != MODULE_MOTOR_STATUS_OK)
    {
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }

    // 2. 使能舵向电机
    if (module_motor_enable(me->steering_motor) != MODULE_MOTOR_STATUS_OK)
    {
        // 回滚：禁用已使能的驱动电机
        (void)module_motor_disable(me->drive_motor);
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }

    me->is_enabled = true;
    return MODULE_SWERVE_STATUS_OK;
}

/**
 * @brief 禁用舵轮（禁用驱动电机和舵向电机）
 * @param me 舵轮对象
 * @return 执行状态
 */
module_swerve_status_t module_swerve_disable(module_swerve_t *me)
{
    module_motor_status_t drive_status;
    module_motor_status_t steering_status;

    if (me == NULL)
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }

    // 禁用两个电机
    drive_status = module_motor_disable(me->drive_motor);
    steering_status = module_motor_disable(me->steering_motor);
    me->is_enabled = false;

    // 两者都成功才算 OK，否则返回 MOTOR_ERROR
    return ((drive_status == MODULE_MOTOR_STATUS_OK) && (steering_status == MODULE_MOTOR_STATUS_OK))
               ? MODULE_SWERVE_STATUS_OK
               : MODULE_SWERVE_STATUS_MOTOR_ERROR;
}

/**
 * @brief 应用运动学目标到舵轮
 * @param me 舵轮对象
 * @param target 运动学目标（线速度 + 舵角）
 * @param delta_time_s 时间步长（秒）
 * @return 执行状态
 * @note 内部调用 alg_swerve_optimize_target 优化目标（最短转向路径）
 *       将目标换算为电机目标并调用 module_motor_update
 */
module_swerve_status_t module_swerve_apply_target(module_swerve_t *me,
                                                  const alg_swerve_module_target_t *target,
                                                  float delta_time_s)
{
    alg_swerve_module_target_t optimized_target;
    float current_steering_angle_rad;
    float drive_velocity_rad_per_s;
    float steering_target_rad;
    module_swerve_status_t status;

    // ---- 参数校验 ----
    if ((me == NULL) || (target == NULL) || !isfinite(target->wheel_velocity_m_per_s) ||
        !isfinite(target->steering_angle_rad) || !isfinite(delta_time_s) || (delta_time_s <= 0.0F))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_enabled)
    {
        return MODULE_SWERVE_STATUS_NOT_READY;
    }

    // ---- 读取当前舵向角度 ----
    status = module_swerve_read_steering_angle(me, &current_steering_angle_rad);
    if (status != MODULE_SWERVE_STATUS_OK)
    {
        return status;
    }

    // ---- 优化目标（选择最短转向路径） ----
    optimized_target = *target;
    if (alg_swerve_optimize_target(current_steering_angle_rad, &optimized_target) !=
        ALG_SWERVE_STATUS_OK)
    {
        return MODULE_SWERVE_STATUS_ALGORITHM_ERROR;
    }

    // ---- 驱动电机：线速度 → 角速度 ----
    // 公式：电机角速度 = (线速度 / 轮半径) × 减速比 × 方向符号
    drive_velocity_rad_per_s = optimized_target.wheel_velocity_m_per_s / me->wheel_radius_m *
                               me->drive_reduction_ratio * me->drive_direction_sign;

    // ---- 舵向电机：舵角 → 电机位置 ----
    // 公式：电机位置 = (舵角 + 零位偏移) × 方向符号
    steering_target_rad = (optimized_target.steering_angle_rad + me->steering_zero_offset_rad) *
                          me->steering_direction_sign;

    // ---- 设置电机目标并更新 ----
    if ((module_motor_set_target(me->drive_motor, drive_velocity_rad_per_s) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_set_target(me->steering_motor, steering_target_rad) !=
         MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->drive_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK) ||
        (module_motor_update(me->steering_motor, delta_time_s) != MODULE_MOTOR_STATUS_OK))
    {
        return MODULE_SWERVE_STATUS_MOTOR_ERROR;
    }

    return MODULE_SWERVE_STATUS_OK;
}

/**
 * @brief 获取当前舵向角度（公共接口）
 * @param me 舵轮对象
 * @param[out] steering_angle_rad 输出舵向角度（弧度）
 * @return 执行状态
 */
module_swerve_status_t module_swerve_get_steering_angle(const module_swerve_t *me,
                                                        float *steering_angle_rad)
{
    if ((me == NULL) || (steering_angle_rad == NULL))
    {
        return MODULE_SWERVE_STATUS_INVALID_ARGUMENT;
    }
    if (!me->is_initialized)
    {
        return MODULE_SWERVE_STATUS_NOT_INITIALIZED;
    }
    // 复用内部函数读取舵向角度
    return module_swerve_read_steering_angle(me, steering_angle_rad);
}