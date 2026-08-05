/**
 * @file module_servo.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 标准 PWM 舵机控制模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 把弧度角度或归一化命令映射为微秒脉宽，并通过 bsp_pwm_t 输出。
 *       普通 PWM 舵机没有位置反馈，本模块只是开环命令转换。
 */

#include "module_servo.h"

#include <math.h>   // isfinite, isnan 检测
#include <stddef.h> // NULL

MODULE_STATIC_ASSERT_SUPER_FIRST(module_servo_t);

/**
 * @brief 将数值钳位到指定范围内
 * @param value 要钳位的值
 * @param minimum_value 最小值
 * @param maximum_value 最大值
 * @return 钳位后的值
 * @note 若 value < minimum 返回 minimum，若 value > maximum 返回 maximum
 */
static float module_servo_clamp(float value, float minimum_value, float maximum_value)
{
    if (value < minimum_value)
    {
        return minimum_value;
    }
    if (value > maximum_value)
    {
        return maximum_value;
    }
    return value;
}

/* ======================== module_device 虚函数实现 ======================== */

/**
 * @brief 设备启动回调（转发至 module_servo_start）
 */
static module_device_status_t module_servo_device_start(module_device_t *const device_base)
{
    module_servo_t *const me = MODULE_CONTAINER_OF(device_base, module_servo_t, super);
    return (module_servo_start(me) == MODULE_SERVO_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_servo_stop）
 */
static module_device_status_t module_servo_device_stop(module_device_t *const device_base)
{
    module_servo_t *const me = MODULE_CONTAINER_OF(device_base, module_servo_t, super);
    return (module_servo_stop(me) == MODULE_SERVO_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（舵机不需要周期性更新，空操作）
 */
static module_device_status_t module_servo_device_update(module_device_t *const device_base,
                                                         uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

/** 舵机模块的设备操作表 */
static const module_device_ops_t s_module_servo_ops = {
    .start = module_servo_device_start,
    .stop = module_servo_device_stop,
    .update = module_servo_device_update,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化舵机模块
 * @param me 舵机对象
 * @param config 配置参数
 * @return 执行状态
 * @note 校验配置参数合理性：频率 > 0，脉宽参数有限且正数，中位在最小和最大之间，
 *       角度范围有限且最小 < 最大，最大脉宽 × 频率 < 1,000,000（确保占空比 < 100%）
 */
module_servo_status_t module_servo_init(module_servo_t *me, const module_servo_config_t *config)
{
    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->pwm == NULL) ||
        !bsp_pwm_is_initialized(config->pwm) || (config->frequency_hz == 0U) ||
        !isfinite(config->minimum_pulse_width_us) || !isfinite(config->neutral_pulse_width_us) ||
        !isfinite(config->maximum_pulse_width_us) || (config->minimum_pulse_width_us <= 0.0F) ||
        (config->neutral_pulse_width_us < config->minimum_pulse_width_us) ||
        (config->neutral_pulse_width_us > config->maximum_pulse_width_us) ||
        !isfinite(config->minimum_angle_rad) || !isfinite(config->maximum_angle_rad) ||
        (config->minimum_angle_rad >= config->maximum_angle_rad) ||
        // 确保最大脉宽不超过 PWM 周期（否则占空比会超过 100%）
        ((config->maximum_pulse_width_us * (float)config->frequency_hz) >= 1000000.0F))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (module_servo_t){0};

    // 复制配置参数
    me->pwm = config->pwm;
    me->frequency_hz = config->frequency_hz;
    me->minimum_pulse_width_us = config->minimum_pulse_width_us;
    me->neutral_pulse_width_us = config->neutral_pulse_width_us;
    me->maximum_pulse_width_us = config->maximum_pulse_width_us;
    me->minimum_angle_rad = config->minimum_angle_rad;
    me->maximum_angle_rad = config->maximum_angle_rad;
    // 初始命令角度设为中点
    me->commanded_angle_rad = 0.5F * (config->minimum_angle_rad + config->maximum_angle_rad);

    // ---- 两阶段设备初始化 ----
    if (module_device_init_base(&me->super, &s_module_servo_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_SERVO_STATUS_OK;
}

/**
 * @brief 启动舵机（配置频率并输出中位脉宽）
 * @param me 舵机对象
 * @return 执行状态
 */
module_servo_status_t module_servo_start(module_servo_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }

    // 设置 PWM 频率并启动
    if ((bsp_pwm_set_frequency(me->pwm, me->frequency_hz) != BSP_STATUS_OK) ||
        (bsp_pwm_start(me->pwm) != BSP_STATUS_OK))
    {
        return MODULE_SERVO_STATUS_TRANSPORT_ERROR;
    }

    me->is_started = true;
    // 启动后输出中位脉宽（安全位置）
    return module_servo_set_pulse_width(me, me->neutral_pulse_width_us);
}

/**
 * @brief 停止舵机（关闭 PWM 输出）
 * @param me 舵机对象
 * @return 执行状态
 * @note 停止后输出行为取决于 BSP 平台的 PWM 安全状态（可能保持、高阻或固定电平）
 */
module_servo_status_t module_servo_stop(module_servo_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }

    me->is_started = false;
    return (bsp_pwm_stop(me->pwm) == BSP_STATUS_OK) ? MODULE_SERVO_STATUS_OK
                                                    : MODULE_SERVO_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 设置舵机角度（弧度）
 * @param me 舵机对象
 * @param angle_rad 目标角度（弧度）
 * @return 执行状态
 * @note 角度会被钳位到配置的最小/最大角度范围
 *       按照线性映射：角度 → 位置比例 → 脉宽
 */
module_servo_status_t module_servo_set_angle(module_servo_t *me, float angle_rad)
{
    float position_ratio;
    float pulse_width_us;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(angle_rad))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_SERVO_STATUS_NOT_STARTED;
    }

    // 钳位角度到有效范围
    angle_rad = module_servo_clamp(angle_rad, me->minimum_angle_rad, me->maximum_angle_rad);

    // 计算位置比例 [0, 1]
    position_ratio =
        (angle_rad - me->minimum_angle_rad) / (me->maximum_angle_rad - me->minimum_angle_rad);

    // 线性映射到脉宽
    pulse_width_us = me->minimum_pulse_width_us +
                     position_ratio * (me->maximum_pulse_width_us - me->minimum_pulse_width_us);

    // 设置脉宽
    if (module_servo_set_pulse_width(me, pulse_width_us) != MODULE_SERVO_STATUS_OK)
    {
        return MODULE_SERVO_STATUS_TRANSPORT_ERROR;
    }

    me->commanded_angle_rad = angle_rad;
    return MODULE_SERVO_STATUS_OK;
}

/**
 * @brief 设置归一化输出（通常用于速度/位置控制器的输出）
 * @param me 舵机对象
 * @param normalized_output 归一化值 [-1.0, 1.0]
 * @return 执行状态
 * @note positive → 从中位到最大脉宽，negative → 从中位到最小脉宽
 *       0 → 中位脉宽
 */
module_servo_status_t module_servo_set_normalized_output(module_servo_t *me,
                                                         float normalized_output)
{
    float pulse_width_us;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(normalized_output) || (normalized_output < -1.0F) ||
        (normalized_output > 1.0F))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }

    // 正负分别映射：正 → 中位到最大，负 → 中位到最小
    pulse_width_us =
        (normalized_output >= 0.0F)
            ? me->neutral_pulse_width_us +
                  normalized_output * (me->maximum_pulse_width_us - me->neutral_pulse_width_us)
            : me->neutral_pulse_width_us +
                  normalized_output * (me->neutral_pulse_width_us - me->minimum_pulse_width_us);

    return module_servo_set_pulse_width(me, pulse_width_us);
}

/**
 * @brief 直接设置脉冲宽度（微秒）
 * @param me 舵机对象
 * @param pulse_width_us 脉宽（微秒）
 * @return 执行状态
 * @note 脉宽会被校验是否在最小/最大范围内
 *       占空比 = 脉宽 × 频率 / 1,000,000
 */
module_servo_status_t module_servo_set_pulse_width(module_servo_t *me, float pulse_width_us)
{
    float duty_cycle;

    // ---- 参数校验 ----
    if ((me == NULL) || !isfinite(pulse_width_us))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_SERVO_STATUS_NOT_STARTED;
    }

    // 校验脉宽是否在有效范围内
    if ((pulse_width_us < me->minimum_pulse_width_us) ||
        (pulse_width_us > me->maximum_pulse_width_us))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }

    // 计算占空比：duty = pulse_width / period（period = 1/frequency）
    duty_cycle = pulse_width_us * (float)me->frequency_hz / 1000000.0F;

    // 设置 PWM 占空比
    return (bsp_pwm_set_duty_cycle(me->pwm, duty_cycle) == BSP_STATUS_OK)
               ? MODULE_SERVO_STATUS_OK
               : MODULE_SERVO_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 获取最后一次命令角度
 * @param me 舵机对象
 * @param angle_rad 输出角度（弧度）
 * @return 执行状态
 * @note 返回的是命令值，不是传感器实测角度（舵机无反馈）
 *       未启动时可读取配置的中位角度
 */
module_servo_status_t module_servo_get_commanded_angle(const module_servo_t *me, float *angle_rad)
{
    if ((me == NULL) || (angle_rad == NULL))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }
    *angle_rad = me->commanded_angle_rad;
    return MODULE_SERVO_STATUS_OK;
}
