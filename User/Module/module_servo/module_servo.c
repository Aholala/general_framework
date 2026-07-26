#include "module_servo.h"

#include <math.h>
#include <stddef.h>

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

static module_device_status_t module_servo_device_start(module_device_t *const device_base)
{
    module_servo_t *const me = MODULE_CONTAINER_OF(device_base, module_servo_t, super);
    return (module_servo_start(me) == MODULE_SERVO_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_servo_device_stop(module_device_t *const device_base)
{
    module_servo_t *const me = MODULE_CONTAINER_OF(device_base, module_servo_t, super);
    return (module_servo_stop(me) == MODULE_SERVO_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_servo_device_update(module_device_t *const device_base,
                                                         uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

static const module_device_ops_t s_module_servo_ops = {
    .start = module_servo_device_start,
    .stop = module_servo_device_stop,
    .update = module_servo_device_update,
};

module_servo_status_t module_servo_init(module_servo_t *me, const module_servo_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->pwm == NULL) ||
        !bsp_device_is_initialized(&config->pwm->super) || (config->frequency_hz == 0U) ||
        !isfinite(config->minimum_pulse_width_us) || !isfinite(config->neutral_pulse_width_us) ||
        !isfinite(config->maximum_pulse_width_us) || (config->minimum_pulse_width_us <= 0.0F) ||
        (config->neutral_pulse_width_us < config->minimum_pulse_width_us) ||
        (config->neutral_pulse_width_us > config->maximum_pulse_width_us) ||
        !isfinite(config->minimum_angle_rad) || !isfinite(config->maximum_angle_rad) ||
        (config->minimum_angle_rad >= config->maximum_angle_rad) ||
        ((config->maximum_pulse_width_us * (float)config->frequency_hz) >= 1000000.0F))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    *me = (module_servo_t){0};
    me->pwm = config->pwm;
    me->frequency_hz = config->frequency_hz;
    me->minimum_pulse_width_us = config->minimum_pulse_width_us;
    me->neutral_pulse_width_us = config->neutral_pulse_width_us;
    me->maximum_pulse_width_us = config->maximum_pulse_width_us;
    me->minimum_angle_rad = config->minimum_angle_rad;
    me->maximum_angle_rad = config->maximum_angle_rad;
    me->commanded_angle_rad = 0.5F * (config->minimum_angle_rad + config->maximum_angle_rad);
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

module_servo_status_t module_servo_start(module_servo_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }
    if ((bsp_pwm_set_frequency(me->pwm, me->frequency_hz) != BSP_STATUS_OK) ||
        (bsp_pwm_start(me->pwm) != BSP_STATUS_OK))
    {
        return MODULE_SERVO_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    return module_servo_set_pulse_width(me, me->neutral_pulse_width_us);
}

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

module_servo_status_t module_servo_set_angle(module_servo_t *me, float angle_rad)
{
    float position_ratio;
    float pulse_width_us;

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
    angle_rad = module_servo_clamp(angle_rad, me->minimum_angle_rad, me->maximum_angle_rad);
    position_ratio =
        (angle_rad - me->minimum_angle_rad) / (me->maximum_angle_rad - me->minimum_angle_rad);
    pulse_width_us = me->minimum_pulse_width_us +
                     position_ratio * (me->maximum_pulse_width_us - me->minimum_pulse_width_us);
    if (module_servo_set_pulse_width(me, pulse_width_us) != MODULE_SERVO_STATUS_OK)
    {
        return MODULE_SERVO_STATUS_TRANSPORT_ERROR;
    }
    me->commanded_angle_rad = angle_rad;
    return MODULE_SERVO_STATUS_OK;
}

module_servo_status_t module_servo_set_normalized_output(module_servo_t *me,
                                                         float normalized_output)
{
    float pulse_width_us;

    if ((me == NULL) || !isfinite(normalized_output) || (normalized_output < -1.0F) ||
        (normalized_output > 1.0F))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_SERVO_STATUS_NOT_INITIALIZED;
    }
    pulse_width_us =
        (normalized_output >= 0.0F)
            ? me->neutral_pulse_width_us +
                  normalized_output * (me->maximum_pulse_width_us - me->neutral_pulse_width_us)
            : me->neutral_pulse_width_us +
                  normalized_output * (me->neutral_pulse_width_us - me->minimum_pulse_width_us);
    return module_servo_set_pulse_width(me, pulse_width_us);
}

module_servo_status_t module_servo_set_pulse_width(module_servo_t *me, float pulse_width_us)
{
    float duty_cycle;

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
    if ((pulse_width_us < me->minimum_pulse_width_us) ||
        (pulse_width_us > me->maximum_pulse_width_us))
    {
        return MODULE_SERVO_STATUS_INVALID_ARGUMENT;
    }
    duty_cycle = pulse_width_us * (float)me->frequency_hz / 1000000.0F;
    return (bsp_pwm_set_duty_cycle(me->pwm, duty_cycle) == BSP_STATUS_OK)
               ? MODULE_SERVO_STATUS_OK
               : MODULE_SERVO_STATUS_TRANSPORT_ERROR;
}

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
