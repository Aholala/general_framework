#ifndef MODULE_SERVO_H
#define MODULE_SERVO_H

#include "bsp_pwm.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        MODULE_SERVO_STATUS_OK = 0,
        MODULE_SERVO_STATUS_INVALID_ARGUMENT,
        MODULE_SERVO_STATUS_NOT_INITIALIZED,
        MODULE_SERVO_STATUS_NOT_STARTED,
        MODULE_SERVO_STATUS_TRANSPORT_ERROR
    } module_servo_status_t;

    typedef struct
    {
        bsp_pwm_t *pwm;
        uint32_t frequency_hz;
        float minimum_pulse_width_us;
        float neutral_pulse_width_us;
        float maximum_pulse_width_us;
        float minimum_angle_rad;
        float maximum_angle_rad;
        const char *logical_name;
        uint32_t registration_key;
    } module_servo_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_pwm_t *pwm;
        uint32_t frequency_hz;
        float minimum_pulse_width_us;
        float neutral_pulse_width_us;
        float maximum_pulse_width_us;
        float minimum_angle_rad;
        float maximum_angle_rad;
        float commanded_angle_rad;
        bool is_started;
    } module_servo_t;

    module_servo_status_t module_servo_init(module_servo_t *me,
                                            const module_servo_config_t *config);
    module_servo_status_t module_servo_start(module_servo_t *me);
    module_servo_status_t module_servo_stop(module_servo_t *me);
    module_servo_status_t module_servo_set_angle(module_servo_t *me, float angle_rad);
    module_servo_status_t module_servo_set_normalized_output(module_servo_t *me,
                                                             float normalized_output);
    module_servo_status_t module_servo_set_pulse_width(module_servo_t *me, float pulse_width_us);
    module_servo_status_t module_servo_get_commanded_angle(const module_servo_t *me,
                                                           float *angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SERVO_H */
