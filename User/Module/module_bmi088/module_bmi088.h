#ifndef MODULE_BMI088_H
#define MODULE_BMI088_H

#include "bsp_spi.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct module_bmi088 module_bmi088_t;

    typedef enum
    {
        MODULE_BMI088_STATUS_OK = 0,
        MODULE_BMI088_STATUS_INVALID_ARGUMENT,
        MODULE_BMI088_STATUS_NOT_INITIALIZED,
        MODULE_BMI088_STATUS_TRANSPORT_ERROR,
        MODULE_BMI088_STATUS_ACCEL_NOT_FOUND,
        MODULE_BMI088_STATUS_GYRO_NOT_FOUND,
        MODULE_BMI088_STATUS_REGISTER_VERIFY_FAILED,
        MODULE_BMI088_STATUS_SELF_TEST_FAILED,
        MODULE_BMI088_STATUS_CALIBRATION_MOTION,
        MODULE_BMI088_STATUS_OUT_OF_RANGE
    } module_bmi088_status_t;

    typedef enum
    {
        MODULE_BMI088_ACCEL_RANGE_3G = 0,
        MODULE_BMI088_ACCEL_RANGE_6G,
        MODULE_BMI088_ACCEL_RANGE_12G,
        MODULE_BMI088_ACCEL_RANGE_24G
    } module_bmi088_accel_range_t;

    typedef enum
    {
        MODULE_BMI088_GYRO_RANGE_2000_DPS = 0,
        MODULE_BMI088_GYRO_RANGE_1000_DPS,
        MODULE_BMI088_GYRO_RANGE_500_DPS,
        MODULE_BMI088_GYRO_RANGE_250_DPS,
        MODULE_BMI088_GYRO_RANGE_125_DPS
    } module_bmi088_gyro_range_t;

    typedef enum
    {
        MODULE_BMI088_SENSOR_ACCEL = 0,
        MODULE_BMI088_SENSOR_GYRO
    } module_bmi088_sensor_t;

    typedef struct
    {
        uint8_t source_axis;
        float direction_sign;
    } module_bmi088_axis_map_t;

    typedef struct
    {
        int16_t acceleration[3];
        int16_t angular_velocity[3];
        int16_t temperature;
    } module_bmi088_raw_data_t;

    typedef struct
    {
        float acceleration_m_per_s2[3];
        float angular_velocity_rad_per_s[3];
        float temperature_c;
        uint32_t timestamp_us;
        uint32_t sample_interval_us;
        uint32_t sample_count;
        uint32_t failed_sample_count;
        bool is_valid;
    } module_bmi088_data_t;

    typedef void (*module_bmi088_chip_select_t)(void *user_context, module_bmi088_sensor_t sensor,
                                                bool is_selected);
    typedef void (*module_bmi088_delay_ms_t)(void *user_context, uint32_t delay_ms);
    typedef uint32_t (*module_bmi088_get_time_us_t)(void *user_context);

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        bsp_spi_t *spi;
        module_bmi088_chip_select_t set_chip_select;
        module_bmi088_delay_ms_t delay_ms;
        module_bmi088_get_time_us_t get_time_us;
        void *user_context;
        module_bmi088_accel_range_t acceleration_range;
        module_bmi088_gyro_range_t angular_velocity_range;
        module_bmi088_axis_map_t axis_map[3];
        uint32_t transfer_timeout_ms;
    } module_bmi088_config_t;

    struct module_bmi088
    {
        module_device_t super;
        bsp_spi_t *spi;
        module_bmi088_chip_select_t set_chip_select;
        module_bmi088_delay_ms_t delay_ms;
        module_bmi088_get_time_us_t get_time_us;
        void *user_context;
        module_bmi088_accel_range_t acceleration_range;
        module_bmi088_gyro_range_t angular_velocity_range;
        module_bmi088_axis_map_t axis_map[3];
        module_bmi088_raw_data_t raw_data;
        module_bmi088_data_t data;
        float acceleration_scale_m_per_s2;
        float angular_velocity_scale_rad_per_s;
        float angular_velocity_bias_rad_per_s[3];
        uint32_t transfer_timeout_ms;
        bool has_timestamp;
    };

    module_bmi088_status_t module_bmi088_init(module_bmi088_t *const me,
                                              const module_bmi088_config_t *const config);
    module_bmi088_status_t
    module_bmi088_configure(module_bmi088_t *const me,
                            module_bmi088_accel_range_t acceleration_range,
                            module_bmi088_gyro_range_t angular_velocity_range);
    module_bmi088_status_t module_bmi088_read(module_bmi088_t *const me);
    module_bmi088_status_t module_bmi088_run_self_test(module_bmi088_t *const me);
    module_bmi088_status_t module_bmi088_calibrate_gyroscope(module_bmi088_t *const me,
                                                             uint32_t sample_count,
                                                             uint32_t sample_interval_ms,
                                                             float maximum_stationary_deviation);
    module_bmi088_status_t
    module_bmi088_set_gyroscope_bias(module_bmi088_t *const me,
                                     const float angular_velocity_bias_rad_per_s[3]);
    const module_bmi088_data_t *module_bmi088_get_data(const module_bmi088_t *const me);
    const module_bmi088_raw_data_t *module_bmi088_get_raw_data(const module_bmi088_t *const me);
    module_device_t *module_bmi088_as_device(module_bmi088_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BMI088_H */
