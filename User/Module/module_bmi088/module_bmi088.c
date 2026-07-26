#include "module_bmi088.h"

#include "module_bmi088_registers.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define MODULE_BMI088_STANDARD_GRAVITY (9.80665F)
#define MODULE_BMI088_DEGREES_TO_RADIANS (0.01745329251994329577F)
#define MODULE_BMI088_MAX_TRANSFER_SIZE (10U)

typedef struct
{
    uint8_t register_address;
    uint8_t register_value;
} module_bmi088_register_config_t;

static module_bmi088_status_t module_bmi088_exchange(module_bmi088_t *const me,
                                                     module_bmi088_sensor_t sensor,
                                                     const uint8_t *transmit_data,
                                                     uint8_t *receive_data, size_t data_size)
{
    bsp_status_t status;
    me->set_chip_select(me->user_context, sensor, true);
    status = bsp_spi_exchange(me->spi, transmit_data, receive_data, data_size,
                              BSP_TRANSFER_MODE_BLOCKING, me->transfer_timeout_ms);
    me->set_chip_select(me->user_context, sensor, false);
    return (status == BSP_STATUS_OK) ? MODULE_BMI088_STATUS_OK
                                     : MODULE_BMI088_STATUS_TRANSPORT_ERROR;
}

static module_bmi088_status_t module_bmi088_write_register(module_bmi088_t *const me,
                                                           module_bmi088_sensor_t sensor,
                                                           uint8_t register_address,
                                                           uint8_t register_value)
{
    const uint8_t transmit_data[2] = {register_address, register_value};
    uint8_t receive_data[2] = {0U};
    return module_bmi088_exchange(me, sensor, transmit_data, receive_data, sizeof(transmit_data));
}

static module_bmi088_status_t module_bmi088_read_registers(module_bmi088_t *const me,
                                                           module_bmi088_sensor_t sensor,
                                                           uint8_t register_address,
                                                           uint8_t *receive_data, size_t data_size)
{
    uint8_t transmit_buffer[MODULE_BMI088_MAX_TRANSFER_SIZE] = {0U};
    uint8_t receive_buffer[MODULE_BMI088_MAX_TRANSFER_SIZE] = {0U};
    size_t protocol_overhead = (sensor == MODULE_BMI088_SENSOR_ACCEL) ? 2U : 1U;

    if ((receive_data == NULL) || (data_size == 0U) ||
        ((data_size + protocol_overhead) > MODULE_BMI088_MAX_TRANSFER_SIZE))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    transmit_buffer[0] = register_address | MODULE_BMI088_SPI_READ_BIT;
    if (module_bmi088_exchange(me, sensor, transmit_buffer, receive_buffer,
                               data_size + protocol_overhead) != MODULE_BMI088_STATUS_OK)
    {
        return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    (void)memcpy(receive_data, &receive_buffer[protocol_overhead], data_size);
    return MODULE_BMI088_STATUS_OK;
}

static module_bmi088_status_t module_bmi088_write_and_verify(module_bmi088_t *const me,
                                                             module_bmi088_sensor_t sensor,
                                                             uint8_t register_address,
                                                             uint8_t register_value)
{
    uint8_t read_value;
    module_bmi088_status_t status =
        module_bmi088_write_register(me, sensor, register_address, register_value);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }
    me->delay_ms(me->user_context, 1U);
    status = module_bmi088_read_registers(me, sensor, register_address, &read_value, 1U);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }
    return (read_value == register_value) ? MODULE_BMI088_STATUS_OK
                                          : MODULE_BMI088_STATUS_REGISTER_VERIFY_FAILED;
}

static float module_bmi088_get_acceleration_scale(module_bmi088_accel_range_t acceleration_range)
{
    static const float range_g[] = {3.0F, 6.0F, 12.0F, 24.0F};
    return range_g[acceleration_range] * MODULE_BMI088_STANDARD_GRAVITY / 32768.0F;
}

static float
module_bmi088_get_angular_velocity_scale(module_bmi088_gyro_range_t angular_velocity_range)
{
    static const float range_dps[] = {2000.0F, 1000.0F, 500.0F, 250.0F, 125.0F};
    return range_dps[angular_velocity_range] * MODULE_BMI088_DEGREES_TO_RADIANS / 32768.0F;
}

static int16_t module_bmi088_decode_int16(const uint8_t data[2])
{
    return (int16_t)(((uint16_t)data[1] << 8U) | data[0]);
}

static module_bmi088_status_t
module_bmi088_validate_axis_map(const module_bmi088_axis_map_t axis_map[3])
{
    bool source_is_used[3] = {false, false, false};
    size_t axis_index;
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        if ((axis_map[axis_index].source_axis > 2U) ||
            ((axis_map[axis_index].direction_sign != 1.0F) &&
             (axis_map[axis_index].direction_sign != -1.0F)) ||
            source_is_used[axis_map[axis_index].source_axis])
        {
            return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
        }
        source_is_used[axis_map[axis_index].source_axis] = true;
    }
    return MODULE_BMI088_STATUS_OK;
}

static module_device_status_t module_bmi088_device_start(module_device_t *const device_base)
{
    (void)device_base;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_bmi088_device_stop(module_device_t *const device_base)
{
    (void)device_base;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_bmi088_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_bmi088_t *const me = MODULE_CONTAINER_OF(device_base, module_bmi088_t, super);
    (void)elapsed_time_ms;
    return (module_bmi088_read(me) == MODULE_BMI088_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static const module_device_ops_t s_module_bmi088_device_ops = {
    .start = module_bmi088_device_start,
    .stop = module_bmi088_device_stop,
    .update = module_bmi088_device_update,
};

static module_bmi088_status_t
module_bmi088_configure_internal(module_bmi088_t *const me,
                                 module_bmi088_accel_range_t acceleration_range,
                                 module_bmi088_gyro_range_t angular_velocity_range)
{
    static const uint8_t acceleration_range_values[] = {0x00U, 0x01U, 0x02U, 0x03U};
    static const uint8_t angular_velocity_range_values[] = {0x00U, 0x01U, 0x02U, 0x03U, 0x04U};
    if ((me == NULL) || (acceleration_range > MODULE_BMI088_ACCEL_RANGE_24G) ||
        (angular_velocity_range > MODULE_BMI088_GYRO_RANGE_125_DPS))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    const module_bmi088_register_config_t acceleration_config[] = {
        {MODULE_BMI088_ACCEL_POWER_CONTROL_REGISTER, MODULE_BMI088_ACCEL_POWER_ON},
        {MODULE_BMI088_ACCEL_POWER_CONFIG_REGISTER, MODULE_BMI088_ACCEL_ACTIVE_MODE},
        {MODULE_BMI088_ACCEL_CONFIG_REGISTER, MODULE_BMI088_ACCEL_CONFIG_NORMAL_800_HZ},
        {MODULE_BMI088_ACCEL_RANGE_REGISTER, acceleration_range_values[acceleration_range]},
        {MODULE_BMI088_ACCEL_INTERRUPT_IO_REGISTER, MODULE_BMI088_ACCEL_INTERRUPT_OUTPUT_ENABLE},
        {MODULE_BMI088_ACCEL_INTERRUPT_MAP_REGISTER, MODULE_BMI088_ACCEL_INTERRUPT_MAP_DATA_READY}};
    const module_bmi088_register_config_t gyroscope_config[] = {
        {MODULE_BMI088_GYRO_RANGE_REGISTER, angular_velocity_range_values[angular_velocity_range]},
        {MODULE_BMI088_GYRO_BANDWIDTH_REGISTER, MODULE_BMI088_GYRO_BANDWIDTH_2000_230_HZ},
        {MODULE_BMI088_GYRO_POWER_REGISTER, MODULE_BMI088_GYRO_NORMAL_MODE},
        {MODULE_BMI088_GYRO_INTERRUPT_CONTROL_REGISTER, MODULE_BMI088_GYRO_DATA_READY_ENABLE},
        {MODULE_BMI088_GYRO_INTERRUPT_IO_REGISTER,
         MODULE_BMI088_GYRO_INTERRUPT_PUSH_PULL_ACTIVE_LOW},
        {MODULE_BMI088_GYRO_INTERRUPT_MAP_REGISTER,
         MODULE_BMI088_GYRO_INTERRUPT_MAP_DATA_READY_INT3}};
    size_t register_index;
    module_bmi088_status_t status;

    for (register_index = 0U;
         register_index < (sizeof(acceleration_config) / sizeof(acceleration_config[0]));
         ++register_index)
    {
        status = module_bmi088_write_and_verify(
            me, MODULE_BMI088_SENSOR_ACCEL, acceleration_config[register_index].register_address,
            acceleration_config[register_index].register_value);
        if (status != MODULE_BMI088_STATUS_OK)
        {
            return status;
        }
    }
    for (register_index = 0U;
         register_index < (sizeof(gyroscope_config) / sizeof(gyroscope_config[0]));
         ++register_index)
    {
        status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_GYRO,
                                                gyroscope_config[register_index].register_address,
                                                gyroscope_config[register_index].register_value);
        if (status != MODULE_BMI088_STATUS_OK)
        {
            return status;
        }
    }
    me->acceleration_range = acceleration_range;
    me->angular_velocity_range = angular_velocity_range;
    me->acceleration_scale_m_per_s2 = module_bmi088_get_acceleration_scale(acceleration_range);
    me->angular_velocity_scale_rad_per_s =
        module_bmi088_get_angular_velocity_scale(angular_velocity_range);
    return MODULE_BMI088_STATUS_OK;
}

module_bmi088_status_t module_bmi088_configure(module_bmi088_t *const me,
                                               module_bmi088_accel_range_t acceleration_range,
                                               module_bmi088_gyro_range_t angular_velocity_range)
{
    if (me == NULL)
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }
    return module_bmi088_configure_internal(me, acceleration_range, angular_velocity_range);
}

module_bmi088_status_t module_bmi088_init(module_bmi088_t *const me,
                                          const module_bmi088_config_t *const config)
{
    uint8_t chip_identifier;
    size_t axis_index;
    module_bmi088_status_t status;

    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->set_chip_select == NULL) ||
        (config->delay_ms == NULL) ||
        (module_bmi088_validate_axis_map(config->axis_map) != MODULE_BMI088_STATUS_OK))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_init_base(&me->super, &s_module_bmi088_device_ops,
                                (config->logical_name != NULL) ? config->logical_name : "bmi088",
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    me->spi = config->spi;
    me->set_chip_select = config->set_chip_select;
    me->delay_ms = config->delay_ms;
    me->get_time_us = config->get_time_us;
    me->user_context = config->user_context;
    me->transfer_timeout_ms = config->transfer_timeout_ms;
    me->raw_data = (module_bmi088_raw_data_t){0};
    me->data = (module_bmi088_data_t){0};
    me->has_timestamp = false;
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        me->axis_map[axis_index] = config->axis_map[axis_index];
        me->angular_velocity_bias_rad_per_s[axis_index] = 0.0F;
    }
    me->set_chip_select(me->user_context, MODULE_BMI088_SENSOR_ACCEL, false);
    me->set_chip_select(me->user_context, MODULE_BMI088_SENSOR_GYRO, false);

    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                       MODULE_BMI088_ACCEL_SOFT_RESET_REGISTER,
                                       MODULE_BMI088_ACCEL_SOFT_RESET_COMMAND);
    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_GYRO,
                                       MODULE_BMI088_GYRO_SOFT_RESET_REGISTER,
                                       MODULE_BMI088_GYRO_SOFT_RESET_COMMAND);
    me->delay_ms(me->user_context, 80U);

    status = module_bmi088_read_registers(
        me, MODULE_BMI088_SENSOR_ACCEL, MODULE_BMI088_ACCEL_CHIP_ID_REGISTER, &chip_identifier, 1U);
    if ((status != MODULE_BMI088_STATUS_OK) ||
        (chip_identifier != MODULE_BMI088_ACCEL_CHIP_ID_VALUE))
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_ACCEL_NOT_FOUND;
    }
    status = module_bmi088_read_registers(
        me, MODULE_BMI088_SENSOR_GYRO, MODULE_BMI088_GYRO_CHIP_ID_REGISTER, &chip_identifier, 1U);
    if ((status != MODULE_BMI088_STATUS_OK) ||
        (chip_identifier != MODULE_BMI088_GYRO_CHIP_ID_VALUE))
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_GYRO_NOT_FOUND;
    }
    status = module_bmi088_configure_internal(me, config->acceleration_range,
                                              config->angular_velocity_range);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return status;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BMI088_STATUS_OK;
}

module_bmi088_status_t module_bmi088_read(module_bmi088_t *const me)
{
    uint8_t acceleration_data[6];
    uint8_t angular_velocity_data[6];
    uint8_t temperature_data[2];
    int16_t sensor_acceleration[3];
    int16_t sensor_angular_velocity[3];
    int16_t temperature_raw;
    size_t axis_index;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_BMI088_STATUS_INVALID_ARGUMENT
                            : MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }
    if ((module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                      MODULE_BMI088_ACCEL_DATA_REGISTER, acceleration_data,
                                      6U) != MODULE_BMI088_STATUS_OK) ||
        (module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_GYRO,
                                      MODULE_BMI088_GYRO_DATA_REGISTER, angular_velocity_data,
                                      6U) != MODULE_BMI088_STATUS_OK) ||
        (module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                      MODULE_BMI088_ACCEL_TEMPERATURE_REGISTER, temperature_data,
                                      2U) != MODULE_BMI088_STATUS_OK))
    {
        me->data.is_valid = false;
        ++me->data.failed_sample_count;
        return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        sensor_acceleration[axis_index] =
            module_bmi088_decode_int16(&acceleration_data[axis_index * 2U]);
        sensor_angular_velocity[axis_index] =
            module_bmi088_decode_int16(&angular_velocity_data[axis_index * 2U]);
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        const uint8_t source_axis = me->axis_map[axis_index].source_axis;
        const float direction_sign = me->axis_map[axis_index].direction_sign;
        me->raw_data.acceleration[axis_index] = sensor_acceleration[source_axis];
        me->raw_data.angular_velocity[axis_index] = sensor_angular_velocity[source_axis];
        me->data.acceleration_m_per_s2[axis_index] = direction_sign *
                                                     (float)sensor_acceleration[source_axis] *
                                                     me->acceleration_scale_m_per_s2;
        me->data.angular_velocity_rad_per_s[axis_index] =
            direction_sign * (float)sensor_angular_velocity[source_axis] *
                me->angular_velocity_scale_rad_per_s -
            me->angular_velocity_bias_rad_per_s[axis_index];
    }
    temperature_raw =
        (int16_t)(((uint16_t)temperature_data[0] << 3U) | (temperature_data[1] >> 5U));
    if (temperature_raw > 1023)
    {
        temperature_raw -= 2048;
    }
    me->raw_data.temperature = temperature_raw;
    me->data.temperature_c = (float)temperature_raw * 0.125F + 23.0F;
    if (me->get_time_us != NULL)
    {
        const uint32_t timestamp_us = me->get_time_us(me->user_context);
        me->data.sample_interval_us =
            me->has_timestamp ? (timestamp_us - me->data.timestamp_us) : 0U;
        me->data.timestamp_us = timestamp_us;
        me->has_timestamp = true;
    }
    ++me->data.sample_count;
    me->data.is_valid = true;
    return MODULE_BMI088_STATUS_OK;
}

module_bmi088_status_t module_bmi088_run_self_test(module_bmi088_t *const me)
{
    uint8_t positive_data[6];
    uint8_t negative_data[6];
    int32_t acceleration_delta[3];
    uint8_t self_test_status = 0U;
    uint32_t poll_count;
    size_t axis_index;
    module_bmi088_status_t status;
    const module_bmi088_accel_range_t saved_acceleration_range =
        (me != NULL) ? me->acceleration_range : MODULE_BMI088_ACCEL_RANGE_6G;
    const module_bmi088_gyro_range_t saved_angular_velocity_range =
        (me != NULL) ? me->angular_velocity_range : MODULE_BMI088_GYRO_RANGE_2000_DPS;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return (me == NULL) ? MODULE_BMI088_STATUS_INVALID_ARGUMENT
                            : MODULE_BMI088_STATUS_NOT_INITIALIZED;
    }
    status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_ACCEL,
                                            MODULE_BMI088_ACCEL_CONFIG_REGISTER, 0xACU);
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_and_verify(me, MODULE_BMI088_SENSOR_ACCEL,
                                                MODULE_BMI088_ACCEL_RANGE_REGISTER, 0x03U);
    }
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                              MODULE_BMI088_ACCEL_SELF_TEST_POSITIVE);
    }
    me->delay_ms(me->user_context, 50U);
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_DATA_REGISTER, positive_data, 6U);
    }
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                              MODULE_BMI088_ACCEL_SELF_TEST_NEGATIVE);
    }
    me->delay_ms(me->user_context, 50U);
    if (status == MODULE_BMI088_STATUS_OK)
    {
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_ACCEL,
                                              MODULE_BMI088_ACCEL_DATA_REGISTER, negative_data, 6U);
    }
    (void)module_bmi088_write_register(me, MODULE_BMI088_SENSOR_ACCEL,
                                       MODULE_BMI088_ACCEL_SELF_TEST_REGISTER,
                                       MODULE_BMI088_ACCEL_SELF_TEST_OFF);
    if (status == MODULE_BMI088_STATUS_OK)
    {
        for (axis_index = 0U; axis_index < 3U; ++axis_index)
        {
            acceleration_delta[axis_index] =
                (int32_t)module_bmi088_decode_int16(&positive_data[axis_index * 2U]) -
                (int32_t)module_bmi088_decode_int16(&negative_data[axis_index * 2U]);
            if (acceleration_delta[axis_index] < 0)
            {
                acceleration_delta[axis_index] = -acceleration_delta[axis_index];
            }
        }
        if ((acceleration_delta[0] < 1365) || (acceleration_delta[1] < 1365) ||
            (acceleration_delta[2] < 683))
        {
            status = MODULE_BMI088_STATUS_SELF_TEST_FAILED;
        }
    }
    if (status != MODULE_BMI088_STATUS_OK)
    {
        (void)module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
        return status;
    }

    status = module_bmi088_write_register(me, MODULE_BMI088_SENSOR_GYRO,
                                          MODULE_BMI088_GYRO_SELF_TEST_REGISTER,
                                          MODULE_BMI088_GYRO_SELF_TEST_TRIGGER);
    if (status != MODULE_BMI088_STATUS_OK)
    {
        return status;
    }
    for (poll_count = 0U; poll_count < 20U; ++poll_count)
    {
        me->delay_ms(me->user_context, 5U);
        status = module_bmi088_read_registers(me, MODULE_BMI088_SENSOR_GYRO,
                                              MODULE_BMI088_GYRO_SELF_TEST_REGISTER,
                                              &self_test_status, 1U);
        if ((status == MODULE_BMI088_STATUS_OK) &&
            ((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_READY) != 0U))
        {
            break;
        }
    }
    if (((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_READY) == 0U) ||
        ((self_test_status & MODULE_BMI088_GYRO_SELF_TEST_FAILED) != 0U))
    {
        (void)module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
        return MODULE_BMI088_STATUS_SELF_TEST_FAILED;
    }
    return module_bmi088_configure(me, saved_acceleration_range, saved_angular_velocity_range);
}

module_bmi088_status_t module_bmi088_calibrate_gyroscope(module_bmi088_t *const me,
                                                         uint32_t sample_count,
                                                         uint32_t sample_interval_ms,
                                                         float maximum_stationary_deviation)
{
    float sum[3] = {0.0F, 0.0F, 0.0F};
    float minimum[3] = {INFINITY, INFINITY, INFINITY};
    float maximum[3] = {-INFINITY, -INFINITY, -INFINITY};
    uint32_t sample_index;
    size_t axis_index;

    if ((me == NULL) || !module_device_is_initialized(&me->super) || (sample_count == 0U) ||
        (maximum_stationary_deviation <= 0.0F))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    for (sample_index = 0U; sample_index < sample_count; ++sample_index)
    {
        if (module_bmi088_read(me) != MODULE_BMI088_STATUS_OK)
        {
            return MODULE_BMI088_STATUS_TRANSPORT_ERROR;
        }
        for (axis_index = 0U; axis_index < 3U; ++axis_index)
        {
            const float unbiased_value = me->axis_map[axis_index].direction_sign *
                                         (float)me->raw_data.angular_velocity[axis_index] *
                                         me->angular_velocity_scale_rad_per_s;
            sum[axis_index] += unbiased_value;
            if (unbiased_value < minimum[axis_index])
            {
                minimum[axis_index] = unbiased_value;
            }
            if (unbiased_value > maximum[axis_index])
            {
                maximum[axis_index] = unbiased_value;
            }
        }
        me->delay_ms(me->user_context, sample_interval_ms);
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        if ((maximum[axis_index] - minimum[axis_index]) > maximum_stationary_deviation)
        {
            return MODULE_BMI088_STATUS_CALIBRATION_MOTION;
        }
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        me->angular_velocity_bias_rad_per_s[axis_index] = sum[axis_index] / (float)sample_count;
    }
    return MODULE_BMI088_STATUS_OK;
}

module_bmi088_status_t
module_bmi088_set_gyroscope_bias(module_bmi088_t *const me,
                                 const float angular_velocity_bias_rad_per_s[3])
{
    size_t axis_index;
    if ((me == NULL) || (angular_velocity_bias_rad_per_s == NULL) ||
        !module_device_is_initialized(&me->super))
    {
        return MODULE_BMI088_STATUS_INVALID_ARGUMENT;
    }
    for (axis_index = 0U; axis_index < 3U; ++axis_index)
    {
        if (!isfinite(angular_velocity_bias_rad_per_s[axis_index]))
        {
            return MODULE_BMI088_STATUS_OUT_OF_RANGE;
        }
        me->angular_velocity_bias_rad_per_s[axis_index] =
            angular_velocity_bias_rad_per_s[axis_index];
    }
    return MODULE_BMI088_STATUS_OK;
}

const module_bmi088_data_t *module_bmi088_get_data(const module_bmi088_t *const me)
{
    return ((me != NULL) && module_device_is_initialized(&me->super)) ? &me->data : NULL;
}

const module_bmi088_raw_data_t *module_bmi088_get_raw_data(const module_bmi088_t *const me)
{
    return ((me != NULL) && module_device_is_initialized(&me->super)) ? &me->raw_data : NULL;
}

module_device_t *module_bmi088_as_device(module_bmi088_t *const me)
{
    return (me != NULL) ? &me->super : NULL;
}
