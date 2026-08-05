#include "app_imu.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

#include <math.h>

static app_imu_config_t app_imu_config;
static app_imu_snapshot_t app_imu_snapshot;
static bool app_imu_initialized;

bsp_status_t app_imu_init(const app_imu_config_t *config)
{
    if ((config == NULL) || (config->sensor == NULL) ||
        (config->accelerometer_correction_gain < 0.0F) ||
        (config->accelerometer_correction_gain > 1.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_imu_config = *config;
    app_imu_snapshot = (app_imu_snapshot_t){0};
    app_imu_initialized = true;
    return BSP_STATUS_OK;
}

void app_imu_update(float delta_time_s)
{
    const module_bmi088_process_data_t *data;
    float accelerometer_pitch_rad;
    float accelerometer_roll_rad;
    float gain;

    if (!app_imu_initialized)
    {
        return;
    }
    if (module_bmi088_read(app_imu_config.sensor) != MODULE_BMI088_STATUS_OK)
    {
        app_imu_snapshot.valid = false;
        app_exchange_publish_imu(&app_imu_snapshot);
        return;
    }
    data = module_bmi088_get_data(app_imu_config.sensor);
    if ((data == NULL) || !data->is_valid)
    {
        app_imu_snapshot.valid = false;
        app_exchange_publish_imu(&app_imu_snapshot);
        return;
    }

    app_imu_snapshot.roll_rad += data->angular_velocity_rad_per_s[0] * delta_time_s;
    app_imu_snapshot.pitch_rad += data->angular_velocity_rad_per_s[1] * delta_time_s;
    app_imu_snapshot.yaw_rad += data->angular_velocity_rad_per_s[2] * delta_time_s;
    accelerometer_roll_rad =
        atan2f(data->acceleration_m_per_s2[1], data->acceleration_m_per_s2[2]);
    accelerometer_pitch_rad =
        atan2f(-data->acceleration_m_per_s2[0],
               sqrtf((data->acceleration_m_per_s2[1] * data->acceleration_m_per_s2[1]) +
                     (data->acceleration_m_per_s2[2] * data->acceleration_m_per_s2[2])));
    gain = app_imu_config.accelerometer_correction_gain;
    app_imu_snapshot.roll_rad =
        ((1.0F - gain) * app_imu_snapshot.roll_rad) + (gain * accelerometer_roll_rad);
    app_imu_snapshot.pitch_rad =
        ((1.0F - gain) * app_imu_snapshot.pitch_rad) + (gain * accelerometer_pitch_rad);
    app_imu_snapshot.angular_velocity_rad_per_s[0] = data->angular_velocity_rad_per_s[0];
    app_imu_snapshot.angular_velocity_rad_per_s[1] = data->angular_velocity_rad_per_s[1];
    app_imu_snapshot.angular_velocity_rad_per_s[2] = data->angular_velocity_rad_per_s[2];
    app_imu_snapshot.sample_count = data->sample_count;
    app_imu_snapshot.valid = true;
    app_exchange_publish_imu(&app_imu_snapshot);
}
