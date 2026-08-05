#ifndef APP_IMU_H
#define APP_IMU_H

#include "bsp_common.h"
#include "module_bmi088.h"

#include <stdbool.h>

typedef struct
{
    module_bmi088_t *sensor;
    float accelerometer_correction_gain;
} app_imu_config_t;

bsp_status_t app_imu_init(const app_imu_config_t *config);
void app_imu_update(float delta_time_s);

#endif
