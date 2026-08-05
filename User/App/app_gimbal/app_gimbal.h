#ifndef APP_GIMBAL_H
#define APP_GIMBAL_H

#include "bsp_common.h"
#include "module_board_comm.h"
#include "module_motor.h"

#include <stdbool.h>

typedef struct
{
    module_motor_t *pitch_motor;
    module_motor_t *yaw_motor;
    module_board_comm_t *board_comm;
    float yaw_lqr_position_gain;
    float yaw_lqr_velocity_gain;
    float pitch_lqr_position_gain;
    float pitch_lqr_velocity_gain;
    float target_tolerance_rad;
} app_gimbal_config_t;

bsp_status_t app_gimbal_init(const app_gimbal_config_t *config);
void app_gimbal_update(float delta_time_s);

#endif
