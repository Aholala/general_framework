#include "app_gimbal.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

#include <math.h>

static app_gimbal_config_t app_gimbal_config;
static bool app_gimbal_initialized;

bsp_status_t app_gimbal_init(const app_gimbal_config_t *config)
{
    if ((config == NULL) || (config->pitch_motor == NULL) || (config->yaw_motor == NULL) ||
        (config->target_tolerance_rad < 0.0F))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_gimbal_config = *config;
    app_gimbal_initialized = true;
    return BSP_STATUS_OK;
}

void app_gimbal_update(float delta_time_s)
{
    app_gimbal_command_t command;
    app_imu_snapshot_t imu;
    app_gimbal_feedback_t feedback = {0};
    const module_motor_feedback_t *pitch_feedback;
    const module_motor_feedback_t *yaw_feedback;
    float pitch_position;
    float yaw_position;
    float pitch_target;
    float yaw_target;

    if (!app_gimbal_initialized)
    {
        return;
    }
    app_exchange_read_gimbal_command(&command);
    app_exchange_read_imu(&imu);
    pitch_feedback = module_motor_get_feedback(app_gimbal_config.pitch_motor);
    yaw_feedback = module_motor_get_feedback(app_gimbal_config.yaw_motor);
    if (!command.enabled || (pitch_feedback == NULL) || (yaw_feedback == NULL))
    {
        (void)module_motor_disable(app_gimbal_config.pitch_motor);
        (void)module_motor_disable(app_gimbal_config.yaw_motor);
        app_exchange_publish_gimbal_feedback(&feedback);
        return;
    }

    pitch_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                         ? imu.pitch_rad
                         : pitch_feedback->position_rad;
    yaw_position = (command.feedback_mode == APP_GIMBAL_FEEDBACK_IMU) && imu.valid
                       ? imu.yaw_rad
                       : yaw_feedback->position_rad;
    pitch_target = command.pitch_target_rad;
    yaw_target = command.yaw_target_rad;
    if (command.control_mode == APP_GIMBAL_CONTROL_LQR)
    {
        pitch_target = (app_gimbal_config.pitch_lqr_position_gain *
                        (command.pitch_target_rad - pitch_position)) -
                       (app_gimbal_config.pitch_lqr_velocity_gain *
                        pitch_feedback->velocity_rad_per_s);
        yaw_target = (app_gimbal_config.yaw_lqr_position_gain *
                      (command.yaw_target_rad - yaw_position)) -
                     (app_gimbal_config.yaw_lqr_velocity_gain *
                      yaw_feedback->velocity_rad_per_s);
    }

    (void)module_motor_enable(app_gimbal_config.pitch_motor);
    (void)module_motor_enable(app_gimbal_config.yaw_motor);
    (void)module_motor_set_target(app_gimbal_config.pitch_motor, pitch_target);
    (void)module_motor_set_target(app_gimbal_config.yaw_motor, yaw_target);
    (void)module_motor_update(app_gimbal_config.pitch_motor, delta_time_s);
    (void)module_motor_update(app_gimbal_config.yaw_motor, delta_time_s);

    feedback.pitch_rad = pitch_position;
    feedback.yaw_rad = yaw_position;
    feedback.pitch_velocity_rad_per_s = pitch_feedback->velocity_rad_per_s;
    feedback.yaw_velocity_rad_per_s = yaw_feedback->velocity_rad_per_s;
    feedback.motors_online = pitch_feedback->is_online && yaw_feedback->is_online;
    feedback.target_locked =
        (fabsf(command.pitch_target_rad - pitch_position) <=
         app_gimbal_config.target_tolerance_rad) &&
        (fabsf(command.yaw_target_rad - yaw_position) <= app_gimbal_config.target_tolerance_rad);
    app_exchange_publish_gimbal_feedback(&feedback);

    if (app_gimbal_config.board_comm != NULL)
    {
        const module_board_comm_gimbal_process_data_t board_data = {
            .yaw_rad = feedback.yaw_rad,
            .pitch_rad = feedback.pitch_rad,
            .yaw_velocity_rad_per_s = feedback.yaw_velocity_rad_per_s,
            .pitch_velocity_rad_per_s = feedback.pitch_velocity_rad_per_s,
            .imu_valid = imu.valid,
            .motors_online = feedback.motors_online,
        };
        if (module_board_comm_send_gimbal(app_gimbal_config.board_comm, &board_data) !=
            MODULE_BOARD_COMM_STATUS_OK)
        {
            bsp_error_record(BSP_STATUS_IO_ERROR, "send_gimbal", 0);
        }
    }
}
