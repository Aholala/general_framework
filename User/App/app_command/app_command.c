#include "app_command.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"
#include "app_vision.h"

#include <string.h>

static app_command_config_t app_command_config;
static float app_command_yaw_target_rad;
static float app_command_pitch_target_rad;
static uint32_t app_command_sequence;
static bool app_command_initialized;

static bool app_command_get_remote(module_board_comm_remote_process_data_t *remote)
{
    if (app_command_config.dr16_is_local)
    {
        const module_dr16_process_data_t *const data =
            module_dr16_get_data(app_command_config.dr16);
        if ((data == NULL) || !data->is_online)
        {
            return false;
        }
        memset(remote, 0, sizeof(*remote));
        memcpy(remote->channel, data->channel, sizeof(remote->channel));
        remote->left_switch = (module_board_comm_switch_t)data->left_switch;
        remote->right_switch = (module_board_comm_switch_t)data->right_switch;
        remote->mouse_x = data->mouse_x;
        remote->mouse_y = data->mouse_y;
        remote->mouse_z = data->mouse_z;
        remote->mouse_left_pressed = data->mouse_left_pressed;
        remote->mouse_right_pressed = data->mouse_right_pressed;
        remote->keyboard = data->keyboard;
        remote->dial = data->dial;
        remote->is_online = true;
        remote->update_count = data->valid_frame_count;
        if (app_command_config.board_comm != NULL)
        {
            (void)module_board_comm_send_remote(app_command_config.board_comm, remote);
        }
        return true;
    }
    if (app_command_config.board_comm != NULL)
    {
        const module_board_comm_remote_process_data_t *const data =
            module_board_comm_get_remote(app_command_config.board_comm);
        if (data != NULL)
        {
            *remote = *data;
            return true;
        }
    }
    return false;
}

bsp_status_t app_command_init(const app_command_config_t *config)
{
    if ((config == NULL) || (config->dr16_is_local && (config->dr16 == NULL)))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_command_config = *config;
    app_command_yaw_target_rad = 0.0F;
    app_command_pitch_target_rad = 0.0F;
    app_command_sequence = 0U;
    app_command_initialized = true;
    return BSP_STATUS_OK;
}

void app_command_update(float delta_time_s)
{
    module_board_comm_remote_process_data_t remote;
    app_chassis_command_t chassis = {0};
    app_gimbal_command_t gimbal = {0};
    app_shooter_command_t shooter = {0};
    app_gimbal_feedback_t gimbal_feedback;
    app_vision_target_t vision_target;
    float channel[4];
    bool online;
    size_t index;

    if (!app_command_initialized)
    {
        return;
    }
    online = app_command_get_remote(&remote);
    app_exchange_read_gimbal_feedback(&gimbal_feedback);
    app_exchange_read_vision_target(&vision_target);
    for (index = 0U; index < 4U; ++index)
    {
        channel[index] = online ? module_dr16_normalize_channel_value(remote.channel[index]) : 0.0F;
    }

    app_command_yaw_target_rad +=
        channel[0] * APP_GIMBAL_MAX_YAW_RATE_RAD_PER_S * delta_time_s;
    app_command_pitch_target_rad +=
        channel[1] * APP_GIMBAL_MAX_PITCH_RATE_RAD_PER_S * delta_time_s;
    if (app_command_pitch_target_rad > APP_GIMBAL_MAX_PITCH_RAD)
    {
        app_command_pitch_target_rad = APP_GIMBAL_MAX_PITCH_RAD;
    }
    else if (app_command_pitch_target_rad < APP_GIMBAL_MIN_PITCH_RAD)
    {
        app_command_pitch_target_rad = APP_GIMBAL_MIN_PITCH_RAD;
    }

    chassis.enabled = online;
    chassis.velocity_x_m_per_s = channel[3] * APP_CHASSIS_MAX_VELOCITY_M_PER_S;
    chassis.velocity_y_m_per_s = channel[2] * APP_CHASSIS_MAX_VELOCITY_M_PER_S;
    chassis.self_lock_when_stopped = true;
    chassis.gimbal_yaw_rad = gimbal_feedback.yaw_rad;
    if (!online || (remote.left_switch == MODULE_BOARD_COMM_SWITCH_DOWN))
    {
        chassis.mode = APP_CHASSIS_MODE_NO_FORCE;
        chassis.enabled = false;
    }
    else if (remote.left_switch == MODULE_BOARD_COMM_SWITCH_UP)
    {
        chassis.mode = APP_CHASSIS_MODE_SPIN;
        chassis.angular_velocity_rad_per_s = APP_CHASSIS_MAX_SPIN_RAD_PER_S;
    }
    else if (remote.right_switch == MODULE_BOARD_COMM_SWITCH_DOWN)
    {
        chassis.mode = APP_CHASSIS_MODE_FOLLOW_GIMBAL;
    }
    else
    {
        chassis.mode = APP_CHASSIS_MODE_NORMAL;
    }

    gimbal.enabled = online;
    gimbal.yaw_target_rad = app_command_yaw_target_rad;
    gimbal.pitch_target_rad = app_command_pitch_target_rad;
    gimbal.control_mode = (remote.right_switch == MODULE_BOARD_COMM_SWITCH_UP)
                              ? APP_GIMBAL_CONTROL_LQR
                              : APP_GIMBAL_CONTROL_PID;
    gimbal.feedback_mode = (remote.right_switch == MODULE_BOARD_COMM_SWITCH_MIDDLE)
                               ? APP_GIMBAL_FEEDBACK_IMU
                               : APP_GIMBAL_FEEDBACK_ENCODER;
    if (vision_target.target_valid && remote.mouse_right_pressed)
    {
        gimbal.yaw_target_rad = vision_target.target_yaw_rad;
        gimbal.pitch_target_rad = vision_target.target_pitch_rad;
        gimbal.feedback_mode = APP_GIMBAL_FEEDBACK_IMU;
    }

    shooter.friction_enabled = online && (remote.dial > 100);
    shooter.fire_requested = shooter.friction_enabled && (remote.dial > 500);
    shooter.automatic_fire_enabled = remote.mouse_right_pressed && vision_target.tracking_ready;
    shooter.friction_velocity_rad_per_s = 500.0F;
    app_vision_set_mode(remote.mouse_right_pressed ? APP_VISION_MODE_AUTOMATIC
                                                   : APP_VISION_MODE_MANUAL);
    ++app_command_sequence;
    chassis.sequence = app_command_sequence;
    gimbal.sequence = app_command_sequence;
    shooter.sequence = app_command_sequence;
    app_exchange_publish_chassis_command(&chassis);
    app_exchange_publish_gimbal_command(&gimbal);
    app_exchange_publish_shooter_command(&shooter);
}
