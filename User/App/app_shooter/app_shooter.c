#include "app_shooter.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

static app_shooter_config_t app_shooter_config;
static bool app_shooter_previous_fire_request;
static bool app_shooter_initialized;

bsp_status_t app_shooter_init(const app_shooter_config_t *config)
{
    if ((config == NULL) || (config->shooter == NULL))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    app_shooter_config = *config;
    app_shooter_previous_fire_request = false;
    app_shooter_initialized = true;
    return BSP_STATUS_OK;
}

void app_shooter_update(float delta_time_s)
{
    app_shooter_command_t command;
    app_gimbal_feedback_t gimbal;
    app_shooter_feedback_t feedback;

    if (!app_shooter_initialized)
    {
        return;
    }
    app_exchange_read_shooter_command(&command);
    app_exchange_read_gimbal_feedback(&gimbal);
    if (command.friction_enabled)
    {
        (void)module_shooter_enable(app_shooter_config.shooter);
    }
    else
    {
        (void)module_shooter_disable(app_shooter_config.shooter);
    }
    (void)module_shooter_set_friction(app_shooter_config.shooter, command.friction_enabled,
                                      command.friction_velocity_rad_per_s);
    if (command.fire_requested && !app_shooter_previous_fire_request)
    {
        (void)module_shooter_request_shots(app_shooter_config.shooter, 1U);
    }
    app_shooter_previous_fire_request = command.fire_requested;

    if (command.automatic_fire_enabled)
    {
        const module_shooter_fire_control_input_t fire_control = {
            .automatic_fire_enabled = true,
            .tracking_ready = gimbal.target_locked,
            .referee_allows_fire = true,
        };
        (void)module_shooter_update_fire_control(app_shooter_config.shooter, &fire_control,
                                                 delta_time_s);
    }
    (void)module_shooter_update(app_shooter_config.shooter, delta_time_s);

    feedback.state = (uint8_t)module_shooter_get_state(app_shooter_config.shooter);
    feedback.jam_retry_count = module_shooter_get_jam_retry_count(app_shooter_config.shooter);
    feedback.friction_ready = module_shooter_get_friction_ready(app_shooter_config.shooter);
    feedback.fire_permission = module_shooter_get_fire_permission(app_shooter_config.shooter);
    app_exchange_publish_shooter_feedback(&feedback);
    if (app_shooter_config.board_comm != NULL)
    {
        const module_board_comm_shooter_process_data_t board_data = {
            .state = feedback.state,
            .jam_retry_count = feedback.jam_retry_count,
            .friction_ready = feedback.friction_ready,
            .fire_permission = feedback.fire_permission,
        };
        if (module_board_comm_send_shooter(app_shooter_config.board_comm, &board_data) !=
            MODULE_BOARD_COMM_STATUS_OK)
        {
            bsp_error_record(BSP_STATUS_IO_ERROR, "send_shooter", 0);
        }
    }
}
