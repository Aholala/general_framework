#include "app_chassis.h"

#include "app_config.h"
#include "app_exchange.h"
#include "app_types.h"

#include <math.h>

static app_chassis_config_t app_chassis_config;
static bool app_chassis_initialized;

static void app_chassis_disable_all(void)
{
    size_t index;
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        (void)module_swerve_disable(app_chassis_config.modules[index]);
    }
}

bool app_chassis_init(const app_chassis_config_t *config)
{
    size_t index;
    if ((config == NULL) || (config->kinematics == NULL))
    {
        return false;
    }
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        if (config->modules[index] == NULL)
        {
            return false;
        }
    }
    app_chassis_config = *config;
    app_chassis_initialized = true;
    return true;
}

void app_chassis_update(float delta_time_s)
{
    app_chassis_command_t input;
    app_chassis_feedback_t feedback = {0};
    alg_swerve_module_target_t targets[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
    alg_swerve_command_t command = {0};
    bool stopped;
    size_t index;

    if (!app_chassis_initialized)
    {
        return;
    }
    app_exchange_read_chassis_command(&input);
    if (!input.enabled || (input.mode == APP_CHASSIS_MODE_NO_FORCE))
    {
        app_chassis_disable_all();
        feedback.mode = APP_CHASSIS_MODE_NO_FORCE;
        app_exchange_publish_chassis_feedback(&feedback);
        return;
    }

    command.velocity_x_m_per_s = input.velocity_x_m_per_s;
    command.velocity_y_m_per_s = input.velocity_y_m_per_s;
    command.angular_velocity_rad_per_s = input.angular_velocity_rad_per_s;
    command.reference_heading_rad = input.gimbal_yaw_rad;
    command.command_is_reference_relative = true;
    if (input.mode == APP_CHASSIS_MODE_FOLLOW_GIMBAL)
    {
        command.angular_velocity_rad_per_s =
            APP_CHASSIS_FOLLOW_GAIN * alg_swerve_wrap_angle_rad(input.gimbal_yaw_rad);
    }

    stopped = (fabsf(command.velocity_x_m_per_s) < APP_CHASSIS_STOP_DEADBAND) &&
              (fabsf(command.velocity_y_m_per_s) < APP_CHASSIS_STOP_DEADBAND) &&
              (fabsf(command.angular_velocity_rad_per_s) < APP_CHASSIS_STOP_DEADBAND);
    if (stopped && input.self_lock_when_stopped)
    {
        if (alg_swerve_calculate_self_lock(app_chassis_config.kinematics, targets,
                                           ALG_SWERVE_RECTANGULAR_MODULE_COUNT) !=
            ALG_SWERVE_STATUS_OK)
        {
            app_chassis_disable_all();
            return;
        }
        feedback.self_lock_active = true;
    }
    else if (alg_swerve_calculate(app_chassis_config.kinematics, &command, targets,
                                  ALG_SWERVE_RECTANGULAR_MODULE_COUNT) != ALG_SWERVE_STATUS_OK)
    {
        app_chassis_disable_all();
        return;
    }

    feedback.motors_online = true;
    for (index = 0U; index < ALG_SWERVE_RECTANGULAR_MODULE_COUNT; ++index)
    {
        if (module_swerve_enable(app_chassis_config.modules[index]) != MODULE_SWERVE_STATUS_OK)
        {
            feedback.motors_online = false;
        }
        if (module_swerve_apply_target(app_chassis_config.modules[index], &targets[index],
                                       delta_time_s) != MODULE_SWERVE_STATUS_OK)
        {
            feedback.motors_online = false;
        }
    }
    feedback.velocity_x_m_per_s = command.velocity_x_m_per_s;
    feedback.velocity_y_m_per_s = command.velocity_y_m_per_s;
    feedback.angular_velocity_rad_per_s = command.angular_velocity_rad_per_s;
    feedback.mode = input.mode;
    app_exchange_publish_chassis_feedback(&feedback);

    if (app_chassis_config.board_comm != NULL)
    {
        const module_board_comm_chassis_process_data_t board_data = {
            .velocity_x_m_per_s = feedback.velocity_x_m_per_s,
            .velocity_y_m_per_s = feedback.velocity_y_m_per_s,
            .angular_velocity_rad_per_s = feedback.angular_velocity_rad_per_s,
            .motors_online = feedback.motors_online,
            .self_lock_active = feedback.self_lock_active,
        };
        (void)module_board_comm_send_chassis(app_chassis_config.board_comm, &board_data);
    }
}
