#include "app_exchange.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static app_chassis_command_t app_exchange_chassis_command;
static app_gimbal_command_t app_exchange_gimbal_command;
static app_shooter_command_t app_exchange_shooter_command;
static app_imu_snapshot_t app_exchange_imu_snapshot;
static app_gimbal_feedback_t app_exchange_gimbal_feedback;
static app_chassis_feedback_t app_exchange_chassis_feedback;
static app_shooter_feedback_t app_exchange_shooter_feedback;
static app_vision_target_t app_exchange_vision_target;

#define APP_EXCHANGE_DEFINE(type, suffix, storage)                                           \
    void app_exchange_publish_##suffix(const type *value)                                    \
    {                                                                                         \
        if (value != NULL)                                                                    \
        {                                                                                     \
            taskENTER_CRITICAL();                                                             \
            storage = *value;                                                                 \
            taskEXIT_CRITICAL();                                                              \
        }                                                                                     \
    }                                                                                         \
    void app_exchange_read_##suffix(type *value)                                             \
    {                                                                                         \
        if (value != NULL)                                                                    \
        {                                                                                     \
            taskENTER_CRITICAL();                                                             \
            *value = storage;                                                                 \
            taskEXIT_CRITICAL();                                                              \
        }                                                                                     \
    }

void app_exchange_init(void)
{
    taskENTER_CRITICAL();
    memset(&app_exchange_chassis_command, 0, sizeof(app_exchange_chassis_command));
    memset(&app_exchange_gimbal_command, 0, sizeof(app_exchange_gimbal_command));
    memset(&app_exchange_shooter_command, 0, sizeof(app_exchange_shooter_command));
    memset(&app_exchange_imu_snapshot, 0, sizeof(app_exchange_imu_snapshot));
    memset(&app_exchange_gimbal_feedback, 0, sizeof(app_exchange_gimbal_feedback));
    memset(&app_exchange_chassis_feedback, 0, sizeof(app_exchange_chassis_feedback));
    memset(&app_exchange_shooter_feedback, 0, sizeof(app_exchange_shooter_feedback));
    memset(&app_exchange_vision_target, 0, sizeof(app_exchange_vision_target));
    taskEXIT_CRITICAL();
}

APP_EXCHANGE_DEFINE(app_chassis_command_t, chassis_command, app_exchange_chassis_command)
APP_EXCHANGE_DEFINE(app_gimbal_command_t, gimbal_command, app_exchange_gimbal_command)
APP_EXCHANGE_DEFINE(app_shooter_command_t, shooter_command, app_exchange_shooter_command)
APP_EXCHANGE_DEFINE(app_imu_snapshot_t, imu, app_exchange_imu_snapshot)
APP_EXCHANGE_DEFINE(app_gimbal_feedback_t, gimbal_feedback, app_exchange_gimbal_feedback)
APP_EXCHANGE_DEFINE(app_chassis_feedback_t, chassis_feedback, app_exchange_chassis_feedback)
APP_EXCHANGE_DEFINE(app_shooter_feedback_t, shooter_feedback, app_exchange_shooter_feedback)
APP_EXCHANGE_DEFINE(app_vision_target_t, vision_target, app_exchange_vision_target)
