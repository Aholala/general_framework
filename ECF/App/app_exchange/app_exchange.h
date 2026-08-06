#ifndef APP_EXCHANGE_H
#define APP_EXCHANGE_H

#include "app_types.h"

void app_exchange_init(void);
void app_exchange_publish_chassis_command(const app_chassis_command_t *command);
void app_exchange_read_chassis_command(app_chassis_command_t *command);
void app_exchange_publish_gimbal_command(const app_gimbal_command_t *command);
void app_exchange_read_gimbal_command(app_gimbal_command_t *command);
void app_exchange_publish_shooter_command(const app_shooter_command_t *command);
void app_exchange_read_shooter_command(app_shooter_command_t *command);
void app_exchange_publish_imu(const app_imu_snapshot_t *snapshot);
void app_exchange_read_imu(app_imu_snapshot_t *snapshot);
void app_exchange_publish_gimbal_feedback(const app_gimbal_feedback_t *feedback);
void app_exchange_read_gimbal_feedback(app_gimbal_feedback_t *feedback);
void app_exchange_publish_chassis_feedback(const app_chassis_feedback_t *feedback);
void app_exchange_read_chassis_feedback(app_chassis_feedback_t *feedback);
void app_exchange_publish_shooter_feedback(const app_shooter_feedback_t *feedback);
void app_exchange_read_shooter_feedback(app_shooter_feedback_t *feedback);
void app_exchange_publish_vision_target(const app_vision_target_t *target);
void app_exchange_read_vision_target(app_vision_target_t *target);

#endif
