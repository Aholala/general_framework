#ifndef APP_TYPES_H
#define APP_TYPES_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    APP_CHASSIS_MODE_NO_FORCE = 0,
    APP_CHASSIS_MODE_NORMAL,
    APP_CHASSIS_MODE_SPIN,
    APP_CHASSIS_MODE_FOLLOW_GIMBAL
} app_chassis_mode_t;

typedef enum
{
    APP_GIMBAL_CONTROL_PID = 0,
    APP_GIMBAL_CONTROL_LQR
} app_gimbal_control_mode_t;

typedef enum
{
    APP_GIMBAL_FEEDBACK_ENCODER = 0,
    APP_GIMBAL_FEEDBACK_IMU
} app_gimbal_feedback_mode_t;

typedef struct
{
    float velocity_x_m_per_s;
    float velocity_y_m_per_s;
    float angular_velocity_rad_per_s;
    float gimbal_yaw_rad;
    app_chassis_mode_t mode;
    bool self_lock_when_stopped;
    bool enabled;
    uint32_t sequence;
} app_chassis_command_t;

typedef struct
{
    float yaw_target_rad;
    float pitch_target_rad;
    app_gimbal_control_mode_t control_mode;
    app_gimbal_feedback_mode_t feedback_mode;
    bool enabled;
    uint32_t sequence;
} app_gimbal_command_t;

typedef struct
{
    bool friction_enabled;
    bool fire_requested;
    bool automatic_fire_enabled;
    float friction_velocity_rad_per_s;
    uint32_t sequence;
} app_shooter_command_t;

typedef struct
{
    float yaw_rad;
    float pitch_rad;
    float roll_rad;
    float angular_velocity_rad_per_s[3];
    uint32_t sample_count;
    bool valid;
} app_imu_snapshot_t;

typedef struct
{
    float yaw_rad;
    float pitch_rad;
    float yaw_velocity_rad_per_s;
    float pitch_velocity_rad_per_s;
    bool motors_online;
    bool target_locked;
} app_gimbal_feedback_t;

typedef struct
{
    float velocity_x_m_per_s;
    float velocity_y_m_per_s;
    float angular_velocity_rad_per_s;
    app_chassis_mode_t mode;
    bool self_lock_active;
    bool motors_online;
} app_chassis_feedback_t;

typedef struct
{
    uint8_t state;
    uint8_t jam_retry_count;
    bool friction_ready;
    bool fire_permission;
} app_shooter_feedback_t;

typedef struct
{
    float target_yaw_rad;
    float target_pitch_rad;
    uint32_t update_count;
    bool target_valid;
    bool tracking_ready;
} app_vision_target_t;

#endif
