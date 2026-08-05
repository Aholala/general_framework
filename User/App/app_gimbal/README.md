# Gimbal application

## Responsibility

`app_gimbal` controls a two-axis gimbal through the generic `module_motor_t` interface. The project
mapping is DM4310 pitch on CAN1 and voltage-firmware GM6020 yaw on CAN2.

## Selectable control

- PID: angle targets are passed to motors configured for cascaded angle control.
- LQR template: position and velocity errors are combined using injected gains.
- Encoder lock: motor encoder position is the feedback source.
- IMU lock: the latest valid IMU attitude is the feedback source.

## Inputs and outputs

Input is `app_gimbal_command_t` plus `app_imu_snapshot_t`. Output is
`app_gimbal_feedback_t`, including angles, angular velocities, online state and target-lock state.
The feedback is also available to the chassis board and vision computer.

## Validation

- Invalid/disabled command disables both axes.
- IMU mode falls back safely when the IMU snapshot is invalid.
- Pitch remains limited by Command application configuration.
- Target lock requires both axes inside the configured tolerance.
