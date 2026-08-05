# Chassis application

## Responsibility

`app_chassis` converts a body/reference-frame velocity command into four swerve-module targets.
It owns chassis mode transitions but delegates kinematics to `alg_swerve` and individual wheel
actuation to `module_swerve`.

## Supported modes

- No-force: all drive and steering motors are disabled.
- Normal: translation is expressed relative to the gimbal heading.
- Spin: translation remains gimbal-relative while the body rotates continuously.
- Follow-gimbal: body angular velocity closes the gimbal/body heading error.
- Stationary self-lock: zero wheel speed with steering axes forming a mechanical lock pattern.

## Inputs and outputs

Input is `app_chassis_command_t`. Output is `app_chassis_feedback_t`, optionally mirrored to the
other controller through `module_board_comm`.

## Validation

- No-force disables all eight chassis motors.
- Spin does not rotate the reference-frame translation vector.
- A zero command enters self-lock when configured.
- A failed wheel module produces offline/degraded feedback.
