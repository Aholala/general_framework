# Command application

## Responsibility

`app_command` converts operator input into typed gimbal, chassis and shooter commands. DR16 may be
local or forwarded from the other controller through `module_board_comm`.

## Inputs

- Four normalized DR16 channels, switches, mouse, keyboard and dial.
- Latest gimbal feedback.
- Latest valid visual target.

## Outputs

- `app_chassis_command_t`
- `app_gimbal_command_t`
- `app_shooter_command_t`
- Manual/automatic mode sent to the vision adapter.

## Mode mapping

- Invalid/offline input or left switch down: no-force.
- Left switch up: chassis spin mode.
- Right switch down: chassis follows gimbal.
- Otherwise: normal chassis mode.
- Mouse right selects automatic visual aiming when a valid target is available.

## Validation

- Offline DR16 always disables gimbal and chassis output.
- Pitch targets remain inside configured mechanical limits.
- A stale visual target never overrides manual control.
- One input update produces commands with one common sequence number.
