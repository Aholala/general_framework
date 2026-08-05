# Shooter application

## Responsibility

`app_shooter` coordinates two M3508 friction motors and one M3508 feeder through
`module_shooter`. Feeder location is selected at compile time for upper-feed (gimbal board) or
lower-feed (chassis board) robots.

## Behaviour

- Dial enables friction wheels and requests manual shots.
- Automatic fire requires automatic mode and a stable visual lock.
- A rising fire request queues one shot.
- `module_shooter` performs jam confirmation, rollback, retry and fault latching.

## Outputs

`app_shooter_feedback_t` reports state, retry count, friction-ready state and fire permission. It
is mirrored over board communication so either controller can observe the firing mechanism.

## Validation

- Disabling friction cancels actuator output safely.
- Holding a manual request does not enqueue a shot every task cycle.
- Low feeder speed plus high current triggers rollback.
- Retry exhaustion latches a shooter fault.
