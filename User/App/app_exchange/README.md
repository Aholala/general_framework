# App exchange

## Why it exists

`app_exchange` is the typed communication boundary between independent App components. It avoids
including one application inside another and avoids scattered mutable global variables.

## Data channels

- Command to chassis, gimbal and shooter.
- IMU attitude snapshot.
- Chassis, gimbal and shooter feedback.
- Vision target.

Each publish/read operation copies one complete structure inside a short FreeRTOS critical
section. Storage is static and no dynamic memory is used.

## Placement

This module belongs directly under `User/App`: it is application infrastructure, not an RTOS task
and not a hardware protocol. CAN/USB packing remains in Module or the vision adapter.

## Validation

- Readers see either the previous or complete new snapshot, never a partially copied structure.
- `app_exchange_init` clears every channel before tasks start.
- Structures contain explicit units and validity fields.
