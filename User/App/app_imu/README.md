# IMU application

## Responsibility

`app_imu` samples BMI088 and publishes the attitude used by IMU-lock gimbal control and visual
telemetry. The current template integrates gyroscope rates and corrects pitch/roll with
accelerometer inclination.

## Output

`app_imu_snapshot_t` contains yaw, pitch, roll, three angular velocities, sample count and a valid
flag. Yaw has no magnetometer correction and therefore may drift; this is acceptable for the
framework template and can later be replaced by `alg_attitude` or `alg_imu_ekf`.

## Scheduling

`task_imu` calls `app_imu_update` every 1 ms.

## Validation

- A failed BMI088 read publishes `valid = false`.
- Sample count increases after successful reads.
- Pitch and roll converge while the board remains stationary.
- No motor control is performed in this application.
