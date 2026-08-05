# Task 调度层

本目录只保存 FreeRTOS 调度适配器。每个任务使用独立子目录，避免任务增多后源文件混在一起。

```text
Task/
├── task_chassis/     底盘周期任务
├── task_command/     命令与通信周期任务
├── task_gimbal/      云台周期任务
├── task_imu/         IMU 高频采样任务
├── task_safety/      安全监控任务
└── task_shooter/     发射机构周期任务
```

每个子目录包含：

- `task_<功能>.c`：永久循环、周期控制和 App 更新调用。
- `task_<功能>.h`：提供给 `freertos.c` 的唯一任务入口。
- `README.md`：说明任务周期、调用关系和职责边界。

Task 只能依赖 App，不允许 App 反向包含 `task_*.h`：

```text
CubeMX StartXxxTask() -> task_xxx_run() -> app_xxx_update()
```

周期任务统一使用 `osDelayUntil()`，避免执行时间累积造成周期漂移。优先级和栈空间由
CubeMX/`freertos.c` 管理，业务状态机和控制算法放在对应 `app_<功能>` 目录。

| CubeMX 任务 | 调度入口 | 默认周期 |
|---|---|---:|
| ChassisTask | `task_chassis_run` | 5 ms |
| CommandTask | `task_command_run` | 5 ms |
| GimbalTask | `task_gimbal_run` | 2 ms |
| IMUTask | `task_imu_run` | 1 ms |
| SafeTask | `task_safety_run` | 5 ms |
| ShootTask | `task_shooter_run` | 5 ms |

