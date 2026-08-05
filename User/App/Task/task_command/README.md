# task_command

命令与通信 FreeRTOS 调度适配器。按照 `APP_COMMAND_PERIOD_MS` 处理 DR16、CAN2
板间通信、视觉数据和命令仲裁，具体映射逻辑由 `app_command` 负责。

