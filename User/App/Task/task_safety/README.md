# task_safety

安全监控调度适配器。周期调用 `app_safety_process()`，检查已注册设备的心跳和离线
状态；故障策略和恢复动作由 `app_safety` 管理。

