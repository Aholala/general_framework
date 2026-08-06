# task_shooter

发射机构调度适配器。按照 `APP_SHOOTER_PERIOD_MS` 调用 `app_shooter_update()`；摩擦轮、
拨弹、堵转确认和回退状态机均位于 App/Module，不放在任务循环中。

