# task_chassis

底盘 FreeRTOS 调度适配器。按照 `APP_CHASSIS_PERIOD_MS` 周期调用
`app_chassis_update()`，不保存底盘模式、运动学参数或电机对象。

