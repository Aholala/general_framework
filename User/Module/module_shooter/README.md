# module_shooter

RoboMaster 发射机构状态机，组合左右摩擦轮和拨弹电机，提供摩擦轮启停、排队发射、位置
步进、堵转确认、自动回退、有限重试和故障锁存。

## 状态机

```text
DISABLED
  -> READY
  -> FEEDING
  -> ROLLBACK
  -> FEEDING / FAULT
```

- `DISABLED`：所有目标安全归零；
- `READY`：摩擦轮可运行，等待射击；
- `FEEDING`：拨弹盘向下一发目标位置运动；
- `ROLLBACK`：检测堵转后反向退让；
- `FAULT`：超过最大重试或电机异常，锁存停机。

## 电机依赖

配置接收三个 `module_motor_t *`，因此摩擦轮和拨弹盘可以使用 M3508 或其他实现同一基类
的电机。方向符号、拨弹步距和阈值均显式配置。

## 使用流程

```c
module_shooter_init(&shooter, &config);
module_shooter_enable(&shooter);
module_shooter_set_friction(&shooter, true, friction_speed_rad_per_s);
module_shooter_request_shots(&shooter, 1U);

module_shooter_update(&shooter, delta_time_s);
```

更新函数只设置电机目标并推进状态；底层电机仍需由统一电机调度器 update/flush。

## 堵转判断

拨弹电机速度低于阈值，同时电流超过 A 阈值或原始电流阈值，并持续
`jam_confirmation_time_s` 后确认堵转。随后目标回退 `rollback_angle_rad`，到达容差后
重试原射击目标。

电流缩放未知时可使用原始阈值；已校准电机优先使用安培值。阈值需要实车标定。

## 队列与故障

`request_shots` 对待发数量做上限检查，防止遥控抖动导致无界累计。`cancel_shots` 清空
请求但不隐式关闭摩擦轮。

故障必须调用 `reset_fault` 明确恢复，且三个电机应在线、无故障。不能在周期任务自动清除。

## 设计边界

热量限制、裁判系统弹速限制、射频控制和摩擦轮到速判定策略属于 App。Module 不决定拨弹盘
安装在哪块板。

## 建议验证

- 摩擦轮启停方向；
- 单发、多发和队列上限；
- 正常到位；
- 瞬时大电流不误判；
- 持续堵转、回退和成功重试；
- 超过重试进入 FAULT；
- 任一电机离线和显式恢复。
