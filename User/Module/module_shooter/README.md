# module_shooter — 发射机构

双摩擦轮 + 拨弹电机的状态机控制。`DISABLED → READY → FEEDING → ROLLBACK → FAULT`。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_shooter_t` | 发射机构对象 |
| `module_shooter_config_t` | 配置：摩擦轮电机、拨弹电机、射击参数 |
| `module_shooter_state_t` | 状态枚举：`DISABLED/READY/FEEDING/ROLLBACK/FAULT` |

## 读取状态

```c
module_shooter_state_t state = module_shooter_get_state(&shooter);
uint8_t pending = module_shooter_get_pending_shots(&shooter);  // 待发弹量
uint8_t jams    = module_shooter_get_jam_retry_count(&shooter); // 卡弹次数
bool friction_ok = module_shooter_get_friction_ready(&shooter);
bool can_fire    = module_shooter_get_fire_permission(&shooter);
```

## 用法

```c
module_shooter_t shooter;
module_shooter_config_t cfg = {
    .friction_motor = &friction_m3508,
    .feeder_motor   = &feeder_m2006,
    .friction_velocity_rad_per_s = 500,
    .feed_angle_rad = 0.628f,     // 每发步进角度
    .feed_timeout_ms = 500,
    .jam_current_threshold_a = 2.0f,
};
module_shooter_init(&shooter, &cfg);

// 设置摩擦轮速度 + 拨弹盘位置
module_shooter_set_friction_velocity(&shooter, 500);
module_shooter_set_feed_position(&shooter, 0.628f);

// 火控更新（由 App 周期性调用）
module_shooter_update_fire_control(&shooter, tracking_ready, referee_ok, fire_cmd);

// 周期更新状态机
module_shooter_update(&shooter, dt);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_shooter_init(me, cfg)` | 初始化 |
| `module_shooter_set_friction_velocity(me, rad_per_s)` | 设摩擦轮目标速度 |
| `module_shooter_set_feed_position(me, rad)` | 设拨弹步进角度 |
| `module_shooter_update_fire_control(me, tracking, ref_ok, fire)` | 火控决策 |
| `module_shooter_update(me, dt)` | 周期更新状态机 |
| `module_shooter_get_state(me)` | 读状态 |
| `module_shooter_get_pending_shots(me)` | 待发弹量 |
| `module_shooter_get_jam_retry_count(me)` | 卡弹次数 |
| `module_shooter_get_friction_ready(me)` | 摩擦轮到速？ |
| `module_shooter_get_fire_permission(me)` | 火控许可？ |
