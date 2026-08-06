# app_shooter — 发射机构控制

双摩擦轮 + 拨弹盘。读取 `module_shooter` 状态 + 裁判系统火控信号。

## 用法

```c
app_shooter_config_t cfg = {
    .shooter = &shooter,
    .board_comm = &link,
};
app_shooter_init(&cfg);

// 周期更新
app_shooter_update(dt);
// 从 app_exchange 读取命令 → module_shooter_update_fire_control → 发布反馈
```

## 火控条件

射击需同时满足：
1. `tracking_ready` = 视觉目标有效 + 姿态误差在容差内
2. `referee_ok` = 裁判系统允许发射（热量+弹量）
3. `fire_requested` = 遥控器触发（拨轮/鼠标）
4. `friction_ready` = 摩擦轮到速
