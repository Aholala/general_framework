# app_chassis — 底盘控制

舵轮底盘运动控制。支持跟随/旋转/无力三种模式。

## 用法

```c
app_chassis_config_t cfg = {
    .kinematics = &swerve,
    .modules[0] = &swerve_fl, .modules[1] = &swerve_fr,
    .modules[2] = &swerve_rl, .modules[3] = &swerve_rr,
};
app_chassis_init(&cfg);

// 周期更新
app_chassis_update(dt);
// 从 app_exchange 读取命令 → 运动学逆解 → 控制舵轮
```

## 底盘模式

| 模式 | 说明 |
|------|------|
| `NORMAL` | 正常遥控（X/Y/W 三自由度） |
| `SPIN` | 小陀螺（绕 Z 轴恒速旋转） |
| `FOLLOW_GIMBAL` | 底盘跟随云台指向 |
| `NO_FORCE` | 无力模式（所有电机断电） |
