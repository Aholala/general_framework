# alg_omni — 全向轮底盘

任意数量全向轮的逆解/正解/里程计。和 `alg_mecanum` 类似但通用性更强。

## 用法

```c
alg_omni_t omni;
alg_omni_wheel_config_t wheels[] = {
    { .x_m = 0.15f, .y_m = 0.15f, .direction_rad = 0.785f },  // 45°
    { .x_m = 0.15f, .y_m = -0.15f, .direction_rad = -0.785f }, // -45°
    { .x_m = -0.15f, .y_m = 0.15f, .direction_rad = -0.785f },
    { .x_m = -0.15f, .y_m = -0.15f, .direction_rad = 0.785f },
};
alg_omni_config_t cfg = { .wheel_count = 4, .wheels = wheels, .wheel_radius_m = 0.05f };
alg_omni_init(&omni, &cfg);

// 逆解
float wheel_speeds[4];
alg_omni_inverse(&omni, &cmd, wheel_speeds);

// 正解 + 里程计
alg_omni_forward(&omni, wheel_speeds);
alg_omni_update_odometry(&omni, wheel_speeds, dt);
```

和 `alg_mecanum` 的区别：全向轮方向角是配置参数，麦克纳姆轮固定 45°。
