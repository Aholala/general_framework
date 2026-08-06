# alg_mecanum — 麦克纳姆轮底盘

四轮麦克纳姆逆解/正解/里程计。输入 `alg_chassis_velocity_t`，输出四轮速度。

## 用法

```c
alg_mecanum_t mecanum;
alg_mecanum_config_t cfg = {
    .wheel_radius_m = 0.076f,
    .wheel_base_x_m = 0.3f,   // 前后轴距一半
    .wheel_base_y_m = 0.3f,   // 左右轴距一半
};
alg_mecanum_init(&mecanum, &cfg);

// 逆解
alg_chassis_velocity_t cmd = {1.0f, 0, 0};
float wheel_speeds[4];  // 左前/右前/左后/右后 (rad/s)
alg_mecanum_inverse(&mecanum, &cmd, wheel_speeds);

// 正解 + 里程计
alg_mecanum_forward(&mecanum, wheel_speeds);
alg_mecanum_update_odometry(&mecanum, wheel_speeds, 0.001f);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `alg_mecanum_init(me, cfg)` | 初始化 |
| `alg_mecanum_inverse(me, cmd, wheels[4])` | 逆解 |
| `alg_mecanum_forward(me, wheels[4])` | 正解 |
| `alg_mecanum_update_odometry(me, wheels, dt)` | 里程计 |
| `alg_mecanum_get_pose(me)` | 读位姿 |
| `alg_mecanum_get_solution(me)` | 读解算诊断 |

## 坐标系

```
    前(+x)
  ┌────────┐
  │ FL  FR │  左前(FL) 右前(FR)
  │        │
  │ RL  RR │  左后(RL) 右后(RR)
  └────────┘
   左(+y)
```
