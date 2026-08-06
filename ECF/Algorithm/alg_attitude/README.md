# alg_attitude — 姿态估计（Mahony/Madgwick）

纯算法。从陀螺+加速度计估计四元数/Euler角。支持 Mahony（显式互补）和 Madgwick（梯度下降）两种算法。

## 用法

```c
alg_attitude_t att;
alg_attitude_config_t cfg = {
    .algorithm = ALG_ATTITUDE_MAHONY,
    .kp = 0.5f,   // 互补滤波比例增益（Mahony）
    .ki = 0.0f,   // 积分增益（通常设 0）
};
alg_attitude_init(&att, &cfg);

// 周期更新
alg_attitude_update(&att,
    gx, gy, gz,     // 陀螺 (rad/s)
    ax, ay, az,     // 加速度计 (m/s²)
    0.001f);        // dt (s)

// 读取
float roll, pitch, yaw;
alg_attitude_get_euler(&att, &roll, &pitch, &yaw);

alg_attitude_quaternion_t q;
alg_attitude_get_quaternion(&att, &q);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `alg_attitude_init(me, cfg)` | 初始化 |
| `alg_attitude_update(me, gx,gy,gz, ax,ay,az, dt)` | 更新 |
| `alg_attitude_get_euler(me, &r, &p, &y)` | 读欧拉角 (rad) |
| `alg_attitude_get_quaternion(me, &q)` | 读四元数 |
| `alg_attitude_get_rotation_matrix(me, &R)` | 读旋转矩阵 |
