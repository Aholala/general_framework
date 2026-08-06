# alg_lqr — LQR 线性二次型调节器

纯算法。DARE（离散代数 Riccati 方程）求解 + 有限时域 LQR。

## 用法

```c
alg_lqr_controller_t lqr;
alg_lqr_dare_config_t cfg = {
    .state_dim = 4, .input_dim = 2,
};
alg_lqr_init(&lqr, &cfg);

// 设系统矩阵
alg_lqr_set_matrices(&lqr, A, B, Q, R);

// 求解 DARE → 得到增益 K
alg_lqr_solve_dare(&lqr);

// 计算控制量
float x[4] = {...};  // 状态
float u[2];          // 输出
alg_lqr_compute(&lqr, x, u);
```

## 角度 LQR

```c
alg_lqr_angle_t alqr;
// 自动处理 -π/+π 环绕
alg_lqr_angle_update(&alqr, angle_sp, angle_mv, vel_mv, dt, &output);
```
