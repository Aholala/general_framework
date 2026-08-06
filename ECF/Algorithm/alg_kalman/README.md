# alg_kalman — 卡尔曼滤波器库

纯算法。标量 KF、线性 KF、扩展 KF (EKF)。

## 包含的滤波器

| 类型 | 结构体 | 用途 |
|------|--------|------|
| 标量 KF | `alg_kalman_scalar_t` | 单变量 |
| 线性 KF | `alg_kalman_linear_t` | 矩阵形式 |
| 扩展 KF | `alg_kalman_extended_t` | 非线性系统（用户提供 f/h 函数） |

## 线性 KF 用法

```c
alg_kalman_linear_t kf;
alg_kalman_linear_init(&kf, state_dim, meas_dim);

// 设置矩阵（F, H, Q, R, P0）
alg_kalman_linear_set_F(&kf, F);
alg_kalman_linear_set_H(&kf, H);
// ... Q, R, P

// 预测
alg_kalman_linear_predict(&kf);

// 更新
float z[2] = {...};
alg_kalman_linear_update(&kf, z);

// 读取状态
const float *x = alg_kalman_linear_get_state(&kf);
```

## EKF 用法

```c
alg_kalman_extended_t ekf;
// 用户提供 f(x,u) 状态转移函数和 h(x) 观测函数
alg_kalman_extended_set_state_transition(&ekf, my_f, my_F_jacobian);
alg_kalman_extended_set_observation(&ekf, my_h, my_H_jacobian);
alg_kalman_extended_predict(&ekf, u);
alg_kalman_extended_update(&ekf, z);
```
