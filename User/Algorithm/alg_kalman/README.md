# 通用卡尔曼滤波库 (alg_kalman) —— 完整使用指南

## 1. 模块概述

`alg_kalman` 是 Algorithm 层独立的卡尔曼滤波库，提供三种卡尔曼滤波算法：标量卡尔曼、线性卡尔曼和扩展卡尔曼（EKF）。所有算法均使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有矩阵由调用者提供。

**核心功能**：

- 标量卡尔曼滤波（单变量）
- 任意维线性卡尔曼滤波
- 扩展卡尔曼滤波（EKF，非线性模型）

**设计哲学**：

- **零动态内存**：所有内存由调用者静态分配
- **纯标准库依赖**：只依赖 `<stdbool.h>`、`<stddef.h>`、`<math.h>`
- **多实例支持**：每个对象独立状态，可创建任意多个实例
- **时变模型支持**：可在运行中修改矩阵，无需重新初始化
- **数值稳定**：Joseph 形式协方差更新 + 部分主元 Gauss-Jordan 求逆

## 2. 矩阵存储规则

所有矩阵使用**行优先连续存储**：

```text
A(row, column) = A[row * column_count + column]
```

## 3. 三种滤波器对比

| 特性     | 标量卡尔曼 | 线性卡尔曼   | 扩展卡尔曼 |
| :------- | :--------- | :----------- | :--------- |
| 状态维度 | 1          | 任意 n       | 任意 n     |
| 测量维度 | 1          | 任意 m       | 任意 m     |
| 控制输入 | 状态增量   | 任意 c       | 任意 c     |
| 模型类型 | 线性       | 线性         | 非线性     |
| 模型定义 | 内置       | 矩阵 F, H, B | 回调函数   |
| 适用场景 | 单变量估计 | 线性系统     | 非线性系统 |

## 4. 使用示例

### 4.1 标量卡尔曼（温度估计）

```c
static alg_kalman_scalar_t s_temp_filter;

void init_temp_filter(void) {
    alg_kalman_scalar_init(&s_temp_filter, 0.01F, 0.5F, 25.0F, 1.0F);
}

float filter_temperature(float measurement) {
    float filtered;
    alg_kalman_scalar_update(&s_temp_filter, measurement, &filtered);
    return filtered;
}
```

### 4.2 线性卡尔曼（恒加速度模型）

```c
#define STATE_DIM 2
#define MEAS_DIM 1
#define CONTROL_DIM 0

static float s_state[STATE_DIM];
static float s_cov[STATE_DIM * STATE_DIM];
static float s_F[STATE_DIM * STATE_DIM] = {{1.0F, 0.01F}, {0.0F, 1.0F}};
static float s_Q[STATE_DIM * STATE_DIM] = {{0.01F, 0.0F}, {0.0F, 0.01F}};
static float s_H[MEAS_DIM * STATE_DIM] = {{1.0F, 0.0F}};
static float s_R[MEAS_DIM * MEAS_DIM] = {{0.1F}};
static float s_workspace[ALG_KALMAN_WORKSPACE_SIZE(STATE_DIM, MEAS_DIM)];

alg_kalman_linear_t s_kf;

void init_kf(void) {
    alg_kalman_linear_config_t config = {
        .state_dimension = STATE_DIM,
        .measurement_dimension = MEAS_DIM,
        .control_dimension = 0,
        .state = s_state,
        .covariance = s_cov,
        .transition_matrix = s_F,
        .control_matrix = NULL,
        .process_noise = s_Q,
        .measurement_matrix = s_H,
        .measurement_noise = s_R,
        .workspace = s_workspace,
        .workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0]),
    };
    // 初始状态
    float init_state[STATE_DIM] = {0.0F, 1.0F};
    float init_cov[STATE_DIM * STATE_DIM] = {{1.0F, 0.0F}, {0.0F, 1.0F}};
    alg_kalman_linear_init(&s_kf, &config);
    alg_kalman_linear_reset(&s_kf, init_state, init_cov);
}

void predict_and_correct(float measurement) {
    alg_kalman_linear_predict(&s_kf, NULL);
    alg_kalman_linear_correct(&s_kf, &measurement);
    const float *state = alg_kalman_linear_get_state(&s_kf);
}
```

### 4.3 扩展卡尔曼（IMU 姿态）

```c
#define STATE_DIM 6
#define MEAS_DIM 3
#define CONTROL_DIM 3

// 状态转移函数
static alg_kalman_status_t state_function(const float *state, ...) {
    // 四元数积分...
    return ALG_KALMAN_STATUS_OK;
}

// 测量函数
static alg_kalman_status_t measurement_function(const float *state, ...) {
    // 重力方向预测...
    return ALG_KALMAN_STATUS_OK;
}

alg_kalman_extended_config_t config = {
    .state_dimension = STATE_DIM,
    .measurement_dimension = MEAS_DIM,
    .control_dimension = CONTROL_DIM,
    .state = s_state,
    .covariance = s_cov,
    .process_noise = s_Q,
    .measurement_noise = s_R,
    .workspace = s_workspace,
    .workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0]),
    .state_function = state_function,
    .state_jacobian_function = state_jacobian,
    .measurement_function = measurement_function,
    .measurement_jacobian_function = measurement_jacobian,
    .user_context = &imu_data,
};

alg_kalman_extended_init(&s_ekf, &config);
alg_kalman_extended_predict(&s_ekf, gyro_input, 0.01F);
alg_kalman_extended_correct(&s_ekf, accel_input);
```

## 5. 工作区大小计算

使用 `ALG_KALMAN_WORKSPACE_SIZE` 宏计算所需浮点元素数：

```c
static float s_workspace[
    ALG_KALMAN_WORKSPACE_SIZE(STATE_DIMENSION, MEASUREMENT_DIMENSION)];

config.workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0]);
```

## 6. 数值稳定性

- **协方差更新**：使用 Joseph 形式 `P = (I-KH)P(I-KH)^T + KRK^T`，比直接形式更稳定
- **对称化**：每次更新后主动对称化协方差矩阵
- **矩阵求逆**：使用部分主元 Gauss-Jordan 消元法
- **检查机制**：所有输入检查有限数、对角线非负

## 7. 并发约束

- 同一滤波器对象不能多线程并发访问
- 推荐一个对象只由一个执行上下文持有
- 跨上下文数据通过消息队列或双缓冲传递

## 8. 建议验证测试项

- [ ] 标量卡尔曼收敛性和复位
- [ ] 二维恒加速度线性模型
- [ ] 多维测量和矩阵求逆
- [ ] 非线性平方观测 EKF
- [ ] Joseph 协方差更新后的对称性
- [ ] 空指针、未初始化对象和非法参数
- [ ] 工作区不足返回 `INSUFFICIENT_WORKSPACE`
- [ ] 奇异创新协方差返回 `SINGULAR_MATRIX`

---

## 一页式使用顺序与可读信息

1. 单变量信号选择 `alg_kalman_scalar_t`；固定矩阵模型选择 linear；非线性模型选择 extended。
2. 调用者静态准备状态、协方差、矩阵和工作区，生命周期必须覆盖滤波器对象。
3. 调用对应 `*_init()`，设置初始状态和协方差。
4. 每周期严格执行 `predict → correct`；标量简单场景可直接调用 `alg_kalman_scalar_update()`。
5. 通过 `*_get_state()` 和 `*_get_covariance()` 复制输出；传感器重置时调用 `*_reset()`。

| 可读取结构体            | 主要信息                                    |
| ----------------------- | ------------------------------------------- |
| `alg_kalman_scalar_t`   | 当前估计、协方差、过程噪声和测量噪声        |
| `alg_kalman_linear_t`   | 状态向量、协方差、模型矩阵和工作区引用      |
| `alg_kalman_extended_t` | 非线性状态、协方差、模型/雅可比回调和工作区 |

协方差不是“误差值”，而是不确定度；必须同时检查矩阵维度、有限值和函数返回状态。
