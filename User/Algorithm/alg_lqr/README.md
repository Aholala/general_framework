# alg_lqr —— 完整使用指南

`alg_lqr` 是 Algorithm 层独立的线性二次型调节器（LQR）算法库。提供无限时域（DARE）和有限时域 LQR 求解、实时控制器执行、连续模型离散化及 LQI 积分增广功能。所有算法均使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有矩阵由调用者提供。

## 1. 模块概述

**核心功能**：

- 多状态、多输入 LQR 控制器（状态反馈 + 前馈 + 平衡控制）
- 无限时域离散代数 Riccati 方程（DARE）迭代求解
- 有限时域反向 Riccati 递推（返回完整增益序列）
- 状态-控制交叉权重（`cross_weight`）
- 连续模型 Tustin 双线性离散化
- LQI（线性二次积分）增广模型构建
- 每个控制通道独立限幅

**设计哲学**：

- **零动态内存**：所有内存（状态、增益、工作区）由调用者静态分配
- **纯标准库依赖**：只依赖 `<stdbool.h>`、`<stddef.h>`、`<math.h>`
- **多实例支持**：可创建任意多个控制器或求解器实例
- **时变模型支持**：可在运行中修改矩阵指针指向的新数据，无需重新初始化
- **数值鲁棒**：部分主元 Gauss-Jordan 求逆、迭代对称化、严格有限性检查

---

## 2. 矩阵存储规则

所有矩阵使用**行优先连续存储**：

```text
A(row, column) = A[row * column_count + column]
```

---

## 3. 功能对比与适用场景

| 功能模块          | 接口 / 类型                           | 适用场景                                         |
| :---------------- | :------------------------------------ | :----------------------------------------------- |
| **LQR 控制器**    | `alg_lqr_controller_t`                | 实时控制环，执行 `u = u_eq + u_ff - K*(x-x_ref)` |
| **角度 LQR**      | `alg_lqr_angle_t`                     | 二状态角度/角速度控制的专用封装                  |
| **无限时域 LQR**  | `alg_lqr_dare_solve()`                | 稳态最优控制，模型固定，需离线或低频率在线求解   |
| **有限时域 LQR**  | `alg_lqr_finite_solve()`              | 短时域最优轨迹、起停过程、终端约束问题           |
| **Tustin 离散化** | `alg_lqr_discretize_tustin()`         | 将连续时间模型转换为离散时间模型                 |
| **LQI 积分增广**  | `alg_lqr_lqi_build_augmented_model()` | 消除稳态误差，构建增广状态用于积分控制           |

---

## 4. 离散系统模型与代价函数

库使用离散线性时不变模型：

```text
x(k+1) = A x(k) + B u(k)
```

无限时域代价函数：

```text
J = Σ_{k=0}^{∞} [ x(k)ᵀ Q x(k) + 2 x(k)ᵀ N u(k) + u(k)ᵀ R u(k) ]
```

**矩阵维度说明**：

| 矩阵 | 维度    | 描述                              |
| :--- | :------ | :-------------------------------- |
| A    | `n × n` | 状态转移矩阵                      |
| B    | `n × m` | 控制输入矩阵                      |
| Q    | `n × n` | 状态代价权重（对称半正定）        |
| R    | `m × m` | 控制代价权重（对称正定）          |
| N    | `n × m` | 交叉权重（可为 NULL，表示零矩阵） |
| P    | `n × n` | Riccati 解矩阵                    |
| K    | `m × n` | 反馈增益矩阵                      |

`cross_weight` 为 `NULL` 时等价于 N = 0。

---

## 5. 使用示例

### 5.1 无限时域 LQR（DARE 求解）

对于固定模型，建议在开发阶段离线求解增益并作为常量烧录；若必须在运行时求解，应在低优先级任务中执行。

```c
#include "alg_lqr.h"

enum { STATE_DIM = 2, CTRL_DIM = 1 };

// 静态内存分配
static float s_workspace[ALG_LQR_RICCATI_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];
static float s_riccati[STATE_DIM * STATE_DIM];
static float s_gain[CTRL_DIM * STATE_DIM];

// 假设已定义 A, B, Q, R 矩阵数据（此处仅为示意）
static float A[STATE_DIM * STATE_DIM] = {1.0f, 0.01f, 0.0f, 1.0f};
static float B[STATE_DIM * CTRL_DIM] = {0.0f, 0.01f};
static float Q[STATE_DIM * STATE_DIM] = {1.0f, 0.0f, 0.0f, 1.0f};
static float R[CTRL_DIM * CTRL_DIM] = {0.1f};

void solve_gain(void) {
    alg_lqr_dare_config_t config = {
        .state_dimension = STATE_DIM,
        .control_dimension = CTRL_DIM,
        .state_matrix = A,
        .control_matrix = B,
        .state_weight = Q,
        .control_weight = R,
        .cross_weight = NULL,
        .tolerance = 1.0e-6f,
        .maximum_iterations = 1000U,
        .workspace = s_workspace,
        .workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0])
    };

    size_t iter;
    alg_lqr_status_t status = alg_lqr_dare_solve(&config, s_riccati, s_gain, &iter);
    if (status == ALG_LQR_STATUS_OK) {
        // 使用 s_gain 作为控制器增益
    }
}
```

### 5.2 有限时域 LQR

适用于固定时长轨迹跟踪或起停控制。

```c
enum { STATE_DIM = 2, CTRL_DIM = 1, HORIZON = 50 };

static float s_gain_sequence[HORIZON * CTRL_DIM * STATE_DIM];
static float s_workspace[ALG_LQR_FINITE_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];
static float s_initial_P[STATE_DIM * STATE_DIM];

void solve_finite_horizon(void) {
    // 假设已定义 A, B, Q, R, P_f（终端权重）
    alg_lqr_finite_config_t config = {
        .state_dimension = STATE_DIM,
        .control_dimension = CTRL_DIM,
        .horizon_length = HORIZON,
        .state_matrix = A,
        .control_matrix = B,
        .state_weight = Q,
        .control_weight = R,
        .cross_weight = NULL,
        .terminal_state_weight = P_f,   // 终端代价权重
        .workspace = s_workspace,
        .workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0])
    };

    alg_lqr_finite_solve(&config, s_gain_sequence, s_initial_P);
    // 使用时：第 k 步增益 = &s_gain_sequence[k * CTRL_DIM * STATE_DIM]
}
```

### 5.3 LQR 控制器实时运行

控制器在控制周期内执行，仅包含矩阵-向量乘法，适合高频率中断。

```c
// 假设已求解得到增益矩阵 s_gain（m×n）
static float s_control_output[CTRL_DIM];
static float s_state[STATE_DIM];
static float s_reference[STATE_DIM];
static float s_equilibrium[CTRL_DIM];
static float s_ff[CTRL_DIM];

// 限幅值（若不需限幅，设置为 NULL）
static float s_ctrl_min[CTRL_DIM] = {-1.0f};
static float s_ctrl_max[CTRL_DIM] = { 1.0f};

alg_lqr_controller_config_t ctrl_config = {
    .state_dimension = STATE_DIM,
    .control_dimension = CTRL_DIM,
    .gain_matrix = s_gain,
    .control_min = s_ctrl_min,
    .control_max = s_ctrl_max
};

alg_lqr_controller_t controller;
alg_lqr_controller_init(&controller, &ctrl_config);

// 在控制中断中调用
void control_loop(void) {
    // 获取当前状态 s_state，参考 s_reference，平衡点 s_equilibrium，前馈 s_ff
    alg_lqr_controller_update(&controller,
                              s_state,
                              s_reference,
                              s_equilibrium,
                              s_ff,
                              s_control_output);
    // 输出 s_control_output 到执行器
}
```

#### 5.3.1 二维角度 LQR 封装

角度与角速度二状态控制直接使用 `alg_lqr` 内的专用封装：

```c
static const float angle_gain[2] = {12.0F, 0.8F};
alg_lqr_angle_config_t angle_config = {
    .gain_matrix = angle_gain,
    .control_min = -10.0F,
    .control_max = 10.0F,
    .equilibrium_control = 0.0F,
};
alg_lqr_angle_t angle_controller;
alg_lqr_angle_init(&angle_controller, &angle_config);
alg_lqr_angle_reset(&angle_controller, current_angle_rad,
                    current_velocity_rad_per_s, current_output);

alg_lqr_angle_input_t angle_input = {
    .target_position_rad = target_angle_rad,
    .target_velocity_rad_per_s = target_velocity_rad_per_s,
    .measured_position_rad = current_angle_rad,
    .measured_velocity_rad_per_s = current_velocity_rad_per_s,
    .actuator_feedforward = feedforward,
    .delta_time_s = 0.001F,
};
alg_lqr_angle_update(&angle_controller, &angle_input, &output);
```

`reset` 用于切换控制器前检查当前状态。LQR 没有积分状态，因此不会修改内部增益。

### 5.4 连续模型离散化（Tustin）

```c
enum { STATE_DIM = 2, CTRL_DIM = 1 };

static float Ac[STATE_DIM * STATE_DIM] = {0.0f, 1.0f, 0.0f, 0.0f};
static float Bc[STATE_DIM * CTRL_DIM] = {0.0f, 1.0f};
static float Ad[STATE_DIM * STATE_DIM];
static float Bd[STATE_DIM * CTRL_DIM];
static float workspace[ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];

void discretize(void) {
    alg_lqr_discretize_tustin(Ac, Bc, STATE_DIM, CTRL_DIM, 0.01f,
                              Ad, Bd, workspace,
                              sizeof(workspace)/sizeof(workspace[0]));
    // 使用 Ad, Bd 进行 LQR 设计
}
```

### 5.5 LQI 积分增广

构建增广状态 `[x; e]`，其中 `e` 为积分误差，用于消除稳态偏差。

```c
enum { STATE_DIM = 2, CTRL_DIM = 1, INTEGRAL_DIM = 1 };

static float A[STATE_DIM * STATE_DIM];
static float B[STATE_DIM * CTRL_DIM];
static float C[INTEGRAL_DIM * STATE_DIM];  // 输出矩阵，选择要积分的状态
static float A_aug[(STATE_DIM+INTEGRAL_DIM)*(STATE_DIM+INTEGRAL_DIM)];
static float B_aug[(STATE_DIM+INTEGRAL_DIM)*CTRL_DIM];

void build_augmented(void) {
    alg_lqr_lqi_build_augmented_model(A, B, C,
                                      STATE_DIM, CTRL_DIM, INTEGRAL_DIM,
                                      0.01f, A_aug, B_aug);
    // 然后为增广系统设计 LQR 增益
}
```

---

## 6. 工作区大小计算

使用预定义宏计算所需浮点元素数，确保工作区足够：

```c
// Riccati 迭代所需
static float s_workspace_dare[
    ALG_LQR_RICCATI_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];

// 有限时域所需
static float s_workspace_finite[
    ALG_LQR_FINITE_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];

// Tustin 离散化所需
static float s_workspace_tustin[
    ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(STATE_DIM, CTRL_DIM)];
```

工作区大小的单位是 `float` 元素数量，**不是字节**。

---

## 7. 数值稳定性

- **矩阵求逆**：使用部分主元 Gauss-Jordan 消元，检测奇异并返回 `SINGULAR_MATRIX`。
- **对称化**：每次 Riccati 迭代后强制 `P` 对称，消除浮点误差累积。
- **收敛判据**：使用最大元素变化量（`max(|P_{k+1}-P_k|)`）判断收敛。
- **输入校验**：所有输入指针、矩阵元素进行有限性检查；Q、R、P_f 检查对角线非负。
- **奇异阈值**：`1e-12`，低于此值认为奇异。

调用者仍需保证：

- `(A, B)` 可稳定（可控）
- `Q` 为对称半正定，`R` 为对称正定
- 状态量已归一化或合理缩放（条件数影响收敛速度）

---

## 8. 实时性建议

- **控制器**（`alg_lqr_controller_update`）：仅包含矩阵-向量乘法和限幅，适合高优先级中断或周期任务。
- **Riccati 求解**（`alg_lqr_dare_solve` / `alg_lqr_finite_solve`）：涉及矩阵乘法、求逆和多轮迭代，**不建议**在实时中断中运行。推荐：
  - 固定模型 → 离线求解，增益作为常量。
  - 少量模型切换 → 预存多组增益。
  - 必须在线求解 → 在低优先级任务或初始化阶段完成。

---

## 9. 并发约束

- 同一 `alg_lqr_controller_t` 或求解器配置对象**不能**被多个执行上下文同时访问。
- 推荐每个控制器对象只由一个线程/中断持有。
- 若需跨任务传递数据（如增益更新），使用消息队列或双缓冲机制。

---

## 10. 建议验证测试项

- [ ] 标量系统 DARE 与解析解对比（收敛性）
- [ ] 二维双积分系统闭环稳定性与收敛时间
- [ ] 多状态、多输入系统参考跟踪与扰动抑制
- [ ] 状态参考、平衡控制、前馈组合的正确性
- [ ] 控制限幅（饱和）功能测试
- [ ] 有限时域反向递推与终端代价一致性
- [ ] 交叉权重 `N` 非零时的增益计算
- [ ] Tustin 离散化与解析解（已知传递函数）对比
- [ ] LQI 增广模型对常值扰动消除效果
- [ ] 奇异矩阵、不收敛、工作区不足等错误路径
- [ ] 工作区大小宏与实际需求匹配性检查

---

**总结**：`alg_lqr` 为嵌入式系统提供了一套完整的 LQR 最优控制解决方案，覆盖从模型离散化、增益计算到实时控制的全链路，且保持零动态内存、纯 C11 的可移植性。配合 BSP/Module 层的执行器与传感器接口，可快速实现高可靠性的最优控制系统。
