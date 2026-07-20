# alg_lqr

`alg_lqr` 是 Algorithm 层的线性二次型调节器算法库。源码和头文件直接放在模块目录中，文件、类型、函数和枚举分别使用 `alg_lqr`、`AlgLqr` 和 `ALG_LQR` 前缀。

## 功能范围

| 功能 | 接口 |
|---|---|
| 多状态、多输入 LQR 控制器 | `AlgLqrController_t` |
| 无限时域离散 LQR | `AlgLqrDare_Solve()` |
| 有限时域离散 LQR | `AlgLqrFinite_Solve()` |
| 状态—控制交叉权重 N | `cross_weight` |
| 状态参考跟踪 | `reference_state` |
| 平衡控制输入 | `equilibrium_control` |
| 外部前馈 | `feedforward_control` |
| 每个控制通道独立限幅 | `control_min/control_max` |
| 连续模型 Tustin 离散化 | `AlgLqrDiscretize_Tustin()` |
| LQI 积分状态增广 | `AlgLqrLqi_BuildAugmentedModel()` |
| 奇异矩阵检测 | `ALG_LQR_STATUS_SINGULAR_MATRIX` |
| DARE 收敛检测 | `ALG_LQR_STATUS_NOT_CONVERGED` |

所有矩阵维度在运行时指定，没有写死状态数或控制输入数。

## 可移植性

- 纯 C11。
- 不依赖 HAL、CMSIS 或 RTOS。
- 不调用系统时钟。
- 不使用动态内存。
- 不使用可变全局状态。
- 矩阵、结果和工作区全部由调用者提供。
- 支持裸机、定时中断和 RTOS 任务。
- 只依赖 `<stdbool.h>`、`<stddef.h>` 和 `<math.h>`。

## 离散系统模型

库使用以下模型：

```text
x(k+1) = A x(k) + B u(k)
```

代价函数为：

```text
J = Σ[xᵀQx + 2xᵀNu + uᵀRu]
```

矩阵尺寸：

| 矩阵 | 尺寸 |
|---|---|
| A | `n × n` |
| B | `n × m` |
| Q | `n × n` |
| R | `m × m` |
| N | `n × m` |
| P | `n × n` |
| K | `m × n` |

其中 `n` 是状态维数，`m` 是控制输入维数。所有矩阵均采用行优先连续存储：

```text
matrix(row, column) = matrix[row * column_count + column]
```

`cross_weight` 可以为 `NULL`，表示 N 为零矩阵。

## 无限时域 LQR

`AlgLqrDare_Solve()` 迭代求解离散代数 Riccati 方程：

```text
K = (R + BᵀPB)⁻¹(BᵀPA + Nᵀ)

P = Q + AᵀPA - (AᵀPB + N)K
```

示例：

```c
enum
{
    STATE_DIMENSION = 2,
    CONTROL_DIMENSION = 1
};

static float s_workspace[
    ALG_LQR_RICCATI_WORKSPACE_SIZE(STATE_DIMENSION, CONTROL_DIMENSION)];
static float s_riccati[STATE_DIMENSION * STATE_DIMENSION];
static float s_gain[CONTROL_DIMENSION * STATE_DIMENSION];

AlgLqrDareConfig_t config = {
    .state_dimension = STATE_DIMENSION,
    .control_dimension = CONTROL_DIMENSION,
    .state_matrix = state_matrix,
    .control_matrix = control_matrix,
    .state_weight = state_weight,
    .control_weight = control_weight,
    .cross_weight = NULL,
    .tolerance = 1.0e-5F,
    .maximum_iterations = 1000U,
    .workspace = s_workspace,
    .workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0])
};

AlgLqrStatus_t status = AlgLqrDare_Solve(&config,
                                         s_riccati,
                                         s_gain,
                                         NULL);
```

工作区大小的单位是 `float` 元素数量，不是字节。

对于固定模型，建议在开发工具中离线求解 K，将增益作为只读常量放入固件。运行时求解适合模型会切换、参数需要现场配置或无法提前确定的系统。

## 有限时域 LQR

`AlgLqrFinite_Solve()` 从终端权重反向递推，返回完整增益序列：

```text
gain_sequence[0]            第 0 个控制周期
gain_sequence[1]            第 1 个控制周期
...
gain_sequence[horizon - 1]  最后一个控制周期
```

每个增益占用 `control_dimension * state_dimension` 个元素。

有限时域求解适合：

- 固定长度轨迹。
- 起停过程。
- 状态权重终端强化。
- 简单预测控制问题。

## 控制器执行

控制器计算：

```text
u = u_equilibrium + u_feedforward - K(x - x_reference)
```

示例：

```c
AlgLqrControllerConfig_t controller_config = {
    .state_dimension = STATE_DIMENSION,
    .control_dimension = CONTROL_DIMENSION,
    .gain_matrix = s_gain,
    .control_min = control_min,
    .control_max = control_max
};

AlgLqrController_t controller;
AlgLqrController_Init(&controller, &controller_config);

AlgLqrController_Update(&controller,
                        state,
                        reference_state,
                        equilibrium_control,
                        feedforward_control,
                        output);
```

不需要的参考、平衡输入或前馈可以传入 `NULL`，表示零向量。

如果不需要限幅，`control_min` 和 `control_max` 必须同时为 `NULL`；需要限幅时必须同时提供。

## 连续模型离散化

`AlgLqrDiscretize_Tustin()` 使用双线性变换：

```text
Ad = (I - Ac·dt/2)⁻¹(I + Ac·dt/2)
Bd = (I - Ac·dt/2)⁻¹(Bc·dt)
```

相比简单前向欧拉，Tustin 方法在较大采样周期下通常有更好的稳定性保持能力。

该函数只离散化 A 和 B。连续时间代价矩阵 Q、R、N 到离散代价的严格变换取决于模型和零阶保持假设，应由设计工具完成。不能简单认为所有场景都只需乘以采样周期。

## LQI 积分增广

`AlgLqrLqi_BuildAugmentedModel()` 构建：

```text
x_aug = [x; integral_error]

A_aug = [ A       0 ]
        [-dt·C    I ]

B_aug = [B]
        [0]
```

然后为增广模型配置 Q 和 R，并调用 DARE 求解即可得到同时包含状态反馈和积分反馈的增益。

积分误差的参考输入属于外部已知项，调用者每周期更新积分状态时需要使用：

```text
integral_error += dt × (reference_output - measured_output)
```

执行器饱和时，LQI 积分状态同样可能累积。应用层应结合限幅状态暂停积分或实现反算，不能仅依赖输出截断。

## 数值实现

- Riccati 矩阵每轮都会恢复对称性。
- 矩阵求逆使用带部分主元选择的 Gauss-Jordan 方法。
- 检测 NaN、无穷值和近似奇异矩阵。
- DARE 使用最大元素变化量判断收敛。
- Q、R 和终端权重检查有限值及非负对角线。
- 控制器支持每个输入通道单独限幅。

调用者仍必须确保：

- `(A, B)` 可稳定。
- Q 为对称半正定矩阵。
- R 为对称正定矩阵。
- 包含 N 时，整体代价矩阵合理。
- 状态量已使用合理尺度或完成归一化。

仅检查对角线不能数学上证明矩阵正定。模型不可控或权重不合理时，求解器可能返回不收敛或奇异矩阵。

## 实时性建议

控制器执行只有矩阵向量乘法，适合高频实时环路。Riccati 求解涉及矩阵乘法、矩阵求逆和多轮迭代，不建议在高优先级中断中运行。

推荐：

- 固定模型：离线求解 K。
- 少量模型切换：预存多组 K。
- 必须在线求解：在低优先级任务或初始化阶段完成。

## 测试覆盖

`Test/alg_lqr_test.c` 覆盖：

- 具有解析解的标量 DARE。
- 二维双积分系统闭环收敛。
- 多状态、多输入参考跟踪。
- 平衡输入、前馈和控制限幅。
- 有限时域反向 Riccati 递推。
- 状态—控制交叉权重 N。
- 连续模型 Tustin 离散化。
- LQI 增广矩阵。
- 奇异矩阵和不收敛状态。
