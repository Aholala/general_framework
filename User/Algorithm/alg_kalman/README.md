# alg_kalman

`alg_kalman` 是 Algorithm 层独立的卡尔曼滤波库。源码和头文件直接位于模块目录，不再划分 `Inc` 和 `Src`。

## 提供的算法

| 算法 | 对象 | 适用场景 |
|---|---|---|
| 标量卡尔曼 | `AlgKalmanScalar_t` | 单个传感器量的低成本递推估计 |
| 任意维线性卡尔曼 | `AlgKalmanLinear_t` | 状态模型和观测模型均为线性系统 |
| 扩展卡尔曼 EKF | `AlgKalmanExtended_t` | 非线性状态模型或非线性观测模型 |

线性 KF 与 EKF 都没有固定维度上限。状态维数、测量维数和控制维数由初始化配置决定。

## 可移植性

- 纯 C11。
- 不依赖 HAL、CMSIS 或 RTOS。
- 不使用动态内存。
- 不读取系统时钟。
- 不使用可变全局状态。
- 所有矩阵均为调用者持有的连续 `float` 数组。
- 工作区由调用者静态提供。
- 支持多个互不干扰的滤波器实例。
- 只依赖 `<stdbool.h>`、`<stddef.h>` 和 `<math.h>`。

## 矩阵存储规则

所有矩阵均使用行优先连续存储：

```text
A(row, column) = A[row * column_count + column]
```

线性卡尔曼模型为：

```text
x(k) = F x(k-1) + B u(k)
P(k) = F P(k-1) F^T + Q
z(k) = H x(k) + v(k)
```

对应关系：

| 配置成员 | 矩阵 | 尺寸 |
|---|---|---|
| `state` | x | `n × 1` |
| `covariance` | P | `n × n` |
| `transition_matrix` | F | `n × n` |
| `control_matrix` | B | `n × c` |
| `process_noise` | Q | `n × n` |
| `measurement_matrix` | H | `m × n` |
| `measurement_noise` | R | `m × m` |

其中 `n` 是状态维数，`m` 是测量维数，`c` 是控制输入维数。

## 静态工作区

使用宏计算所需的 `float` 元素数量：

```c
enum
{
    STATE_DIMENSION = 2,
    MEASUREMENT_DIMENSION = 1
};

static float s_workspace[
    ALG_KALMAN_WORKSPACE_SIZE(STATE_DIMENSION, MEASUREMENT_DIMENSION)];
```

传入 `workspace_size` 的是元素数量，不是字节数量：

```c
.workspace_size = sizeof(s_workspace) / sizeof(s_workspace[0])
```

工作区会在每次预测或校正中被覆盖，不能用于保存其他长期数据。

## 标量卡尔曼

```c
static AlgKalmanScalar_t s_temperature_filter;

void TemperatureFilter_Init(void)
{
    (void)AlgKalmanScalar_Init(&s_temperature_filter,
                               0.01F,
                               0.50F,
                               25.0F,
                               1.0F);
}

AlgKalmanStatus_t TemperatureFilter_Update(float measurement,
                                           float *filtered_temperature)
{
    return AlgKalmanScalar_Update(&s_temperature_filter,
                                  measurement,
                                  filtered_temperature);
}
```

如果系统存在已知状态增量，可分开调用：

```c
AlgKalmanScalar_Predict(&filter, known_state_delta);
AlgKalmanScalar_Correct(&filter, measurement, &output);
```

## 线性卡尔曼

`AlgKalmanLinearConfig_t` 保存矩阵地址，不复制矩阵。所有数组必须在滤波器整个生命周期内持续有效。

时间变化的模型可以在每次预测前更新调用者持有的 `F`、`B`、`Q`、`H` 或 `R` 数组，无需重新初始化对象。

当 `control_dimension` 为零时：

- `control_matrix` 可以为 `NULL`。
- `AlgKalmanLinear_Predict()` 的控制输入可以为 `NULL`。

当 `control_dimension` 大于零时，控制矩阵和控制输入都不能为空。

## 扩展卡尔曼

EKF 通过四个回调注入具体模型：

- `state_function`：计算非线性状态预测 `f(x, u, dt)`。
- `state_jacobian_function`：计算状态雅可比 `F = ∂f/∂x`。
- `measurement_function`：计算预测测量 `h(x)`。
- `measurement_jacobian_function`：计算观测雅可比 `H = ∂h/∂x`。

回调收到 `user_context`，可访问只读模型参数，不需要依赖全局变量。回调必须填满输出数组，并确保所有结果为有限浮点数。

## 数值稳定性

- 测量校正使用 Joseph 形式更新协方差：

```text
P = (I-KH)P(I-KH)^T + KRK^T
```

- 每次更新后主动恢复协方差矩阵的对称性。
- 创新协方差使用带部分主元选择的 Gauss-Jordan 方法求逆。
- 奇异创新矩阵返回 `ALG_KALMAN_STATUS_SINGULAR_MATRIX`。
- 数值错误不会提交新的状态和协方差。
- 初始化检查所有输入的有限性以及协方差、Q、R 的非负对角线。

调用者仍必须保证初始协方差、Q 和 R 是合理的对称半正定矩阵。只检查对角线不能完整证明矩阵半正定。

## 单位规则

卡尔曼库无法替调用者判断单位。构造模型时必须保证：

- 状态、控制量和测量量的单位一致。
- F、B、H、Q、R 使用同一离散采样周期。
- EKF 的 `delta_time_s` 单位固定为秒。
- 角度状态统一采用弧度或角度，不在同一模型中混用。

## 并发规则

同一个对象不能同时在任务和中断中无保护调用。推荐一个滤波对象只由一个执行上下文持有；跨上下文数据通过消息队列或双缓冲传递。

## 测试覆盖

`Test/alg_kalman_test.c` 覆盖：

- 标量卡尔曼收敛、复位及独立预测/校正。
- 二维恒加速度线性模型。
- 多维测量和矩阵求逆。
- 非线性平方观测 EKF。
- Joseph 协方差更新后的对称性。
- 空指针、未初始化对象和非法参数。
- 工作区不足。
- 奇异创新协方差。
