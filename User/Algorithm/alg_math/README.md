# alg_math

`alg_math` 是 Algorithm 层的无状态基础数学库，用于统一控制、滤波、姿态、标定和模型
算法中的公共数学操作。模块使用 C11、单精度浮点数和调用者管理的静态内存，不依赖
HAL、RTOS、芯片型号或动态内存。

## 模块内容

### 标量与角度

- 限幅、线性插值、区间映射；
- 死区处理，可选择消除死区后的幅值断层；
- 任意区间回绕、`[-pi, pi)` 角度回绕和最短角差；
- 角度/弧度转换；
- 带错误返回的平方根和除法。

### 统计与查表

- Welford 在线均值、总体/样本方差、标准差、最小值和最大值；
- 数组均值与 RMS；
- 严格递增横坐标的一维分段线性插值；
- 双线性插值。

在线统计只保存固定大小状态，不需要保留历史样本，适合传感器噪声评估和运行监控。

### 向量与四元数

- 二维/三维向量加减、缩放、点积、叉积、模长和归一化；
- 四元数单位化、共轭、乘法；
- ZYX Roll/Pitch/Yaw 与四元数互转；
- 四元数旋转三维向量；
- 最短路径球面线性插值 SLERP。

四元数约定为 `(w, x, y, z)`。`alg_math_quaternion_from_euler` 使用 ZYX 旋转顺序，旋转向量时四元数表示
主动旋转。具体应用仍应在模块文档中声明“机体到世界”或“世界到机体”的方向。

### 动态尺寸矩阵

- 初始化、清零、单位阵、复制；
- 加减、缩放、转置、矩阵乘法、矩阵向量乘法；
- 带部分主元选择的 Gauss-Jordan 求逆；
- 带部分主元选择的高斯消元线性方程求解；
- 对称正定矩阵 Cholesky 分解。

矩阵采用行优先存储。描述符不拥有数据，所有数组和工作区都由调用者提供：

````c
float matrix_data[9];
float inverse_data[9];
float workspace[ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(3U)];
alg_math_matrix_t matrix;
alg_math_matrix_t inverse;
# alg_math —— 完整使用指南

## 1. 模块概述

`alg_math` 是 Algorithm 层独立的基础数学库，提供标量运算、角度处理、在线统计、插值查表、向量/四元数运算以及动态矩阵操作。所有算法使用纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS，不使用动态内存，所有数据由调用者管理。

**核心功能**：

- **标量与角度**：限幅、线性插值、区间映射、死区、角度回绕、角度差、安全平方根/除法
- **统计与查表**：Welford 在线统计（均值、方差、标准差、最小/最大值）、数组均值/RMS、一维线性插值、双线性插值
- **向量与四元数**：2D/3D 向量加减、缩放、点积、叉积、模长、归一化；四元数单位化、共轭、乘法、欧拉角转换、旋转向量、SLERP
- **动态矩阵**：初始化、清零、单位阵、复制、加减、缩放、转置、矩阵乘法、矩阵×向量、求逆（Gauss-Jordan）、线性方程组求解（Gauss 消元）、Cholesky 分解

**设计哲学**：

- **零动态内存**：所有矩阵数据、工作区和输出均由调用者静态分配
- **纯标准库依赖**：只依赖 `<stdbool.h>`、`<stddef.h>`、`<stdint.h>`、`<math.h>`
- **显式错误返回**：每个可能失败的接口返回 `alg_math_status_t`，便于诊断
- **数值鲁棒**：部分主元消元、有限性检查、对称性检查（Cholesky）
- **多实例支持**：矩阵描述符和统计结构体可创建多个独立实例

---

## 2. 矩阵存储规则

所有矩阵使用**行优先连续存储**：

```text
A(row, column) = A[row * column_count + column]
````

矩阵描述符 `alg_math_matrix_t` 只保存指针和尺寸，不拥有数据。调用者需保证数据缓冲区在整个生命周期内有效。

---

## 3. 功能分组与适用场景

| 分组        | 功能                           | 适用场景                             |
| :---------- | :----------------------------- | :----------------------------------- |
| 标量/角度   | 限幅、映射、回绕、角度差       | 传感器校准、控制信号处理、角度归一化 |
| 统计        | 在线均值/方差、RMS             | 实时噪声评估、性能监控               |
| 插值        | 一维/双线性查表                | 标定曲线、LUT 插值                   |
| 向量/四元数 | 加减、叉积、旋转、SLERP        | 姿态估计、导航、坐标系变换           |
| 矩阵运算    | 乘、转置、求逆、求解、Cholesky | 卡尔曼滤波、LQR、最小二乘            |

---

## 4. 使用示例

### 4.1 标量与角度

```c
#include "alg_math.h"

void example_scalar(void) {
    float clamped, mapped, angle_diff;

    // 限幅
    alg_math_clamp(15.0f, 0.0f, 10.0f, &clamped); // clamped=10.0

    // 区间映射（将 0~10 映射到 0~100，并限幅）
    alg_math_map_range(5.0f, 0.0f, 10.0f, 0.0f, 100.0f, true, &mapped); // mapped=50.0

    // 角度差（目标 - 当前，最短路径）
    alg_math_angle_difference(3.0f, 4.0f, &angle_diff); // angle_diff ≈ -1.0 (自动回绕)
}
```

### 4.2 在线统计（Welford 算法）

```c
static alg_math_statistics_t s_stats;

void init_stats(void) {
    alg_math_statistics_init(&s_stats);
}

void process_sample(float value) {
    alg_math_statistics_update(&s_stats, value);
}

void get_variance(void) {
    float var;
    alg_math_statistics_get_population_variance(&s_stats, &var);
}
```

### 4.3 一维线性插值

```c
#define TABLE_SIZE 5
static float x_table[TABLE_SIZE] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f};
static float y_table[TABLE_SIZE] = {0.0f, 0.5f, 1.0f, 1.5f, 2.0f};

float interpolate(float input) {
    float output;
    alg_math_interpolate_linear1_d(x_table, y_table, TABLE_SIZE, input, true, &output);
    return output;
}
```

### 4.4 四元数操作

```c
void example_quaternion(void) {
    alg_math_quaternion_t q, q_conj;
    alg_math_vector3_t v = {1.0f, 0.0f, 0.0f};
    alg_math_vector3_t v_rot;

    // 从欧拉角创建四元数（ZYX 顺序）
    alg_math_quaternion_from_euler(0.1f, 0.2f, 0.3f, &q);

    // 归一化
    alg_math_quaternion_normalize(&q, &q);

    // 共轭
    alg_math_quaternion_conjugate(&q, &q_conj);

    // 旋转向量
    alg_math_quaternion_rotate_vector(&q, &v, &v_rot);

    // SLERP
    alg_math_quaternion_t start, end, result;
    alg_math_quaternion_identity(&start);
    // ... 设置 end
    alg_math_quaternion_slerp(&start, &end, 0.5f, &result);
}
```

### 4.5 矩阵运算

```c
#define N 3
float A_data[N*N] = { ... };
float B_data[N*N];
float workspace[ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(N)];
alg_math_matrix_t A, B;

alg_math_matrix_init(&A, A_data, N, N);
alg_math_matrix_init(&B, B_data, N, N);

// 求逆
alg_math_matrix_invert(&A, &B, workspace, sizeof(workspace)/sizeof(float));

// 线性方程求解 Ax = b
float b[N] = {1, 2, 3};
float x[N];
float solve_workspace[ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(N)];
alg_math_matrix_solve(&A, b, x, solve_workspace,
                      ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(N));
```

---

## 5. 工作区大小计算

使用预定义宏计算矩阵求逆和线性方程组求解所需工作区的浮点元素数量：

```c
// 求逆工作区：2 * n * n
float inv_workspace[ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(3)];  // 3×3 矩阵

// 求解工作区：n * (n+1)
float solve_workspace[ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(3)];   // 3元方程组
```

工作区大小的单位是 `float` 元素数量，**不是字节**。

---

## 6. 数值稳定性

- **矩阵求逆/求解**：使用部分主元选择，降低数值误差。
- **对称性检查**：Cholesky 分解前检查输入矩阵的对称性（容忍 `1e-5` 相对误差）。
- **奇异阈值**：主元绝对值小于 `1e-12` 时视为奇异。
- **有限性检查**：所有输入数组和结果均检查 `isfinite()`。
- **安全运算**：`alg_math_safe_sqrt` 和 `alg_math_safe_divide` 显式处理非法输入。

---

## 7. 错误处理

所有可能失败的接口返回 `alg_math_status_t`，定义如下：

| 状态码                             | 含义                               |
| :--------------------------------- | :--------------------------------- |
| `ALG_MATH_STATUS_OK`               | 操作成功                           |
| `ALG_MATH_STATUS_INVALID_ARGUMENT` | 空指针或不允许的内存复用           |
| `ALG_MATH_STATUS_OUT_OF_RANGE`     | NaN、Inf、非法范围、非严格递增表等 |
| `ALG_MATH_STATUS_SIZE_MISMATCH`    | 矩阵维度或工作区大小不匹配         |
| `ALG_MATH_STATUS_SINGULAR`         | 零向量归一化、奇异矩阵、非正定矩阵 |
| `ALG_MATH_STATUS_NUMERICAL_ERROR`  | 运算结果溢出或变为非有限值         |

**关键约定**：不要忽略矩阵求逆、求解、Cholesky 和归一化接口的返回值。

---

## 8. 实时性建议

- **高频调用**：向量、四元数、标量函数均为 O(1) 或 O(n)，适合实时中断。
- **矩阵运算**：求逆、求解、Cholesky 的复杂度为 O(n³)，不建议在高优先级中断中调用大型矩阵（n>10）。推荐在低优先级任务或离线完成。
- **工作区复用**：矩阵求逆和求解使用工作区，可重用同一缓冲区以节省内存。

---

## 9. 并发约束

- 每个矩阵描述符、统计结构体、向量/四元数对象**不能**被多个执行上下文同时修改。
- 若需多任务共享，使用互斥或消息传递。

---

## 10. 建议验证测试项

- [ ] 标量：限幅、映射、死区、角度回绕、角度差
- [ ] 统计：Welford 算法与暴力计算对比
- [ ] 插值：一维线性插值与双线性插值边界情况
- [ ] 向量：点积、叉积、归一化（包括零向量）
- [ ] 四元数：欧拉角往返、旋转向量与轴角对比、SLERP 插值
- [ ] 矩阵：乘、转置（原地/非原地）、求逆、求解、Cholesky（含非正定检测）
- [ ] 错误路径：空指针、尺寸不匹配、奇异矩阵、工作区不足

---

**总结**：`alg_math` 为嵌入式系统提供了全面的基础数学工具，覆盖从简单标量到复杂矩阵运算的各类场景。其无动态内存、纯 C11 的实现使其非常适合资源受限的实时环境。配合上层控制、滤波、导航等模块，可构建完整的算法栈。
alg_math_matrix_init(&matrix, matrix_data, 3U, 3U);
alg_math_matrix_init(&inverse, inverse_data, 3U, 3U);
alg_math_matrix_invert(&matrix,
&inverse,
workspace,
ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(3U));

```

矩阵乘法和矩阵向量乘法禁止结果覆盖输入。方阵转置支持原地操作；矩阵求逆和线性方程
会先复制到工作区，因此输出可以与输入复用。Cholesky 输入与输出不能使用同一数组。

## 错误处理

所有可能失败的接口返回 `alg_math_status_t`：

- `INVALID_ARGUMENT`：空指针或不允许的内存复用；
- `OUT_OF_RANGE`：NaN、Inf、非法范围或非对称 Cholesky 输入；
- `SIZE_MISMATCH`：矩阵维度或工作区大小不匹配；
- `SINGULAR`：零向量归一化、除数过小、奇异矩阵或非正定矩阵；
- `NUMERICAL_ERROR`：运算结果溢出或变为非有限值。

不要忽略矩阵求逆、求解和归一化接口的返回值。

## 使用原则

- 算法层统一使用 `float`，避免无意触发 Cortex-M 上的软件双精度运算。
- 传感器轴向、单位换算和硬件标定属于 Module/BSP 层，不放进本模块。
- 高频固定维度运算可在上层封装专用 2×2、3×3 版本，以减少循环和分支开销。
- 能用 `Solve(A, b)` 时不要先计算 `Inverse(A)`，求解通常更快且数值稳定性更好。
- 对协方差等对称正定矩阵优先使用 Cholesky，而不是通用求逆。

## 验证建议

集成时至少验证标量边界、角度跨界、在线统计、插值、向量归一化、叉积、
欧拉角/四元数转换、向量旋转、SLERP、矩阵求逆、矩阵乘法、线性方程、Cholesky 和
奇异输入。涉及坐标系的模块还必须验证右手系、旋转方向和四元数乘法顺序。

```
