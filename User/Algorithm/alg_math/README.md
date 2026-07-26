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

```c
float matrix_data[9];
float inverse_data[9];
float workspace[ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(3U)];
alg_math_matrix_t matrix;
alg_math_matrix_t inverse;

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
