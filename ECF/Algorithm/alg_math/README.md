# alg_math — 数学工具库

向量/矩阵/四元数运算 + 插值 + 统计。

## 主要类型

| 类型 | 说明 |
|------|------|
| `alg_math_vector2_t` | 2D 向量 `{x, y}` |
| `alg_math_vector3_t` | 3D 向量 `{x, y, z}` |
| `alg_math_quaternion_t` | 四元数 `{q0, q1, q2, q3}` |
| `alg_math_matrix_t` | 矩阵 |
| `alg_math_statistics_t` | 在线均值/方差/标准差 |

## 常用函数

```c
// 向量
float dot = alg_math_vector3_dot(&a, &b);
alg_math_vector3_t c = alg_math_vector3_cross(&a, &b);
float mag = alg_math_vector3_magnitude(&v);
alg_math_vector3_t n = alg_math_vector3_normalize(&v);

// 四元数
alg_math_quaternion_t q = alg_math_quaternion_from_euler(roll, pitch, yaw);
alg_math_quaternion_t qm = alg_math_quaternion_multiply(&q1, &q2);
alg_math_vector3_t v = alg_math_quaternion_rotate(&q, &vec);

// 插值
float y = alg_math_linear_interpolate_table(x, table_x, table_y, n);
float z = alg_math_bilinear_interpolate(x, y, grid, cols, rows);

// 统计
alg_math_statistics_t stat;
alg_math_statistics_init(&stat);
alg_math_statistics_push(&stat, sample);
float mean = alg_math_statistics_get_mean(&stat);
float std = alg_math_statistics_get_std(&stat);
```
