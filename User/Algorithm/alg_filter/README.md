# alg_filter — 数字滤波器库

纯算法。所有滤波器使用 `init` + `update` 模式，时间步长用秒。

## 包含的滤波器

| 类型 | 结构体 | 用途 |
|------|--------|------|
| 一阶低通 | `alg_filter_low_pass_t` | 平滑传感器噪声 |
| 一阶高通 | `alg_filter_high_pass_t` | 去除直流分量 |
| 指数平滑 | `alg_filter_exponential_t` | 极简平滑 |
| 滑动平均 | `alg_filter_moving_average_t` | 固定窗口平均 |
| 中值滤波 | `alg_filter_median_t` | 去除脉冲噪声 |
| FIR | `alg_filter_fir_t` | 自定义系数 |
| 双二阶 | `alg_filter_biquad_t` | IIR 二阶 |
| 互补滤波 | `alg_filter_complementary_t` | 融合高低频信号 |

## 一阶低通

```c
alg_filter_low_pass_t lpf;
alg_filter_low_pass_init(&lpf, 50.0f);  // 截止频率 50Hz

float filtered = 0.0f;
alg_filter_low_pass_update(&lpf, raw_value, 0.001f, &filtered);
```

## 滑动平均

```c
float buffer[16];
alg_filter_moving_average_t ma;
alg_filter_moving_average_init(&ma, buffer, 16);

float avg;
alg_filter_moving_average_update(&ma, raw, &avg);
```

## 互补滤波（陀螺+加速度融合）

```c
alg_filter_complementary_t comp;
alg_filter_complementary_init(&comp, 0.98f);  // 陀螺权重 98%

float angle;
alg_filter_complementary_update(&comp, gyro_rate * dt, accel_angle, &angle);
```

## API 模式

所有滤波器统一使用：
```c
alg_filter_status_t xxx_init(xxx_t *me, params...);
alg_filter_status_t xxx_update(xxx_t *me, float input, float dt, float *output);
alg_filter_status_t xxx_reset(xxx_t *me);
```

状态码：`OK` / `INVALID_ARGUMENT` / `OUT_OF_RANGE` / `NOT_INITIALIZED` / `NUMERICAL_ERROR`
