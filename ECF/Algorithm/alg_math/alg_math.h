/**
 * @file alg_math.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 基础数学算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       提供标量运算、角度处理、在线统计、插值查表、向量/四元数及动态矩阵操作。
 *       所有数据由调用者管理，不使用动态内存。
 *       矩阵采用行优先连续存储，向量和四元数使用结构体。
 */

#ifndef ALG_MATH_H
#define ALG_MATH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 常量定义 ======================== */

/**
 * @brief 圆周率（单精度）
 */
#define ALG_MATH_PI_F (3.14159265358979323846F)

/**
 * @brief 2π（单精度）
 */
#define ALG_MATH_TWO_PI_F (6.28318530717958647692F)

/**
 * @brief π/2（单精度）
 */
#define ALG_MATH_HALF_PI_F (1.57079632679489661923F)

/**
 * @brief 角度转弧度系数
 */
#define ALG_MATH_DEG_TO_RAD_F (ALG_MATH_PI_F / 180.0F)

/**
 * @brief 弧度转角度系数
 */
#define ALG_MATH_RAD_TO_DEG_F (180.0F / ALG_MATH_PI_F)

/**
 * @brief 计算方阵求逆所需工作区浮点元素数量
 * @param order 矩阵阶数
 * @return 所需 float 数组元素个数
 * @note 工作区用于增广矩阵 [A | I]，大小为 2*order*order。
 */
#define ALG_MATH_MATRIX_INVERSE_WORKSPACE_SIZE(order) (2U * (size_t)(order) * (size_t)(order))

/**
 * @brief 计算线性方程组求解所需工作区浮点元素数量
 * @param order 方程阶数
 * @return 所需 float 数组元素个数
 * @note 工作区用于增广矩阵 [A | b]，大小为 order*(order+1)。
 */
#define ALG_MATH_MATRIX_SOLVE_WORKSPACE_SIZE(order) ((size_t)(order) * ((size_t)(order) + 1U))

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 数学库状态码
     */
    typedef enum
    {
        ALG_MATH_STATUS_OK = 0,           // 操作成功
        ALG_MATH_STATUS_INVALID_ARGUMENT, // 参数非法（空指针、不允许的内存复用等）
        ALG_MATH_STATUS_OUT_OF_RANGE,     // 参数超出范围（NaN、Inf、非法维度、非严格递增等）
        ALG_MATH_STATUS_SIZE_MISMATCH,    // 矩阵维度或工作区大小不匹配
        ALG_MATH_STATUS_SINGULAR,         // 奇异（零向量归一化、奇异矩阵、非正定矩阵）
        ALG_MATH_STATUS_NUMERICAL_ERROR   // 数值错误（溢出、非有限结果）
    } alg_math_status_t;

    /* ======================== 基础向量/四元数结构体 ======================== */

    /**
     * @brief 二维向量
     */
    typedef struct
    {
        float x;
        float y;
    } alg_math_vector2_t;

    /**
     * @brief 三维向量
     */
    typedef struct
    {
        float x;
        float y;
        float z;
    } alg_math_vector3_t;

    /**
     * @brief 四元数（w, x, y, z）
     * @note 表示主动旋转，实部在前，虚部在后。
     */
    typedef struct
    {
        float w;
        float x;
        float y;
        float z;
    } alg_math_quaternion_t;

    /* ======================== 矩阵描述符 ======================== */

    /**
     * @brief 动态尺寸矩阵描述符（不拥有数据）
     * @note 所有数据由调用者提供，矩阵行优先存储。
     */
    typedef struct
    {
        size_t rows;    // 行数
        size_t columns; // 列数
        float *data;    // 数据指针（大小为 rows*columns）
    } alg_math_matrix_t;

    /**
     * @brief 在线统计结构体（Welford 算法）
     * @note 仅保存必要状态，无需历史样本。
     */
    typedef struct
    {
        uint32_t sample_count;           // 样本数
        float mean;                      // 均值
        float sum_of_squared_deviations; // 离差平方和（用于方差）
        float minimum;                   // 最小值
        float maximum;                   // 最大值
    } alg_math_statistics_t;

    /* ======================== 数组和标量工具 ======================== */

    /**
     * @brief 检查浮点数组是否全部有限
     * @param values      数组指针
     * @param value_count 元素个数
     * @return true 表示全部有限
     */
    bool alg_math_is_finite_array(const float *values, size_t value_count);

    /**
     * @brief 将值限幅到指定区间
     * @param value       输入值
     * @param lower_limit 下限
     * @param upper_limit 上限（必须 > lower_limit）
     * @param result      输出限幅结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_clamp(float value, float lower_limit, float upper_limit,
                                     float *result);

    /**
     * @brief 线性插值：start + ratio*(end - start)
     * @param start  起始值
     * @param end    终止值
     * @param ratio  插值系数（建议 [0,1]）
     * @param result 输出插值结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_lerp(float start, float end, float ratio, float *result);

    /**
     * @brief 将值从输入区间映射到输出区间
     * @param value          输入值
     * @param input_minimum  输入区间下限
     * @param input_maximum  输入区间上限（必须 > input_minimum）
     * @param output_minimum 输出区间下限
     * @param output_maximum 输出区间上限（必须 > output_minimum）
     * @param clamp_output   是否将输出限幅到输出区间内
     * @param result         输出映射结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_map_range(float value, float input_minimum, float input_maximum,
                                         float output_minimum, float output_maximum,
                                         bool clamp_output, float *result);

    /**
     * @brief 应用死区处理
     * @param value         输入值
     * @param deadband      死区宽度（0~1）
     * @param rescale_output 是否重新缩放输出到 [0,1] 区间
     * @param result        输出结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_apply_deadband(float value, float deadband, bool rescale_output,
                                              float *result);

    /**
     * @brief 将值回绕到 [lower_bound, upper_bound) 区间
     * @param value        输入值
     * @param lower_bound  下限
     * @param upper_bound  上限（必须 > lower_bound）
     * @param result       输出回绕结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_wrap(float value, float lower_bound, float upper_bound,
                                    float *result);

    /**
     * @brief 将角度回绕到 [-π, π) 区间
     * @param angle_rad  输入角度（弧度）
     * @param result_rad 输出回绕角度
     * @return 执行状态
     */
    alg_math_status_t alg_math_wrap_angle_pi(float angle_rad, float *result_rad);

    /**
     * @brief 计算两个角度的最短差值（结果在 [-π, π) 区间）
     * @param target_rad    目标角度
     * @param current_rad   当前角度
     * @param difference_rad 输出差值（target - current）
     * @return 执行状态
     */
    alg_math_status_t alg_math_angle_difference(float target_rad, float current_rad,
                                                float *difference_rad);

    /**
     * @brief 角度转弧度
     * @param angle_deg 角度值
     * @return 弧度值
     */
    float alg_math_degrees_to_radians(float angle_deg);

    /**
     * @brief 弧度转角度
     * @param angle_rad 弧度值
     * @return 角度值
     */
    float alg_math_radians_to_degrees(float angle_rad);

    /**
     * @brief 安全平方根（检查输入非负）
     * @param value  输入值（>=0）
     * @param result 输出平方根
     * @return 执行状态
     */
    alg_math_status_t alg_math_safe_sqrt(float value, float *result);

    /**
     * @brief 安全除法（检查分母绝对值是否过小）
     * @param numerator           分子
     * @param denominator         分母
     * @param minimum_denominator 最小允许绝对值（>0）
     * @param result              输出商
     * @return 执行状态
     */
    alg_math_status_t alg_math_safe_divide(float numerator, float denominator,
                                           float minimum_denominator, float *result);

    /* ======================== 统计功能 ======================== */

    /**
     * @brief 初始化统计结构体
     * @param me 统计对象
     * @return 执行状态
     */
    alg_math_status_t alg_math_statistics_init(alg_math_statistics_t *me);

    /**
     * @brief 更新统计（Welford 在线算法）
     * @param me     统计对象
     * @param sample 新样本
     * @return 执行状态
     */
    alg_math_status_t alg_math_statistics_update(alg_math_statistics_t *me, float sample);

    /**
     * @brief 获取总体方差（分母为 N）
     * @param me       统计对象
     * @param variance 输出方差
     * @return 执行状态
     */
    alg_math_status_t alg_math_statistics_get_population_variance(const alg_math_statistics_t *me,
                                                                  float *variance);

    /**
     * @brief 获取样本方差（分母为 N-1）
     * @param me       统计对象
     * @param variance 输出方差
     * @return 执行状态
     */
    alg_math_status_t alg_math_statistics_get_sample_variance(const alg_math_statistics_t *me,
                                                              float *variance);

    /**
     * @brief 获取标准差
     * @param me                       统计对象
     * @param sample_standard_deviation true: 样本标准差；false: 总体标准差
     * @param standard_deviation       输出标准差
     * @return 执行状态
     */
    alg_math_status_t alg_math_statistics_get_standard_deviation(const alg_math_statistics_t *me,
                                                                 bool sample_standard_deviation,
                                                                 float *standard_deviation);

    /**
     * @brief 计算数组均值
     * @param values     数组
     * @param value_count 元素个数（>0）
     * @param mean       输出均值
     * @return 执行状态
     */
    alg_math_status_t alg_math_array_mean(const float *values, size_t value_count, float *mean);

    /**
     * @brief 计算数组均方根（RMS）
     * @param values     数组
     * @param value_count 元素个数（>0）
     * @param rms        输出 RMS
     * @return 执行状态
     */
    alg_math_status_t alg_math_array_rms(const float *values, size_t value_count, float *rms);

    /* ======================== 插值功能 ======================== */

    /**
     * @brief 一维分段线性插值
     * @param x_values     横坐标表（必须严格递增）
     * @param y_values     纵坐标表
     * @param point_count  点数（>=2）
     * @param input        输入横坐标
     * @param clamp_to_table 是否将输入限幅到表范围
     * @param output       输出插值结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_interpolate_linear1_d(const float *x_values, const float *y_values,
                                                     size_t point_count, float input,
                                                     bool clamp_to_table, float *output);

    /**
     * @brief 双线性插值（单位正方形内）
     * @param x_ratio     x方向插值系数 [0,1]
     * @param y_ratio     y方向插值系数 [0,1]
     * @param value_00    左下角值
     * @param value_10    右下角值
     * @param value_01    左上角值
     * @param value_11    右上角值
     * @param output      输出插值结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_interpolate_bilinear(float x_ratio, float y_ratio, float value_00,
                                                    float value_10, float value_01, float value_11,
                                                    float *output);

    /* ======================== 向量运算（2D） ======================== */

    /**
     * @brief 二维向量相加
     */
    alg_math_status_t alg_math_vector2_add(const alg_math_vector2_t *left,
                                           const alg_math_vector2_t *right,
                                           alg_math_vector2_t *result);

    /**
     * @brief 二维向量相减
     */
    alg_math_status_t alg_math_vector2_subtract(const alg_math_vector2_t *left,
                                                const alg_math_vector2_t *right,
                                                alg_math_vector2_t *result);

    /**
     * @brief 二维向量缩放
     */
    alg_math_status_t alg_math_vector2_scale(const alg_math_vector2_t *vector, float scale,
                                             alg_math_vector2_t *result);

    /**
     * @brief 二维向量点积
     */
    alg_math_status_t alg_math_vector2_dot(const alg_math_vector2_t *left,
                                           const alg_math_vector2_t *right, float *result);

    /**
     * @brief 二维向量模长
     */
    alg_math_status_t alg_math_vector2_norm(const alg_math_vector2_t *vector, float *norm);

    /**
     * @brief 二维向量归一化（返回单位向量）
     */
    alg_math_status_t alg_math_vector2_normalize(const alg_math_vector2_t *vector,
                                                 alg_math_vector2_t *result);

    /* ======================== 向量运算（3D） ======================== */

    /**
     * @brief 三维向量相加
     */
    alg_math_status_t alg_math_vector3_add(const alg_math_vector3_t *left,
                                           const alg_math_vector3_t *right,
                                           alg_math_vector3_t *result);

    /**
     * @brief 三维向量相减
     */
    alg_math_status_t alg_math_vector3_subtract(const alg_math_vector3_t *left,
                                                const alg_math_vector3_t *right,
                                                alg_math_vector3_t *result);

    /**
     * @brief 三维向量缩放
     */
    alg_math_status_t alg_math_vector3_scale(const alg_math_vector3_t *vector, float scale,
                                             alg_math_vector3_t *result);

    /**
     * @brief 三维向量点积
     */
    alg_math_status_t alg_math_vector3_dot(const alg_math_vector3_t *left,
                                           const alg_math_vector3_t *right, float *result);

    /**
     * @brief 三维向量叉积
     */
    alg_math_status_t alg_math_vector3_cross(const alg_math_vector3_t *left,
                                             const alg_math_vector3_t *right,
                                             alg_math_vector3_t *result);

    /**
     * @brief 三维向量模长
     */
    alg_math_status_t alg_math_vector3_norm(const alg_math_vector3_t *vector, float *norm);

    /**
     * @brief 三维向量归一化
     */
    alg_math_status_t alg_math_vector3_normalize(const alg_math_vector3_t *vector,
                                                 alg_math_vector3_t *result);

    /* ======================== 四元数运算 ======================== */

    /**
     * @brief 获取单位四元数（恒等旋转）
     */
    alg_math_status_t alg_math_quaternion_identity(alg_math_quaternion_t *result);

    /**
     * @brief 归一化四元数
     */
    alg_math_status_t alg_math_quaternion_normalize(const alg_math_quaternion_t *quaternion,
                                                    alg_math_quaternion_t *result);

    /**
     * @brief 四元数共轭
     */
    alg_math_status_t alg_math_quaternion_conjugate(const alg_math_quaternion_t *quaternion,
                                                    alg_math_quaternion_t *result);

    /**
     * @brief 四元数乘法（左乘）
     */
    alg_math_status_t alg_math_quaternion_multiply(const alg_math_quaternion_t *left,
                                                   const alg_math_quaternion_t *right,
                                                   alg_math_quaternion_t *result);

    /**
     * @brief 从 ZYX 欧拉角（滚转、俯仰、偏航）构建四元数
     * @param roll_rad   滚转角（弧度）
     * @param pitch_rad  俯仰角（弧度）
     * @param yaw_rad    偏航角（弧度）
     * @param result     输出四元数
     * @return 执行状态
     */
    alg_math_status_t alg_math_quaternion_from_euler(float roll_rad, float pitch_rad, float yaw_rad,
                                                     alg_math_quaternion_t *result);

    /**
     * @brief 将四元数转换为 ZYX 欧拉角
     * @param quaternion 输入四元数
     * @param euler_rad  输出欧拉角（x=roll, y=pitch, z=yaw）
     * @return 执行状态
     */
    alg_math_status_t alg_math_quaternion_to_euler(const alg_math_quaternion_t *quaternion,
                                                   alg_math_vector3_t *euler_rad);

    /**
     * @brief 用四元数旋转三维向量
     * @param quaternion 四元数
     * @param vector     待旋转向量
     * @param result     旋转后向量
     * @return 执行状态
     */
    alg_math_status_t alg_math_quaternion_rotate_vector(const alg_math_quaternion_t *quaternion,
                                                        const alg_math_vector3_t *vector,
                                                        alg_math_vector3_t *result);

    /**
     * @brief 四元数球面线性插值（SLERP）
     * @param start   起始四元数
     * @param end     终止四元数
     * @param ratio   插值系数 [0,1]
     * @param result  输出插值结果
     * @return 执行状态
     */
    alg_math_status_t alg_math_quaternion_slerp(const alg_math_quaternion_t *start,
                                                const alg_math_quaternion_t *end, float ratio,
                                                alg_math_quaternion_t *result);

    /* ======================== 动态矩阵运算 ======================== */

    /**
     * @brief 初始化矩阵描述符
     * @param matrix  矩阵对象
     * @param data    数据缓冲区
     * @param rows    行数
     * @param columns 列数
     * @return 执行状态
     */
    alg_math_status_t alg_math_matrix_init(alg_math_matrix_t *matrix, float *data, size_t rows,
                                           size_t columns);

    /**
     * @brief 矩阵清零
     */
    alg_math_status_t alg_math_matrix_zero(alg_math_matrix_t *matrix);

    /**
     * @brief 设置单位矩阵（必须为方阵）
     */
    alg_math_status_t alg_math_matrix_identity(alg_math_matrix_t *matrix);

    /**
     * @brief 复制矩阵
     */
    alg_math_status_t alg_math_matrix_copy(const alg_math_matrix_t *source,
                                           alg_math_matrix_t *destination);

    /**
     * @brief 矩阵加法
     */
    alg_math_status_t alg_math_matrix_add(const alg_math_matrix_t *left,
                                          const alg_math_matrix_t *right,
                                          alg_math_matrix_t *result);

    /**
     * @brief 矩阵减法
     */
    alg_math_status_t alg_math_matrix_subtract(const alg_math_matrix_t *left,
                                               const alg_math_matrix_t *right,
                                               alg_math_matrix_t *result);

    /**
     * @brief 矩阵缩放
     */
    alg_math_status_t alg_math_matrix_scale(const alg_math_matrix_t *input, float scale,
                                            alg_math_matrix_t *result);

    /**
     * @brief 矩阵乘法（禁止 result 与输入复用）
     */
    alg_math_status_t alg_math_matrix_multiply(const alg_math_matrix_t *left,
                                               const alg_math_matrix_t *right,
                                               alg_math_matrix_t *result);

    /**
     * @brief 矩阵转置（支持原地转置，但仅限方阵）
     */
    alg_math_status_t alg_math_matrix_transpose(const alg_math_matrix_t *input,
                                                alg_math_matrix_t *result);

    /**
     * @brief 矩阵乘向量：result = matrix * vector
     * @param matrix       矩阵（rows×columns）
     * @param vector       向量（长度=columns）
     * @param vector_length 向量长度
     * @param result       输出向量（长度=rows）
     * @param result_length 输出长度
     * @return 执行状态
     */
    alg_math_status_t alg_math_matrix_multiply_vector(const alg_math_matrix_t *matrix,
                                                      const float *vector, size_t vector_length,
                                                      float *result, size_t result_length);

    /**
     * @brief 矩阵求逆（Gauss-Jordan，部分主元）
     * @param input          输入方阵
     * @param inverse        输出逆矩阵
     * @param workspace      工作区（大小由宏计算）
     * @param workspace_size 工作区大小
     * @return 执行状态
     */
    alg_math_status_t alg_math_matrix_invert(const alg_math_matrix_t *input,
                                             alg_math_matrix_t *inverse, float *workspace,
                                             size_t workspace_size);

    /**
     * @brief 线性方程组求解 Ax = b（Gauss 消元，部分主元）
     * @param coefficients   系数矩阵 A（方阵）
     * @param right_hand_side 右端向量 b
     * @param solution       解向量 x
     * @param workspace      工作区
     * @param workspace_size 工作区大小
     * @return 执行状态
     */
    alg_math_status_t alg_math_matrix_solve(const alg_math_matrix_t *coefficients,
                                            const float *right_hand_side, float *solution,
                                            float *workspace, size_t workspace_size);

    /**
     * @brief Cholesky 分解 A = L*L^T（对称正定）
     * @param input             输入对称正定矩阵（仅检查近似对称）
     * @param lower_triangular  输出下三角 L
     * @return 执行状态
     * @note 输入与输出不能使用同一数据指针。
     */
    alg_math_status_t alg_math_matrix_cholesky(const alg_math_matrix_t *input,
                                               alg_math_matrix_t *lower_triangular);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MATH_H */