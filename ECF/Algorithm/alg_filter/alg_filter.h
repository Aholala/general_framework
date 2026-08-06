/**
 * @file alg_filter.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 纯 C11 通用滤波算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 不包含 STM32、HAL、CMSIS 或 RTOS 头文件。
 *       不调用系统时间函数，采样周期由调用者显式传入。
 *       不使用动态内存，不使用可变全局状态。
 *       每个对象拥有独立状态，支持任意数量的静态实例。
 *       只依赖 C11 标准库中的 <stdbool.h>、<stddef.h> 和 <math.h>。
 */

#ifndef ALG_FILTER_H
#define ALG_FILTER_H

#include <stdbool.h> // bool 类型
#include <stddef.h>  // size_t

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /** @brief 滤波库公共状态码 */
    typedef enum
    {
        ALG_FILTER_STATUS_OK = 0,           // 操作成功
        ALG_FILTER_STATUS_INVALID_ARGUMENT, // 参数非法（空指针等）
        ALG_FILTER_STATUS_OUT_OF_RANGE,     // 参数超出范围（如负数、零等）
        ALG_FILTER_STATUS_NOT_INITIALIZED,  // 对象未初始化
        ALG_FILTER_STATUS_NUMERICAL_ERROR   // 数值错误（溢出、非有限数等）
    } alg_filter_status_t;

    /* ======================== 一阶低通滤波器 ======================== */

    /**
     * @brief 一阶 RC 低通滤波器实例
     * @note 实现：y = y + (dt / (tau + dt)) * (x - y)
     *       tau = 1 / (2*pi*fc)
     */
    typedef struct
    {
        float cutoff_frequency_hz; // 截止频率（Hz）
        float output;              // 当前输出值
        bool is_initialized;       // 是否已初始化
        bool has_previous_sample;  // 是否有上次采样（用于首次直接赋值）
    } alg_filter_low_pass_t;

    /* ======================== 一阶高通滤波器 ======================== */

    /**
     * @brief 一阶 RC 高通滤波器实例
     * @note 实现：y = (tau / (tau + dt)) * (y_prev + x - x_prev)
     *       tau = 1 / (2*pi*fc)
     */
    typedef struct
    {
        float cutoff_frequency_hz; // 截止频率（Hz）
        float previous_input;      // 上次输入值
        float output;              // 当前输出值
        bool is_initialized;       // 是否已初始化
        bool has_previous_sample;  // 是否有上次采样
    } alg_filter_high_pass_t;

    /* ======================== 指数移动平均滤波器 ======================== */

    /**
     * @brief 指数移动平均滤波器实例
     * @note 实现：y = y + alpha * (x - y)，alpha ∈ (0, 1]
     *       alpha 越大，响应越快，平滑越少
     */
    typedef struct
    {
        float smoothing_factor;   // 平滑因子（0~1），越大响应越快
        float output;             // 当前输出值
        bool is_initialized;      // 是否已初始化
        bool has_previous_sample; // 是否有上次采样
    } alg_filter_exponential_t;

    /* ======================== 滑动平均滤波器 ======================== */

    /**
     * @brief 滑动平均滤波器实例（使用调用者提供的存储）
     * @note 维护固定大小窗口的均值，使用环形缓冲区
     */
    typedef struct
    {
        float *sample_buffer; // 样本缓冲区（调用者分配）
        size_t capacity;      // 缓冲区容量
        size_t sample_count;  // 当前有效样本数
        size_t write_index;   // 写入位置
        float sum;            // 窗口内样本和
        bool is_initialized;  // 是否已初始化
    } alg_filter_moving_average_t;

    /* ======================== 中值滤波器 ======================== */

    /**
     * @brief 中值滤波器实例（使用调用者提供的样本和工作区）
     * @note 需要两个等长缓冲区：一个保存窗口样本，一个作为排序工作区
     */
    typedef struct
    {
        float *sample_buffer; // 样本缓冲区（调用者分配）
        float *sort_buffer;   // 排序工作区（调用者分配）
        size_t capacity;      // 缓冲区容量
        size_t sample_count;  // 当前有效样本数
        size_t write_index;   // 写入位置
        bool is_initialized;  // 是否已初始化
    } alg_filter_median_t;

    /* ======================== 通用 FIR 滤波器 ======================== */

    /**
     * @brief 通用有限脉冲响应（FIR）滤波器实例
     * @note coefficients[0] 对应当前输入，coefficients[1] 对应前一个输入，以此类推
     *       系数数组和状态数组长度必须等于 tap_count
     */
    typedef struct
    {
        const float *coefficients; // FIR 系数数组（调用者提供）
        float *state_buffer;       // 状态缓冲区（调用者提供）
        size_t tap_count;          // 抽头数
        size_t write_index;        // 写入位置
        bool is_initialized;       // 是否已初始化
    } alg_filter_fir_t;

    /* ======================== Biquad 滤波器 ======================== */

    /** @brief 支持的二阶 Biquad 响应类型 */
    typedef enum
    {
        ALG_FILTER_BIQUAD_LOW_PASS = 0, // 低通
        ALG_FILTER_BIQUAD_HIGH_PASS,    // 高通
        ALG_FILTER_BIQUAD_BAND_PASS,    // 带通
        ALG_FILTER_BIQUAD_NOTCH         // 陷波（带阻）
    } alg_filter_biquad_type_t;

    /**
     * @brief 直接 II 型转置 Biquad 滤波器实例
     * @note 传递函数：H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
     *       使用转置直接 II 型结构，数值稳定性好
     */
    typedef struct
    {
        float b0;            // 分子系数 b0
        float b1;            // 分子系数 b1
        float b2;            // 分子系数 b2
        float a1;            // 分母系数 a1（注意符号：H(z) 分母为 1 + a1*z^-1 + a2*z^-2）
        float a2;            // 分母系数 a2
        float state_1;       // 状态变量 1
        float state_2;       // 状态变量 2
        bool is_initialized; // 是否已初始化
    } alg_filter_biquad_t;

    /* ======================== 互补滤波器 ======================== */

    /**
     * @brief 互补滤波器（融合测量值和变化率）
     * @note 实现：output = weight * (output + rate*dt) + (1-weight) * measured
     *       适用于 IMU 姿态融合（加速度计 + 陀螺仪）等场景
     */
    typedef struct
    {
        float prediction_weight; // 预测权重（0~1），值越大越信任积分预测
        float output;            // 当前输出值
        bool is_initialized;     // 是否已初始化
    } alg_filter_complementary_t;

    /* ======================== 低通滤波器 API ======================== */

    alg_filter_status_t alg_filter_low_pass_init(alg_filter_low_pass_t *me,
                                                 float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_low_pass_set_cutoff(alg_filter_low_pass_t *me,
                                                       float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_low_pass_reset(alg_filter_low_pass_t *me, float initial_output);
    alg_filter_status_t alg_filter_low_pass_update(alg_filter_low_pass_t *me, float input,
                                                   float delta_time_s, float *output);

    /* ======================== 高通滤波器 API ======================== */

    alg_filter_status_t alg_filter_high_pass_init(alg_filter_high_pass_t *me,
                                                  float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_high_pass_set_cutoff(alg_filter_high_pass_t *me,
                                                        float cutoff_frequency_hz);
    alg_filter_status_t alg_filter_high_pass_reset(alg_filter_high_pass_t *me, float initial_input);
    alg_filter_status_t alg_filter_high_pass_update(alg_filter_high_pass_t *me, float input,
                                                    float delta_time_s, float *output);

    /* ======================== 指数平均滤波器 API ======================== */

    alg_filter_status_t alg_filter_exponential_init(alg_filter_exponential_t *me,
                                                    float smoothing_factor);
    alg_filter_status_t alg_filter_exponential_set_factor(alg_filter_exponential_t *me,
                                                          float smoothing_factor);
    alg_filter_status_t alg_filter_exponential_reset(alg_filter_exponential_t *me,
                                                     float initial_output);
    alg_filter_status_t alg_filter_exponential_update(alg_filter_exponential_t *me, float input,
                                                      float *output);

    /* ======================== 滑动平均滤波器 API ======================== */

    alg_filter_status_t alg_filter_moving_average_init(alg_filter_moving_average_t *me,
                                                       float *sample_buffer, size_t capacity);
    alg_filter_status_t alg_filter_moving_average_reset(alg_filter_moving_average_t *me);
    alg_filter_status_t alg_filter_moving_average_update(alg_filter_moving_average_t *me,
                                                         float input, float *output);

    /* ======================== 中值滤波器 API ======================== */

    alg_filter_status_t alg_filter_median_init(alg_filter_median_t *me, float *sample_buffer,
                                               float *sort_buffer, size_t capacity);
    alg_filter_status_t alg_filter_median_reset(alg_filter_median_t *me);
    alg_filter_status_t alg_filter_median_update(alg_filter_median_t *me, float input,
                                                 float *output);

    /* ======================== FIR 滤波器 API ======================== */

    alg_filter_status_t alg_filter_fir_init(alg_filter_fir_t *me, const float *coefficients,
                                            float *state_buffer, size_t tap_count);
    alg_filter_status_t alg_filter_fir_reset(alg_filter_fir_t *me);
    alg_filter_status_t alg_filter_fir_update(alg_filter_fir_t *me, float input, float *output);

    /* ======================== Biquad 滤波器 API ======================== */

    alg_filter_status_t alg_filter_biquad_init(alg_filter_biquad_t *me,
                                               alg_filter_biquad_type_t type,
                                               float sample_frequency_hz, float center_frequency_hz,
                                               float quality_factor);
    alg_filter_status_t alg_filter_biquad_set_coefficients(alg_filter_biquad_t *me, float b0,
                                                           float b1, float b2, float a1, float a2);
    alg_filter_status_t alg_filter_biquad_reset(alg_filter_biquad_t *me);
    alg_filter_status_t alg_filter_biquad_update(alg_filter_biquad_t *me, float input,
                                                 float *output);

    /* ======================== 互补滤波器 API ======================== */

    alg_filter_status_t alg_filter_complementary_init(alg_filter_complementary_t *me,
                                                      float prediction_weight,
                                                      float initial_output);
    alg_filter_status_t alg_filter_complementary_set_weight(alg_filter_complementary_t *me,
                                                            float prediction_weight);
    alg_filter_status_t alg_filter_complementary_reset(alg_filter_complementary_t *me,
                                                       float initial_output);
    alg_filter_status_t alg_filter_complementary_update(alg_filter_complementary_t *me,
                                                        float measured_value,
                                                        float measured_rate_per_s,
                                                        float delta_time_s, float *output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_FILTER_H */