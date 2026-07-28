/**
 * @file alg_kalman.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用卡尔曼滤波库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 卡尔曼滤波库，不依赖 HAL、CMSIS 或 RTOS。
 *       不使用动态内存，不读取系统时钟。
 *       支持标量卡尔曼、线性卡尔曼和扩展卡尔曼（EKF）三种算法。
 *       所有矩阵使用行优先连续存储，工作区由调用者静态提供。
 */

#ifndef ALG_KALMAN_H
#define ALG_KALMAN_H

#include <stdbool.h> // bool 类型
#include <stddef.h>  // size_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 工作区大小宏 ======================== */

/**
 * @brief 计算卡尔曼滤波器工作区所需的浮点元素数量
 * @param state_dimension 状态维度 n
 * @param measurement_dimension 测量维度 m
 * @return 所需 float 数组元素数量
 * @note 工作区用于存储中间计算结果，避免动态内存分配
 *       公式：n + 3*n*n + 4*n*m + 2*m + 2*m*m
 */
#define ALG_KALMAN_WORKSPACE_SIZE(state_dimension, measurement_dimension)                          \
    ((state_dimension) + (3U * (state_dimension) * (state_dimension)) +                            \
     (4U * (state_dimension) * (measurement_dimension)) + (2U * (measurement_dimension)) +         \
     (2U * (measurement_dimension) * (measurement_dimension)))

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 卡尔曼滤波库状态码
     */
    typedef enum
    {
        ALG_KALMAN_STATUS_OK = 0,                 // 操作成功
        ALG_KALMAN_STATUS_INVALID_ARGUMENT,       // 参数非法（空指针等）
        ALG_KALMAN_STATUS_OUT_OF_RANGE,           // 参数超出范围（维度为零、非有限数等）
        ALG_KALMAN_STATUS_INSUFFICIENT_WORKSPACE, // 工作区不足
        ALG_KALMAN_STATUS_NOT_INITIALIZED,        // 对象未初始化
        ALG_KALMAN_STATUS_SINGULAR_MATRIX,        // 矩阵奇异（无法求逆）
        ALG_KALMAN_STATUS_MODEL_ERROR,            // 模型函数错误（EKF 回调返回错误）
        ALG_KALMAN_STATUS_NUMERICAL_ERROR         // 数值错误（溢出、非有限结果）
    } alg_kalman_status_t;

    /* ======================== 标量卡尔曼滤波器 ======================== */

    /**
     * @brief 标量卡尔曼滤波器实例
     * @note 适用于单个传感器量的低成本递推估计
     *       模型：x = x + state_delta（预测步）
     *       z = x + v（观测步）
     */
    typedef struct
    {
        float process_noise;     // 过程噪声方差 Q
        float measurement_noise; // 测量噪声方差 R
        float estimate;          // 当前状态估计值
        float covariance;        // 当前估计协方差 P
        float gain;              // 卡尔曼增益 K
        bool is_initialized;     // 是否已初始化
    } alg_kalman_scalar_t;

    /* ======================== 线性卡尔曼滤波器 ======================== */

    /**
     * @brief 线性卡尔曼滤波器配置
     * @note 模型：x = F*x + B*u，z = H*x + v
     *       所有矩阵由调用者提供，滤波器只保存指针，不复制矩阵
     *       矩阵在滤波器生命周期内必须保持有效
     */
    typedef struct
    {
        size_t state_dimension;          // 状态维度 n
        size_t measurement_dimension;    // 测量维度 m
        size_t control_dimension;        // 控制输入维度 c（0 表示无控制输入）
        float *state;                    // 状态向量 x（n×1）
        float *covariance;               // 协方差矩阵 P（n×n）
        const float *transition_matrix;  // 状态转移矩阵 F（n×n）
        const float *control_matrix;     // 控制矩阵 B（n×c），c=0 时可为 NULL
        const float *process_noise;      // 过程噪声矩阵 Q（n×n）
        const float *measurement_matrix; // 观测矩阵 H（m×n）
        const float *measurement_noise;  // 测量噪声矩阵 R（m×m）
        float *workspace;                // 工作区（调用者分配）
        size_t workspace_size;           // 工作区大小（元素数量）
    } alg_kalman_linear_config_t;

    /**
     * @brief 线性卡尔曼滤波器实例
     */
    typedef struct
    {
        alg_kalman_linear_config_t config; // 配置（包含所有矩阵指针）
        bool is_initialized;               // 是否已初始化
    } alg_kalman_linear_t;

    /* ======================== 扩展卡尔曼滤波器（EKF）回调类型 ======================== */

    /**
     * @brief EKF 状态转移函数回调
     * @param state 当前状态（n×1）
     * @param state_dimension 状态维度
     * @param control_input 控制输入（c×1），c=0 时可 NULL
     * @param control_dimension 控制维度
     * @param delta_time_s 时间步长（秒）
     * @param predicted_state 输出预测状态（n×1）
     * @param user_context 用户上下文
     * @return 卡尔曼状态码
     * @note 实现非线性状态转移：x_pred = f(x, u, dt)
     */
    typedef alg_kalman_status_t (*alg_kalman_state_function_t)(
        const float *state, size_t state_dimension, const float *control_input,
        size_t control_dimension, float delta_time_s, float *predicted_state, void *user_context);

    /**
     * @brief EKF 状态雅可比回调
     * @param state 当前状态
     * @param state_dimension 状态维度
     * @param control_input 控制输入
     * @param control_dimension 控制维度
     * @param delta_time_s 时间步长
     * @param state_jacobian 输出雅可比矩阵 F = ∂f/∂x（n×n）
     * @param user_context 用户上下文
     * @return 卡尔曼状态码
     */
    typedef alg_kalman_status_t (*alg_kalman_state_jacobian_function_t)(
        const float *state, size_t state_dimension, const float *control_input,
        size_t control_dimension, float delta_time_s, float *state_jacobian, void *user_context);

    /**
     * @brief EKF 测量函数回调
     * @param state 当前状态
     * @param state_dimension 状态维度
     * @param measurement_dimension 测量维度
     * @param predicted_measurement 输出预测测量值（m×1）
     * @param user_context 用户上下文
     * @return 卡尔曼状态码
     * @note 实现非线性观测：z_pred = h(x)
     */
    typedef alg_kalman_status_t (*alg_kalman_measurement_function_t)(const float *state,
                                                                     size_t state_dimension,
                                                                     size_t measurement_dimension,
                                                                     float *predicted_measurement,
                                                                     void *user_context);

    /**
     * @brief EKF 测量雅可比回调
     * @param state 当前状态
     * @param state_dimension 状态维度
     * @param measurement_dimension 测量维度
     * @param measurement_jacobian 输出雅可比矩阵 H = ∂h/∂x（m×n）
     * @param user_context 用户上下文
     * @return 卡尔曼状态码
     */
    typedef alg_kalman_status_t (*alg_kalman_measurement_jacobian_function_t)(
        const float *state, size_t state_dimension, size_t measurement_dimension,
        float *measurement_jacobian, void *user_context);

    /* ======================== 扩展卡尔曼滤波器配置 ======================== */

    /**
     * @brief 扩展卡尔曼滤波器配置
     * @note 需要调用者提供四个模型回调函数
     *       状态和协方差由滤波器管理（指针指向调用者数组）
     */
    typedef struct
    {
        size_t state_dimension;                                       // 状态维度 n
        size_t measurement_dimension;                                 // 测量维度 m
        size_t control_dimension;                                     // 控制维度 c（0 表示无控制）
        float *state;                                                 // 状态向量（n×1）
        float *covariance;                                            // 协方差矩阵（n×n）
        const float *process_noise;                                   // 过程噪声矩阵 Q（n×n）
        const float *measurement_noise;                               // 测量噪声矩阵 R（m×m）
        float *workspace;                                             // 工作区
        size_t workspace_size;                                        // 工作区大小
        alg_kalman_state_function_t state_function;                   // 状态转移函数
        alg_kalman_state_jacobian_function_t state_jacobian_function; // 状态雅可比
        alg_kalman_measurement_function_t measurement_function;       // 测量函数
        alg_kalman_measurement_jacobian_function_t measurement_jacobian_function; // 测量雅可比
        void *user_context; // 用户上下文（传递给回调）
    } alg_kalman_extended_config_t;

    /**
     * @brief 扩展卡尔曼滤波器实例
     */
    typedef struct
    {
        alg_kalman_extended_config_t config;
        bool is_initialized;
    } alg_kalman_extended_t;

    /* ======================== 标量卡尔曼 API ======================== */

    alg_kalman_status_t alg_kalman_scalar_init(alg_kalman_scalar_t *me, float process_noise,
                                               float measurement_noise, float initial_estimate,
                                               float initial_covariance);
    alg_kalman_status_t alg_kalman_scalar_set_noise(alg_kalman_scalar_t *me, float process_noise,
                                                    float measurement_noise);
    alg_kalman_status_t alg_kalman_scalar_reset(alg_kalman_scalar_t *me, float initial_estimate,
                                                float initial_covariance);
    alg_kalman_status_t alg_kalman_scalar_predict(alg_kalman_scalar_t *me, float state_delta);
    alg_kalman_status_t alg_kalman_scalar_correct(alg_kalman_scalar_t *me, float measurement,
                                                  float *output);
    alg_kalman_status_t alg_kalman_scalar_update(alg_kalman_scalar_t *me, float measurement,
                                                 float *output);

    /* ======================== 线性卡尔曼 API ======================== */

    alg_kalman_status_t alg_kalman_linear_init(alg_kalman_linear_t *me,
                                               const alg_kalman_linear_config_t *config);
    alg_kalman_status_t alg_kalman_linear_reset(alg_kalman_linear_t *me, const float *initial_state,
                                                const float *initial_covariance);
    alg_kalman_status_t alg_kalman_linear_predict(alg_kalman_linear_t *me,
                                                  const float *control_input);
    alg_kalman_status_t alg_kalman_linear_correct(alg_kalman_linear_t *me,
                                                  const float *measurement);
    const float *alg_kalman_linear_get_state(const alg_kalman_linear_t *me);
    const float *alg_kalman_linear_get_covariance(const alg_kalman_linear_t *me);

    /* ======================== 扩展卡尔曼 API ======================== */

    alg_kalman_status_t alg_kalman_extended_init(alg_kalman_extended_t *me,
                                                 const alg_kalman_extended_config_t *config);
    alg_kalman_status_t alg_kalman_extended_reset(alg_kalman_extended_t *me,
                                                  const float *initial_state,
                                                  const float *initial_covariance);
    alg_kalman_status_t alg_kalman_extended_predict(alg_kalman_extended_t *me,
                                                    const float *control_input, float delta_time_s);
    alg_kalman_status_t alg_kalman_extended_correct(alg_kalman_extended_t *me,
                                                    const float *measurement);
    const float *alg_kalman_extended_get_state(const alg_kalman_extended_t *me);
    const float *alg_kalman_extended_get_covariance(const alg_kalman_extended_t *me);

#ifdef __cplusplus
}
#endif

#endif /* ALG_KALMAN_H */