/**
 * @file alg_lqr.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 线性二次型调节器（LQR）算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       不使用动态内存，不读取系统时钟。
 *       提供无限时域（DARE）和有限时域 Riccati 求解、LQR 控制器、离散化、LQI 增广等功能。
 *       所有矩阵使用行优先连续存储，工作区由调用者静态提供。
 */

#ifndef ALG_LQR_H
#define ALG_LQR_H

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 工作区大小宏 ======================== */

/**
 * @brief 计算 Riccati 迭代所需工作区浮点元素数量
 * @param state_dimension    状态维度 n
 * @param control_dimension  控制维度 m
 * @return 所需 float 数组元素个数
 * @note 用于 alg_lqr_dare_solve() 或 alg_lqr_finite_solve() 的 workspace。
 *       公式：4*n*n + 6*n*m + 3*m*m。
 */
#define ALG_LQR_RICCATI_WORKSPACE_SIZE(state_dimension, control_dimension)                         \
    ((4U * (state_dimension) * (state_dimension)) +                                                \
     (6U * (state_dimension) * (control_dimension)) +                                              \
     (3U * (control_dimension) * (control_dimension)))

/**
 * @brief 计算有限时域 LQR 求解所需工作区浮点元素数量
 * @param state_dimension    状态维度 n
 * @param control_dimension  控制维度 m
 * @return 所需 float 数组元素个数
 * @note  = n*n + ALG_LQR_RICCATI_WORKSPACE_SIZE(n,m)
 */
#define ALG_LQR_FINITE_WORKSPACE_SIZE(state_dimension, control_dimension)                          \
    (((state_dimension) * (state_dimension)) +                                                     \
     ALG_LQR_RICCATI_WORKSPACE_SIZE((state_dimension), (control_dimension)))

/**
 * @brief 计算 Tustin 离散化所需工作区浮点元素数量
 * @param state_dimension    状态维度 n
 * @param control_dimension  控制维度 m
 * @return 所需 float 数组元素个数
 * @note 需要 3 个 n×n 矩阵和 1 个 n×m 矩阵。
 */
#define ALG_LQR_DISCRETIZE_WORKSPACE_SIZE(state_dimension, control_dimension)                      \
    ((3U * (state_dimension) * (state_dimension)) + ((state_dimension) * (control_dimension)))

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief LQR 算法库状态码
     */
    typedef enum
    {
        ALG_LQR_STATUS_OK = 0,                 // 操作成功
        ALG_LQR_STATUS_INVALID_ARGUMENT,       // 参数非法（空指针、限幅不匹配等）
        ALG_LQR_STATUS_OUT_OF_RANGE,           // 参数超出范围（维度零、非有限数、限幅 min>=max）
        ALG_LQR_STATUS_INSUFFICIENT_WORKSPACE, // 工作区不足
        ALG_LQR_STATUS_NOT_INITIALIZED,        // 控制器未初始化
        ALG_LQR_STATUS_SINGULAR_MATRIX,        // 矩阵奇异（求逆失败）
        ALG_LQR_STATUS_NOT_CONVERGED,          // DARE 迭代未收敛
        ALG_LQR_STATUS_NUMERICAL_ERROR         // 数值错误（溢出、非有限结果）
    } alg_lqr_status_t;

    /* ======================== LQR 控制器 ======================== */

    /**
     * @brief LQR 控制器配置结构体
     * @note 使用增益矩阵 K（m×n），可选限幅。
     *       所有矩阵指针在控制器生命周期内必须保持有效。
     */
    typedef struct
    {
        size_t state_dimension;   // 状态维度 n
        size_t control_dimension; // 控制维度 m
        const float *gain_matrix; // 增益矩阵 K（m×n），行优先
        const float *control_min; // 控制下限（m×1），可为 NULL（不限幅）
        const float *control_max; // 控制上限（m×1），必须与 min 同时 NULL 或同时有效
    } alg_lqr_controller_config_t;

    /**
     * @brief LQR 控制器实例
     */
    typedef struct
    {
        alg_lqr_controller_config_t config; // 配置
        bool is_initialized;                // 是否已初始化
    } alg_lqr_controller_t;

    /* ======================== 控制器 API ======================== */

    /**
     * @brief 初始化 LQR 控制器
     * @param me     控制器对象
     * @param config 配置参数
     * @return 执行状态
     * @note 检查增益矩阵有限性、限幅有效性。
     */
    alg_lqr_status_t alg_lqr_controller_init(alg_lqr_controller_t *me,
                                             const alg_lqr_controller_config_t *config);

    /**
     * @brief 根据当前状态计算控制量
     * @param me                控制器对象
     * @param state             当前状态（n×1）
     * @param reference_state   参考状态（n×1），NULL 表示零
     * @param equilibrium_control 平衡控制输入（m×1），NULL 表示零
     * @param feedforward_control 前馈控制输入（m×1），NULL 表示零
     * @param control_output    输出控制量（m×1）
     * @return 执行状态
     * @note 计算公式：u = u_eq + u_ff - K*(x - x_ref)
     *       限幅在计算后应用。
     */
    alg_lqr_status_t alg_lqr_controller_update(const alg_lqr_controller_t *me, const float *state,
                                               const float *reference_state,
                                               const float *equilibrium_control,
                                               const float *feedforward_control,
                                               float *control_output);

    /* ======================== 无限时域 DARE 求解 ======================== */

    /**
     * @brief 离散代数 Riccati 方程（DARE）求解配置
     * @note 用于迭代求解无限时域 LQR：
     *       K = (R + BᵀPB)⁻¹(BᵀPA + Nᵀ)
     *       P = Q + AᵀPA - (AᵀPB + N)K
     *       默认系统已离散化。
     */
    typedef struct
    {
        size_t state_dimension;      // 状态维度 n
        size_t control_dimension;    // 控制维度 m
        const float *state_matrix;   // A（n×n）
        const float *control_matrix; // B（n×m）
        const float *state_weight;   // Q（n×n），对称半正定
        const float *control_weight; // R（m×m），对称正定
        const float *cross_weight;   // N（n×m），可为 NULL 表示零
        float tolerance;             // 收敛容差（最大元素变化量）
        size_t maximum_iterations;   // 最大迭代次数
        float *workspace;            // 工作区
        size_t workspace_size;       // 工作区大小（float 元素个数）
    } alg_lqr_dare_config_t;

    /**
     * @brief 求解无限时域离散代数 Riccati 方程
     * @param config              配置
     * @param riccati_solution    输出 P（n×n）
     * @param gain_matrix         输出最优增益 K（m×n）
     * @param completed_iterations 输出实际迭代次数，可为 NULL
     * @return 执行状态
     * @note 初始 P 取 Q，迭代直至 max(|P_{k+1} - P_k|) < tolerance。
     *       若达到最大迭代次数仍未收敛，返回 NOT_CONVERGED。
     */
    alg_lqr_status_t alg_lqr_dare_solve(const alg_lqr_dare_config_t *config,
                                        float *riccati_solution, float *gain_matrix,
                                        size_t *completed_iterations);

    /* ======================== 有限时域 LQR 求解 ======================== */

    /**
     * @brief 有限时域 LQR 求解配置
     * @note 代价函数：J = x_NᵀP_f x_N + Σ_{k=0}^{N-1} (x_kᵀQ x_k + 2x_kᵀN u_k + u_kᵀR u_k)
     *       返回每一步的增益 K_k。
     */
    typedef struct
    {
        size_t state_dimension;             // 状态维度 n
        size_t control_dimension;           // 控制维度 m
        size_t horizon_length;              // 时域长度 N
        const float *state_matrix;          // A（n×n）
        const float *control_matrix;        // B（n×m）
        const float *state_weight;          // Q（n×n）
        const float *control_weight;        // R（m×m）
        const float *cross_weight;          // N（n×m），可为 NULL
        const float *terminal_state_weight; // 终端权重 P_f（n×n）
        float *workspace;                   // 工作区
        size_t workspace_size;              // 工作区大小
    } alg_lqr_finite_config_t;

    /**
     * @brief 求解有限时域 LQR，返回完整增益序列
     * @param config                配置
     * @param gain_sequence         输出增益序列，按时间步排列：
     *                              [K_0, K_1, ..., K_{N-1}]，每个 K 为 m×n
     * @param initial_riccati_solution 输出 P_0（n×n），可为 NULL
     * @return 执行状态
     * @note 反向递推从终端权重开始，逐步计算 P_k 和 K_{k-1}。
     *       gain_sequence 需至少有 (horizon_length * m * n) 个元素。
     */
    alg_lqr_status_t alg_lqr_finite_solve(const alg_lqr_finite_config_t *config,
                                          float *gain_sequence, float *initial_riccati_solution);

    /* ======================== 连续模型离散化（Tustin） ======================== */

    /**
     * @brief 使用 Tustin 双线性变换将连续状态空间离散化
     * @param continuous_state_matrix   连续 A（n×n）
     * @param continuous_control_matrix 连续 B（n×m）
     * @param state_dimension           状态维度 n
     * @param control_dimension         控制维度 m
     * @param delta_time_s              采样周期（秒，>0）
     * @param discrete_state_matrix     输出 A_d（n×n）
     * @param discrete_control_matrix   输出 B_d（n×m）
     * @param workspace                 工作区
     * @param workspace_size            工作区大小
     * @return 执行状态
     * @note 公式：
     *       A_d = (I - A*dt/2)⁻¹ (I + A*dt/2)
     *       B_d = (I - A*dt/2)⁻¹ (B*dt)
     *       此方法比前向欧拉稳定性更好。
     */
    alg_lqr_status_t alg_lqr_discretize_tustin(const float *continuous_state_matrix,
                                               const float *continuous_control_matrix,
                                               size_t state_dimension, size_t control_dimension,
                                               float delta_time_s, float *discrete_state_matrix,
                                               float *discrete_control_matrix, float *workspace,
                                               size_t workspace_size);

    /* ======================== LQI 积分增广 ======================== */

    /**
     * @brief 构建 LQI（线性二次积分）增广状态模型
     * @param state_matrix          原 A（n×n）
     * @param control_matrix        原 B（n×m）
     * @param output_matrix         输出矩阵 C（p×n），p 为积分通道数
     * @param state_dimension       原状态维度 n
     * @param control_dimension     控制维度 m
     * @param integral_dimension    积分通道数 p
     * @param delta_time_s          采样周期（秒，>0）
     * @param augmented_state_matrix 输出 A_aug（(n+p)×(n+p)）
     * @param augmented_control_matrix 输出 B_aug（(n+p)×m）
     * @return 执行状态
     * @note 增广状态为 [x; e]，e 为积分误差累加。
     *       构建：A_aug = [[A, 0], [-dt*C, I]]，B_aug = [B; 0]
     *       然后可对增广系统设计 LQR 实现状态反馈 + 积分补偿。
     */
    alg_lqr_status_t alg_lqr_lqi_build_augmented_model(
        const float *state_matrix, const float *control_matrix, const float *output_matrix,
        size_t state_dimension, size_t control_dimension, size_t integral_dimension,
        float delta_time_s, float *augmented_state_matrix, float *augmented_control_matrix);

    /* ======================== 二维角度 LQR 封装 ======================== */

    /**
     * @brief 二维角度 LQR 配置
     * @note gain_matrix 为两个元素：[角度增益, 角速度增益]。
     */
    typedef struct
    {
        const float *gain_matrix;  // 两元素增益矩阵
        float control_min;         // 输出下限
        float control_max;         // 输出上限
        float equilibrium_control; // 平衡控制量
    } alg_lqr_angle_config_t;

    /**
     * @brief 二维角度 LQR 单次更新输入
     */
    typedef struct
    {
        float target_position_rad;         // 目标角度（rad）
        float target_velocity_rad_per_s;   // 目标角速度（rad/s）
        float measured_position_rad;       // 测量角度（rad）
        float measured_velocity_rad_per_s; // 测量角速度（rad/s）
        float actuator_feedforward;        // 执行器附加前馈
        float delta_time_s;                // 控制周期（s，仅用于统一输入校验）
    } alg_lqr_angle_input_t;

    /**
     * @brief 二维角度 LQR 对象
     */
    typedef struct
    {
        alg_lqr_controller_t controller; // 二状态、一输出 LQR 控制器
        float gain_matrix[2];            // 从配置复制的增益
        float control_min;               // 输出下限
        float control_max;               // 输出上限
        float equilibrium_control;       // 平衡控制量
    } alg_lqr_angle_t;

    /** @brief 初始化二维角度 LQR */
    alg_lqr_status_t alg_lqr_angle_init(alg_lqr_angle_t *me,
                                        const alg_lqr_angle_config_t *config);

    /**
     * @brief 校验当前状态并重置二维角度 LQR
     * @note LQR 没有积分状态，此函数用于切换控制器前统一校验当前状态。
     */
    alg_lqr_status_t alg_lqr_angle_reset(alg_lqr_angle_t *me, float measured_position_rad,
                                         float measured_velocity_rad_per_s,
                                         float initial_output);

    /** @brief 更新二维角度 LQR 输出 */
    alg_lqr_status_t alg_lqr_angle_update(const alg_lqr_angle_t *me,
                                          const alg_lqr_angle_input_t *input,
                                          float *control_output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_LQR_H */
