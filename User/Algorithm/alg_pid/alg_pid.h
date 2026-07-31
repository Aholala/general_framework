/**
 * @file alg_pid.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用 PID 控制算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       提供位置式、增量式、串级、增益调度和模糊自适应 PID。
 *       支持二自由度、微分先行、抗积分饱和、积分分离、死区、前馈等。
 *       所有数据由调用者管理，不使用动态内存。
 *       每个控制器实例独立状态，支持多实例运行。
 */

#ifndef ALG_PID_H
#define ALG_PID_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief PID 控制器库状态码
     */
    typedef enum
    {
        ALG_PID_STATUS_OK = 0,           // 操作成功
        ALG_PID_STATUS_INVALID_ARGUMENT, // 参数非法（空指针等）
        ALG_PID_STATUS_OUT_OF_RANGE,     // 参数超出范围（NaN、Inf、非法限幅等）
        ALG_PID_STATUS_NOT_INITIALIZED,  // 控制器未初始化
        ALG_PID_STATUS_NUMERICAL_ERROR   // 数值错误（溢出、非有限结果）
    } alg_pid_status_t;

    /**
     * @brief 抗积分饱和模式
     */
    typedef enum
    {
        ALG_PID_ANTI_WINDUP_NONE = 0,        // 无抗积分饱和（积分项自由累加）
        ALG_PID_ANTI_WINDUP_CLAMPING,        // 条件积分：饱和且误差同向时暂停积分
        ALG_PID_ANTI_WINDUP_BACK_CALCULATION // 反算：通过输出差值反馈修正积分项
    } alg_pid_anti_windup_t;

    /**
     * @brief 微分作用模式
     */
    typedef enum
    {
        ALG_PID_DERIVATIVE_ON_ERROR = 0,  // 对误差微分（传统方式，对设定值阶跃敏感）
        ALG_PID_DERIVATIVE_ON_MEASUREMENT // 对测量值微分（微分先行，避免设定值冲击）
    } alg_pid_derivative_mode_t;

    /* ======================== 位置式 PID 配置 ======================== */

    /**
     * @brief 位置式 PID 控制器配置结构体
     * @note 所有增益和限幅均为连续时间形式。
     *       微分滤波截止频率为 0 时表示不滤波。
     */
    typedef struct
    {
        float proportional_gain;             // 比例增益 Kp
        float integral_gain;                 // 积分增益 Ki（连续时间形式）
        float derivative_gain;               // 微分增益 Kd（连续时间形式）
        float setpoint_weight;               // 比例项设定值权重 β（0~1），用于二自由度
        float derivative_setpoint_weight;    // 微分项设定值权重 γ（0~1），用于二自由度
        float velocity_feedforward_gain;     // 速度前馈增益 Kvff
        float acceleration_feedforward_gain; // 加速度前馈增益 Kaff
        float derivative_filter_cutoff_hz;   // 微分低通截止频率（Hz），0 表示不滤波
        float error_deadband;                // 误差死区（绝对值小于此值视为零）
        float integral_separation_threshold; // 积分分离阈值（误差绝对值大于此值暂停积分，0
                                             // 表示始终积分）
        float integral_min;                        // 积分项下限
        float integral_max;                        // 积分项上限
        float output_min;                          // 输出下限
        float output_max;                          // 输出上限（必须 > output_min）
        float back_calculation_gain;               // 反算抗饱和增益（仅用于 BACK_CALCULATION 模式）
        alg_pid_anti_windup_t anti_windup_mode;    // 抗积分饱和模式
        alg_pid_derivative_mode_t derivative_mode; // 微分作用模式
    } alg_pid_config_t;

    /**
     * @brief 位置式 PID 更新输入（高级接口）
     * @note 包含设定值、测量值、设定值导数/二阶导数、外部前馈和时间步长。
     */
    typedef struct
    {
        float setpoint;                     // 设定值
        float measurement;                  // 测量值（反馈）
        float setpoint_rate_per_s;          // 设定值变化率（用于前馈）
        float setpoint_acceleration_per_s2; // 设定值加速度（用于前馈）
        float additional_feedforward;       // 额外前馈（如重力补偿）
        float delta_time_s;                 // 时间步长（秒，>0）
    } alg_pid_input_t;

    /**
     * @brief PID 各项分量（用于调试和监控）
     */
    typedef struct
    {
        float proportional;       // 比例项
        float integral;           // 积分项
        float derivative;         // 微分项
        float feedforward;        // 前馈项（含速度、加速度和额外前馈）
        float unsaturated_output; // 未限幅的输出（P+I+D+FF）
        float output;             // 限幅后的最终输出
    } alg_pid_terms_t;

    /**
     * @brief 位置式 PID 控制器实例
     */
    typedef struct
    {
        alg_pid_config_t config;    // 配置
        alg_pid_terms_t terms;      // 当前各项值
        float previous_error;       // 上一步误差（用于微分计算）
        float previous_setpoint;    // 上一步设定值（用于微分先行）
        float previous_measurement; // 上一步测量值（用于微分先行）
        float filtered_derivative;  // 滤波后的微分值
        bool has_previous_sample;   // 是否有上一步样本（首次更新时无）
        bool is_initialized;        // 是否已初始化
    } alg_pid_t;

    /**
     * @brief 位置式 PID 控制器（位置环语义类型）
     */
    typedef alg_pid_t alg_pid_position_t;

    /**
     * @brief 位置式 PID 控制器（速度环语义类型）
     */
    typedef alg_pid_t alg_pid_velocity_t;

    /* ======================== 位置式 PID API ======================== */

    /**
     * @brief 初始化配置结构体为默认值
     * @param config  配置指针
     * @return 执行状态
     * @note 默认：所有增益为零，设定值权重=1，微分设定值权重=0，输出和积分限幅无限，
     *       微分滤波关闭，死区/积分分离关闭，抗饱和模式为 CLAMPING，微分模式为 ON_MEASUREMENT。
     */
    alg_pid_status_t alg_pid_config_init(alg_pid_config_t *config);

    /**
     * @brief 初始化位置式 PID 控制器
     * @param me     控制器对象
     * @param config 配置（将被复制到对象中）
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_init(alg_pid_t *me, const alg_pid_config_t *config);

    /**
     * @brief 初始化位置环 PID（语义别名）
     */
    alg_pid_status_t alg_pid_position_init(alg_pid_position_t *me, const alg_pid_config_t *config);

    /**
     * @brief 初始化速度环 PID（语义别名）
     */
    alg_pid_status_t alg_pid_velocity_init(alg_pid_velocity_t *me, const alg_pid_config_t *config);

    /**
     * @brief 运行时更新控制器配置
     * @param me     控制器对象
     * @param config 新配置
     * @return 执行状态
     * @note 会重新限幅当前积分和输出项以适应新限幅。
     */
    alg_pid_status_t alg_pid_set_config(alg_pid_t *me, const alg_pid_config_t *config);

    /**
     * @brief 重置控制器状态
     * @param me             控制器对象
     * @param measurement    当前测量值（用于初始化前一次值）
     * @param initial_output 初始输出值（用于设置积分项初始值）
     * @return 执行状态
     * @note 将积分项设为 initial_output（限幅后），清除历史数据。
     */
    alg_pid_status_t alg_pid_reset(alg_pid_t *me, float measurement, float initial_output);

    /**
     * @brief 输出跟踪（无扰切换）
     * @param me              控制器对象
     * @param setpoint        当前设定值
     * @param measurement     当前测量值
     * @param feedforward     当前前馈项
     * @param tracked_output  目标输出值（如手动模式下的输出）
     * @return 执行状态
     * @note 反算积分项使 PID 输出尽量接近 tracked_output，实现无扰切换。
     */
    alg_pid_status_t alg_pid_track_output(alg_pid_t *me, float setpoint, float measurement,
                                          float feedforward, float tracked_output);

    /**
     * @brief 简单更新接口（不含前馈和设定值导数）
     * @param me             控制器对象
     * @param setpoint       设定值
     * @param measurement    测量值
     * @param delta_time_s   时间步长（秒）
     * @param output         输出指针
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_update(alg_pid_t *me, float setpoint, float measurement,
                                    float delta_time_s, float *output);

    /**
     * @brief 高级更新接口（含前馈和设定值导数）
     * @param me      控制器对象
     * @param input   输入结构体（包含设定值、测量值、前馈等）
     * @param output  输出指针
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_update_advanced(alg_pid_t *me, const alg_pid_input_t *input,
                                             float *output);

    /**
     * @brief 获取当前各项分量（用于调试）
     * @param me  控制器对象
     * @return 指向 terms 的指针，未初始化则返回 NULL
     */
    const alg_pid_terms_t *alg_pid_get_terms(const alg_pid_t *me);

    /* ======================== 增量式 PID ======================== */

    /**
     * @brief 增量式 PID 配置结构体
     */
    typedef struct
    {
        float proportional_gain;           // 比例增益 Kp
        float integral_gain;               // 积分增益 Ki（连续时间形式）
        float derivative_gain;             // 微分增益 Kd（连续时间形式）
        float derivative_filter_cutoff_hz; // 微分增量低通截止频率（Hz），0 表示不滤波
        float error_deadband;              // 误差死区
        float delta_output_min;            // 单周期输出增量下限
        float delta_output_max;            // 单周期输出增量上限
        float output_min;                  // 总输出下限
        float output_max;                  // 总输出上限
    } alg_pid_incremental_config_t;

    /**
     * @brief 增量式 PID 控制器实例
     */
    typedef struct
    {
        alg_pid_incremental_config_t config; // 配置
        alg_pid_terms_t terms; // 各项增量（proportional/integral/derivative/feedforward 均为增量）
        float previous_error;  // e(k-1)
        float second_previous_error;     // e(k-2)
        float filtered_derivative_delta; // 滤波后的微分增量
        bool has_previous_sample;        // 是否有上一步样本
        bool is_initialized;             // 是否已初始化
    } alg_pid_incremental_t;

    /* ======================== 增量式 PID API ======================== */

    /**
     * @brief 初始化增量式 PID 配置为默认值
     */
    alg_pid_status_t alg_pid_incremental_config_init(alg_pid_incremental_config_t *config);

    /**
     * @brief 初始化增量式 PID 控制器
     */
    alg_pid_status_t alg_pid_incremental_init(alg_pid_incremental_t *me,
                                              const alg_pid_incremental_config_t *config);

    /**
     * @brief 运行时更新增量式 PID 配置
     */
    alg_pid_status_t alg_pid_incremental_set_config(alg_pid_incremental_t *me,
                                                    const alg_pid_incremental_config_t *config);

    /**
     * @brief 重置增量式 PID 控制器
     * @param me             控制器对象
     * @param initial_output 当前输出值（用于设置初始累积输出）
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_incremental_reset(alg_pid_incremental_t *me, float initial_output);

    /**
     * @brief 增量式 PID 更新
     * @param me                控制器对象
     * @param setpoint          设定值
     * @param measurement       测量值
     * @param feedforward_delta 前馈增量（外部前馈的变化量）
     * @param delta_time_s      时间步长（秒）
     * @param output            输出指针
     * @return 执行状态
     * @note 输出为累积值。前馈增量用于外部前馈的变化。
     */
    alg_pid_status_t alg_pid_incremental_update(alg_pid_incremental_t *me, float setpoint,
                                                float measurement, float feedforward_delta,
                                                float delta_time_s, float *output);

    /**
     * @brief 获取增量式 PID 各项增量（用于调试）
     */
    const alg_pid_terms_t *alg_pid_incremental_get_terms(const alg_pid_incremental_t *me);

    /* ======================== 增益调度 PID ======================== */

    /**
     * @brief 增益调度点
     */
    typedef struct
    {
        float operating_point;   // 工作点（必须严格递增）
        float proportional_gain; // 该点的 Kp
        float integral_gain;     // 该点的 Ki
        float derivative_gain;   // 该点的 Kd
    } alg_pid_gain_point_t;

    /**
     * @brief 增益调度 PID 控制器实例
     * @note 根据工作点线性插值 Kp、Ki、Kd，内部包含一个位置式 PID 控制器。
     */
    typedef struct
    {
        alg_pid_t controller;                    // 内部 PID 控制器
        const alg_pid_gain_point_t *gain_points; // 增益表（外部持有）
        size_t gain_point_count;                 // 增益点数量
        bool is_initialized;                     // 是否已初始化
    } alg_pid_gain_schedule_t;

    /* ======================== 增益调度 PID API ======================== */

    /**
     * @brief 初始化增益调度 PID
     * @param me              控制器对象
     * @param base_config     基础配置（其他参数如限幅、前馈等从此复制）
     * @param gain_points     增益表（必须按工作点递增）
     * @param gain_point_count 增益点数量
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_gain_schedule_init(alg_pid_gain_schedule_t *me,
                                                const alg_pid_config_t *base_config,
                                                const alg_pid_gain_point_t *gain_points,
                                                size_t gain_point_count);

    /**
     * @brief 增益调度 PID 更新
     * @param me             控制器对象
     * @param operating_point 当前工作点
     * @param input          PID 输入
     * @param output         输出指针
     * @return 执行状态
     */
    alg_pid_status_t alg_pid_gain_schedule_update(alg_pid_gain_schedule_t *me,
                                                  float operating_point,
                                                  const alg_pid_input_t *input, float *output);

    /**
     * @brief 重置增益调度 PID
     */
    alg_pid_status_t alg_pid_gain_schedule_reset(alg_pid_gain_schedule_t *me, float measurement,
                                                 float initial_output);

    /* ======================== 模糊自适应 PID ======================== */

    /**
     * @brief 模糊自适应 PID 配置
     * @note 通过二维规则表对 Kp、Ki、Kd 进行在线调整。
     *       规则表为正方形，行对应归一化误差（-1~1），列对应归一化误差变化率（-1~1），
     *       值为调整量（可正可负）。
     */
    typedef struct
    {
        alg_pid_config_t base_config; // 基础配置（增益之外的参数）
        const float
            *proportional_adjustment_table; // Kp 调整量表（axis_point_count × axis_point_count）
        const float *integral_adjustment_table;   // Ki 调整量表
        const float *derivative_adjustment_table; // Kd 调整量表
        size_t axis_point_count;                  // 每个轴上的点数（>=2）
        float error_normalization;                // 误差归一化因子（>0）
        float error_rate_normalization;           // 误差变化率归一化因子（>0）
    } alg_pid_fuzzy_config_t;

    /**
     * @brief 模糊自适应 PID 控制器实例
     */
    typedef struct
    {
        alg_pid_t controller;          // 内部 PID 控制器
        alg_pid_fuzzy_config_t config; // 配置
        float previous_error;          // 上一步误差
        bool has_previous_sample;      // 是否有上一步样本
        bool is_initialized;           // 是否已初始化
    } alg_pid_fuzzy_t;

    /* ======================== 模糊自适应 PID API ======================== */

    /**
     * @brief 初始化模糊自适应 PID
     */
    alg_pid_status_t alg_pid_fuzzy_init(alg_pid_fuzzy_t *me, const alg_pid_fuzzy_config_t *config);

    /**
     * @brief 重置模糊自适应 PID
     */
    alg_pid_status_t alg_pid_fuzzy_reset(alg_pid_fuzzy_t *me, float measurement,
                                         float initial_output);

    /**
     * @brief 模糊自适应 PID 更新
     * @note 根据当前误差和误差变化率查表调整 Kp、Ki、Kd，然后调用内部 PID 更新。
     */
    alg_pid_status_t alg_pid_fuzzy_update(alg_pid_fuzzy_t *me, const alg_pid_input_t *input,
                                          float *output);

    /* ======================== 串级 PID ======================== */

    /**
     * @brief 串级 PID 配置
     * @note 内含位置环（外环）和速度环（内环）两个 PID 控制器。
     */
    typedef struct
    {
        alg_pid_config_t position_config; // 位置环 PID 配置
        alg_pid_config_t velocity_config; // 速度环 PID 配置
        uint32_t position_loop_divider;   // 位置环降频系数（速度环频率 / 位置环频率，>=1）
        float velocity_setpoint_min;      // 速度环设定值下限
        float velocity_setpoint_max;      // 速度环设定值上限（必须 > min）
    } alg_pid_cascade_config_t;

    /**
     * @brief 串级 PID 输入
     */
    typedef struct
    {
        float position_setpoint;    // 位置设定值
        float position_measurement; // 位置测量值
        float velocity_measurement; // 速度测量值
        float velocity_feedforward; // 速度前馈（叠加到位置环输出）
        float actuator_feedforward; // 执行器前馈（叠加到速度环输出）
        float delta_time_s;         // 时间步长（秒，>0）
    } alg_pid_cascade_input_t;

    /**
     * @brief 串级 PID 控制器实例
     */
    typedef struct
    {
        alg_pid_position_t position_controller; // 位置环控制器
        alg_pid_velocity_t velocity_controller; // 速度环控制器
        uint32_t position_loop_divider;         // 降频系数
        uint32_t position_loop_counter;         // 计数器
        float position_elapsed_time_s;          // 累积时间
        float velocity_setpoint_min;            // 速度下限
        float velocity_setpoint_max;            // 速度上限
        float velocity_setpoint;                // 当前速度环设定值（外部可读）
        bool is_initialized;                    // 是否已初始化
    } alg_pid_cascade_t;

    /* ======================== 串级 PID API ======================== */

    /**
     * @brief 初始化串级 PID 控制器
     */
    alg_pid_status_t alg_pid_cascade_init(alg_pid_cascade_t *me,
                                          const alg_pid_cascade_config_t *config);

    /**
     * @brief 重置串级 PID
     */
    alg_pid_status_t alg_pid_cascade_reset(alg_pid_cascade_t *me, float position_measurement,
                                           float velocity_measurement, float initial_output);

    /**
     * @brief 串级 PID 更新
     * @note 位置环按降频系数运行，速度环每周期更新。
     */
    alg_pid_status_t alg_pid_cascade_update(alg_pid_cascade_t *me,
                                            const alg_pid_cascade_input_t *input, float *output);

    /**
     * @brief 获取当前速度环设定值（用于监控）
     */
    float alg_pid_cascade_get_velocity_setpoint(const alg_pid_cascade_t *me);

    /* ======================== 角度串级 PID 封装 ======================== */

    /**
     * @brief 角度串级 PID 配置
     * @note 内部使用 alg_pid_cascade_t，外环控制角度，内环控制角速度。
     */
    typedef struct
    {
        alg_pid_cascade_config_t cascade_config; // 串级 PID 配置
    } alg_pid_angle_config_t;

    /**
     * @brief 角度串级 PID 单次更新输入
     */
    typedef struct
    {
        float target_position_rad;         // 目标角度（rad）
        float target_velocity_rad_per_s;   // 目标角速度前馈（rad/s）
        float measured_position_rad;       // 测量角度（rad）
        float measured_velocity_rad_per_s; // 测量角速度（rad/s）
        float actuator_feedforward;        // 执行器附加前馈
        float delta_time_s;                // 控制周期（s，必须 > 0）
    } alg_pid_angle_input_t;

    /**
     * @brief 角度串级 PID 对象
     */
    typedef struct
    {
        alg_pid_cascade_t cascade; // 位置外环和速度内环
    } alg_pid_angle_t;

    /** @brief 初始化角度串级 PID */
    alg_pid_status_t alg_pid_angle_init(alg_pid_angle_t *me,
                                        const alg_pid_angle_config_t *config);

    /** @brief 用当前角度、角速度和执行器输出重置控制器 */
    alg_pid_status_t alg_pid_angle_reset(alg_pid_angle_t *me, float measured_position_rad,
                                         float measured_velocity_rad_per_s,
                                         float initial_output);

    /** @brief 更新角度串级 PID 输出 */
    alg_pid_status_t alg_pid_angle_update(alg_pid_angle_t *me,
                                          const alg_pid_angle_input_t *input,
                                          float *control_output);

    /** @brief 获取角度外环生成的当前角速度设定值 */
    float alg_pid_angle_get_velocity_setpoint(const alg_pid_angle_t *me);

#ifdef __cplusplus
}
#endif

#endif /* ALG_PID_H */
