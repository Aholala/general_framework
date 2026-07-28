/**
 * @file alg_angle_controller.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 角度控制器多态接口头文件
 * @version 1.0
 * @date 2026-07-25
 * @copyright Copyright (c) 2026
 *
 * @note 角度控制器的多态接口。相同的 alg_angle_controller_t * 可以指向串级 PID
 *       或二维 LQR 控制器，使云台、舵向等上层逻辑不依赖具体闭环算法。
 */

#ifndef ALG_ANGLE_CONTROLLER_H
#define ALG_ANGLE_CONTROLLER_H

#include "alg_lqr.h" // LQR 控制器算法
#include "alg_pid.h" // PID 控制器算法

#include <stdbool.h> // bool 类型

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct alg_angle_controller alg_angle_controller_t;

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 角度控制器状态码
     */
    typedef enum
    {
        ALG_ANGLE_CONTROLLER_STATUS_OK = 0,           // 操作成功
        ALG_ANGLE_CONTROLLER_STATUS_INVALID_ARGUMENT, // 参数非法
        ALG_ANGLE_CONTROLLER_STATUS_NOT_INITIALIZED,  // 对象未初始化
        ALG_ANGLE_CONTROLLER_STATUS_ALGORITHM_ERROR   // 底层算法错误（PID/LQR 初始化失败）
    } alg_angle_controller_status_t;

    /* ======================== 输入结构体 ======================== */

    /**
     * @brief 角度控制器输入
     * @note 全部角度使用弧度，角速度使用弧度/秒
     *       调用方必须先处理编码器跨圈或角度连续化
     */
    typedef struct
    {
        float target_position_rad;         // 目标位置（弧度）
        float target_velocity_rad_per_s;   // 目标速度（rad/s）
        float measured_position_rad;       // 测量位置（弧度）
        float measured_velocity_rad_per_s; // 测量速度（rad/s）
        float actuator_feedforward;        // 执行器前馈
        float delta_time_s;                // 控制周期（秒），必须 > 0
    } alg_angle_controller_input_t;

    /* ======================== 虚操作表 ======================== */

    /**
     * @brief 角度控制器虚操作表
     * @note 派生类必须实现 reset 和 update
     */
    typedef struct
    {
        /**
         * @brief 重置控制器内部状态
         * @param me 控制器基类指针
         * @param measured_position_rad 当前测量位置
         * @param measured_velocity_rad_per_s 当前测量速度
         * @param initial_output 初始输出值
         * @return 执行状态
         */
        alg_angle_controller_status_t (*reset)(alg_angle_controller_t *me,
                                              float measured_position_rad,
                                              float measured_velocity_rad_per_s,
                                              float initial_output);

        /**
         * @brief 更新控制器输出
         * @param me 控制器基类指针
         * @param input 控制器输入
         * @param control_output 输出控制量
         * @return 执行状态
         */
        alg_angle_controller_status_t (*update)(alg_angle_controller_t *me,
                                               const alg_angle_controller_input_t *input,
                                               float *control_output);
    } alg_angle_controller_ops_t;

    /* ======================== 基类结构体 ======================== */

    /**
     * @brief 角度控制器基类
     * @note 派生类必须将 super 作为第一个成员
     */
    struct alg_angle_controller
    {
        const alg_angle_controller_ops_t *vptr; // 虚表指针
        bool is_initialized;                   // 是否已初始化
    };

    /* ======================== PID 派生类 ======================== */

    /**
     * @brief PID 角度控制器配置
     */
    typedef struct
    {
        alg_pid_cascade_config_t cascade_config; // 串级 PID 配置
    } alg_angle_pid_config_t;

    /**
     * @brief PID 角度控制器对象
     * @note 封装 alg_pid_cascade_t，外环位置 → 内环速度
     */
    typedef struct
    {
        alg_angle_controller_t super; // 基类
        alg_pid_cascade_t cascade;   // 串级 PID 控制器
    } alg_angle_pid_t;

    /* ======================== LQR 派生类 ======================== */

    /**
     * @brief LQR 角度控制器配置
     * @note gain_matrix 长度为 2：[Kp, Kd]（位置增益和速度增益）
     *       仅在初始化期间读取，无需长期保存
     */
    typedef struct
    {
        const float *gain_matrix;  // 增益矩阵（2 个元素）
        float control_min;         // 输出下限
        float control_max;         // 输出上限
        float equilibrium_control; // 平衡控制量（稳态输出）
    } alg_angle_lqr_config_t;

    /**
     * @brief LQR 角度控制器对象
     * @note 使用位置误差和速度误差两个状态，增益矩阵长度为 2
     */
    typedef struct
    {
        alg_angle_controller_t super;     // 基类
        alg_lqr_controller_t controller; // LQR 控制器
        float gain_matrix[2];            // 增益矩阵（从配置复制）
        float control_min;               // 输出下限
        float control_max;               // 输出上限
        float equilibrium_control;       // 平衡控制量
    } alg_angle_lqr_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 PID 角度控制器
     * @param me PID 控制器对象
     * @param config 配置参数
     * @return 执行状态
     */
    alg_angle_controller_status_t alg_angle_pid_init(alg_angle_pid_t *me,
                                                   const alg_angle_pid_config_t *config);

    /**
     * @brief 初始化 LQR 角度控制器
     * @param me LQR 控制器对象
     * @param config 配置参数
     * @return 执行状态
     */
    alg_angle_controller_status_t alg_angle_lqr_init(alg_angle_lqr_t *me,
                                                   const alg_angle_lqr_config_t *config);

    /**
     * @brief 将 PID 控制器转为基类指针（向上转型）
     * @param me PID 控制器对象
     * @return 基类指针
     */
    alg_angle_controller_t *alg_angle_pid_as_controller(alg_angle_pid_t *me);

    /**
     * @brief 将 LQR 控制器转为基类指针（向上转型）
     * @param me LQR 控制器对象
     * @return 基类指针
     */
    alg_angle_controller_t *alg_angle_lqr_as_controller(alg_angle_lqr_t *me);

    /**
     * @brief 重置控制器内部状态（多态）
     * @param me 控制器基类指针
     * @param measured_position_rad 当前测量位置
     * @param measured_velocity_rad_per_s 当前测量速度
     * @param initial_output 初始输出值
     * @return 执行状态
     * @note 切换控制器时应先复位新控制器，使状态与当前测量值连续
     */
    alg_angle_controller_status_t alg_angle_controller_reset(alg_angle_controller_t *me,
                                                           float measured_position_rad,
                                                           float measured_velocity_rad_per_s,
                                                           float initial_output);

    /**
     * @brief 更新控制器输出（多态）
     * @param me 控制器基类指针
     * @param input 控制器输入
     * @param control_output 输出控制量
     * @return 执行状态
     */
    alg_angle_controller_status_t
    alg_angle_controller_update(alg_angle_controller_t *me, const alg_angle_controller_input_t *input,
                               float *control_output);

#ifdef __cplusplus
}
#endif

#endif /* ALG_ANGLE_CONTROLLER_H */