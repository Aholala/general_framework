/**
 * @file alg_swerve.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 任意数量舵轮模块运动学算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       支持任意数量、任意位置布局的舵轮模块。
 *       提供逆运动学（速度→各模块轮速+舵角）、正运动学（轮速+舵角→车体速度）、
 *       舵角最短路径优化、静止自锁、模块失效降级及任意旋转中心。
 *       所有数据由调用者管理，不使用动态内存。
 *       依赖 alg_chassis_motion 模块的加权最小二乘求解器。
 */

#ifndef ALG_SWERVE_H
#define ALG_SWERVE_H

#include "alg_chassis_motion.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 舵轮运动学库状态码
     */
    typedef enum
    {
        ALG_SWERVE_STATUS_OK = 0,           // 操作成功（所有模块均可用）
        ALG_SWERVE_STATUS_DEGRADED,         // 部分模块不可用，已降级运行
        ALG_SWERVE_STATUS_INVALID_ARGUMENT, // 参数非法（空指针、非法数值等）
        ALG_SWERVE_STATUS_NOT_INITIALIZED   // 对象未初始化
    } alg_swerve_status_t;

    /* ======================== 数据结构 ======================== */

    /**
     * @brief 舵轮模块几何位置（相对车体原点）
     */
    typedef struct
    {
        float position_x_m; // X 坐标（米）
        float position_y_m; // Y 坐标（米）
    } alg_swerve_module_geometry_t;

    /**
     * @brief 舵轮运动学命令
     * @note 包含期望的车体运动、旋转中心、参考航向等信息。
     */
    typedef struct
    {
        float velocity_x_m_per_s;           // 平移速度 X 分量（m/s）
        float velocity_y_m_per_s;           // 平移速度 Y 分量（m/s）
        float angular_velocity_rad_per_s;   // 角速度（rad/s）
        float reference_heading_rad;        // 参考航向角（弧度，用于坐标变换）
        float center_of_rotation_x_m;       // 旋转中心 X 坐标（相对车体原点）
        float center_of_rotation_y_m;       // 旋转中心 Y 坐标（相对车体原点）
        bool command_is_reference_relative; // true：平移速度相对参考航向坐标系；false：相对车体系
    } alg_swerve_command_t;

    /**
     * @brief 单个舵轮模块的目标状态（逆解输出）
     */
    typedef struct
    {
        float wheel_velocity_m_per_s; // 轮子线速度（m/s，正值表示前进）
        float steering_angle_rad;     // 舵角（弧度，相对车体 x 轴）
    } alg_swerve_module_target_t;

    /**
     * @brief 矩形底盘布局的模块索引（四轮）
     */
    typedef enum
    {
        ALG_SWERVE_MODULE_FRONT_LEFT = 0,   // 左前
        ALG_SWERVE_MODULE_FRONT_RIGHT,      // 右前
        ALG_SWERVE_MODULE_REAR_LEFT,        // 左后
        ALG_SWERVE_MODULE_REAR_RIGHT,       // 右后
        ALG_SWERVE_RECTANGULAR_MODULE_COUNT // 固定为 4
    } alg_swerve_rectangular_module_index_t;

    /**
     * @brief 舵轮底盘运动学实例
     * @note 存储几何布局和限速参数，配置数组由调用者持有。
     */
    typedef struct
    {
        const alg_swerve_module_geometry_t *module_geometry; // 模块几何数组（外部持有）
        size_t module_count;                                 // 模块数量
        float maximum_wheel_velocity_m_per_s;                // 最大轮线速度（>0）
        bool is_initialized;                                 // 是否已初始化
    } alg_swerve_t;

    /* ======================== 公开函数 ======================== */

    /**
     * @brief 初始化舵轮底盘运动学模型
     * @param me        模型对象
     * @param module_geometry  模块几何数组（将被引用，必须保持有效）
     * @param module_count      模块数量（>0）
     * @param maximum_wheel_velocity_m_per_s  最大轮速（>0）
     * @return 执行状态
     */
    alg_swerve_status_t alg_swerve_init(alg_swerve_t *me,
                                        const alg_swerve_module_geometry_t *module_geometry,
                                        size_t module_count, float maximum_wheel_velocity_m_per_s);

    /**
     * @brief 生成标准的矩形四轮布局
     * @param module_geometry  输出几何数组（长度必须至少为4）
     * @param half_wheelbase_m  半轴距（纵向距离的一半，>0）
     * @param half_track_width_m  半轮距（横向距离的一半，>0）
     * @return 执行状态
     * @note 顺序为左前、右前、左后、右后。
     */
    alg_swerve_status_t alg_swerve_configure_rectangular_layout(
        alg_swerve_module_geometry_t module_geometry[ALG_SWERVE_RECTANGULAR_MODULE_COUNT],
        float half_wheelbase_m, float half_track_width_m);

    /**
     * @brief 逆运动学：计算所有模块的目标（所有模块可用，绕原点）
     * @param me            模型对象
     * @param command       运动命令（旋转中心为原点时，center_of_rotation 字段可设为0）
     * @param module_targets 输出：每个模块的目标状态
     * @param target_capacity 输出数组容量（至少为 module_count）
     * @return 执行状态
     * @note 等价于调用 alg_swerve_calculate_with_availability 且所有模块可用。
     */
    alg_swerve_status_t alg_swerve_calculate(const alg_swerve_t *me,
                                             const alg_swerve_command_t *command,
                                             alg_swerve_module_target_t *module_targets,
                                             size_t target_capacity);

    /**
     * @brief 逆运动学：计算所有模块的目标（支持模块可用性）
     * @param me            模型对象
     * @param command       运动命令（包含任意旋转中心）
     * @param module_is_available  模块可用性数组（长度 module_count），NULL 表示全部可用
     * @param module_targets 输出：每个模块的目标状态
     * @param target_capacity 输出数组容量
     * @return 执行状态
     * @note 不可用模块输出轮速为0，舵角为0。
     *       若任一可用模块轮速超过上限，所有可用模块统一缩放。
     *       旋转中心可为任意点。
     */
    alg_swerve_status_t alg_swerve_calculate_with_availability(
        const alg_swerve_t *me, const alg_swerve_command_t *command,
        const bool *module_is_available, alg_swerve_module_target_t *module_targets,
        size_t target_capacity);

    /**
     * @brief 正运动学：从实测模块状态估计车体速度（加权最小二乘）
     * @param me            模型对象
     * @param measured_module_states  各模块实测状态（轮速+舵角）
     * @param module_is_available  模块可用性（NULL 表示全部可用）
     * @param odometry_weights  各模块权重（长度 module_count），NULL 表示等权重（1.0）
     * @param known_component_mask  已知速度分量掩码（bit0=vx, bit1=vy, bit2=wz）
     * @param known_velocity  已知速度分量值（若掩码非零，必须提供）
     * @param constraint_workspace  工作区（至少 2*module_count 个约束）
     * @param workspace_capacity  工作区容量
     * @param solution  输出：估计速度及残差
     * @return 执行状态
     * @note 每个模块提供两个约束（x 和 y 方向的轮速分量）。
     *       可用模块数不足 3 个时，需提供已知分量避免奇异。
     *       残差 RMS 可用于检测异常模块。
     */
    alg_chassis_status_t
    alg_swerve_forward(const alg_swerve_t *me,
                       const alg_swerve_module_target_t *measured_module_states,
                       const bool *module_is_available, const float *odometry_weights,
                       uint8_t known_component_mask, const alg_chassis_velocity_t *known_velocity,
                       alg_chassis_constraint_t *constraint_workspace, size_t workspace_capacity,
                       alg_chassis_solution_t *solution);

    /**
     * @brief 舵角最短路径优化（减少转向时间）
     * @param current_steering_angle_rad  当前实际舵角（弧度）
     * @param module_target  待优化的目标（输入/输出）
     * @return 执行状态
     * @note 若目标舵角与当前舵角之差大于 ±π/2，则将舵角反转 π 并取反轮速，
     *       使转向行程控制在 π/2 以内。
     *       此函数应在逆解后、发送到执行器前调用。
     */
    alg_swerve_status_t alg_swerve_optimize_target(float current_steering_angle_rad,
                                                   alg_swerve_module_target_t *module_target);

    /**
     * @brief 计算静止自锁目标（抵抗外力推动）
     * @param me            模型对象
     * @param module_targets 输出：各模块目标（零轮速，舵角指向车体中心）
     * @param target_capacity 输出数组容量
     * @return 执行状态
     * @note 每个模块的舵角指向车体原点方向，轮速为零，形成机械自锁。
     *       实际保持能力取决于舵机闭环、驱动器使能和摩擦力。
     */
    alg_swerve_status_t alg_swerve_calculate_self_lock(const alg_swerve_t *me,
                                                       alg_swerve_module_target_t *module_targets,
                                                       size_t target_capacity);

    /**
     * @brief 角度回绕函数（将角度回绕到 [-π, π)）
     * @param angle_rad  输入角度
     * @return 回绕后的角度
     */
    float alg_swerve_wrap_angle_rad(float angle_rad);

#ifdef __cplusplus
}
#endif

#endif