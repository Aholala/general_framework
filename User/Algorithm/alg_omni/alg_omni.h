/**
 * @file alg_omni.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 通用全向轮底盘运动学算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       支持任意数量、任意位置和任意驱动方向的全向轮。
 *       提供逆运动学（速度→轮速）、正运动学（轮速→速度）、任意旋转中心支持、
 *       轮速饱和保护及缺轮降级。
 *       所有数据由调用者管理，不使用动态内存。
 *       依赖 alg_chassis_motion 模块提供的公共类型和辅助函数。
 */

#ifndef ALG_OMNI_H
#define ALG_OMNI_H

#include "alg_chassis_motion.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 典型三轮全向底盘轮数 */
#define ALG_OMNI_THREE_WHEEL_COUNT (3U)
/** @brief 典型四轮全向底盘轮数 */
#define ALG_OMNI_FOUR_WHEEL_COUNT (4U)

    /**
     * @brief 单个全向轮的配置结构体
     * @note 描述轮子在车体坐标系中的位置、驱动方向、半径、安装符号和正解权重。
     *       驱动方向角为轮子产生牵引力方向与车体 x 轴的夹角（弧度）。
     */
    typedef struct
    {
        float position_x_m;        // 轮心相对车体原点的 X 坐标（米）
        float position_y_m;        // 轮心相对车体原点的 Y 坐标（米）
        float drive_direction_rad; // 驱动方向角（弧度，相对车体 x 轴）
        float wheel_radius_m;      // 轮子半径（米，>0）
        float direction_sign;      // 电机安装方向符号（+1 或 -1）
        float odometry_weight;     // 正解时该轮的权重（>0）
    } alg_omni_wheel_config_t;

    /**
     * @brief 全向底盘运动学实例
     * @note 初始化后引用调用者提供的轮组配置数组。
     *       配置数组在对象生命周期内必须保持有效。
     */
    typedef struct
    {
        const alg_omni_wheel_config_t *wheel_configs;   // 轮组配置数组（外部持有）
        size_t wheel_count;                             // 轮子数量
        float maximum_wheel_angular_velocity_rad_per_s; // 轮速上限（>0）
        bool is_initialized;                            // 是否已初始化
    } alg_omni_t;

    /* ======================== 公开函数 ======================== */

    /**
     * @brief 初始化全向底盘运动学模型
     * @param me        模型对象
     * @param wheel_configs  轮组配置数组（将被对象引用，必须保持有效）
     * @param wheel_count    轮子数量（必须 >0）
     * @param maximum_wheel_angular_velocity_rad_per_s  轮速上限（>0）
     * @return 执行状态
     * @note 所有轮配置必须有效（位置、方向、半径、权重有限且合法）。
     *       配置数组指针被保存，调用者不能释放或修改该数组。
     */
    alg_chassis_status_t alg_omni_init(alg_omni_t *me, const alg_omni_wheel_config_t *wheel_configs,
                                       size_t wheel_count,
                                       float maximum_wheel_angular_velocity_rad_per_s);

    /**
     * @brief 生成均匀圆周切向布局的轮组配置（用于对称全向底盘）
     * @param wheel_configs  输出：轮组配置数组（需有 wheel_count 个元素）
     * @param wheel_count    轮子数量（≥2）
     * @param center_to_wheel_distance_m  轮心到车体原点的距离（>0）
     * @param wheel_radius_m  轮子半径（>0）
     * @param first_wheel_position_angle_rad  第一个轮子的位置角（弧度）
     * @param tangential_direction_sign  切向方向符号（+1 或
     * -1），决定轮子驱动方向是顺时针还是逆时针切向
     * @param wheel_direction_signs  各轮电机安装方向符号数组（长度 wheel_count），可为 NULL（则全为
     * +1）
     * @param odometry_weight  所有轮相同的正解权重（>0）
     * @return 执行状态
     * @note 轮子均匀分布在圆周上，驱动方向沿圆周切线。
     *       适用于标准的三轮或四轮全向底盘。
     *       第一个轮子位置角决定了整体旋转角度。
     */
    alg_chassis_status_t
    alg_omni_configure_tangential_layout(alg_omni_wheel_config_t *wheel_configs, size_t wheel_count,
                                         float center_to_wheel_distance_m, float wheel_radius_m,
                                         float first_wheel_position_angle_rad,
                                         float tangential_direction_sign,
                                         const float *wheel_direction_signs, float odometry_weight);

    /**
     * @brief 逆运动学：将车体原点速度映射到各轮角速度
     * @param me        模型对象
     * @param chassis_velocity  期望的车体原点速度
     * @param wheel_is_available  各轮是否可用（长度为 wheel_count 的数组），NULL 表示全部可用
     * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度（需有 wheel_count 容量）
     * @param output_capacity  输出数组容量（至少为 wheel_count）
     * @param applied_scale  输出：实际缩放比例（≤1.0）
     * @return 执行状态
     * @note 不可用轮输出为 0。
     *       若任一可用轮超速，所有可用轮按比例缩放以保持运动方向。
     *       等价于绕原点旋转。
     */
    alg_chassis_status_t alg_omni_inverse(const alg_omni_t *me,
                                          const alg_chassis_velocity_t *chassis_velocity,
                                          const bool *wheel_is_available,
                                          float *wheel_angular_velocities_rad_per_s,
                                          size_t output_capacity, float *applied_scale);

    /**
     * @brief 逆运动学：将指定旋转中心的速度映射到各轮角速度
     * @param me        模型对象
     * @param center_velocity  旋转中心处的速度
     * @param center_of_rotation_x_m  旋转中心相对车体原点的 X 坐标
     * @param center_of_rotation_y_m  旋转中心相对车体原点的 Y 坐标
     * @param wheel_is_available  各轮是否可用
     * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度
     * @param output_capacity  输出数组容量
     * @param applied_scale  输出：实际缩放比例
     * @return 执行状态
     * @note 旋转中心可为任意点，实现绕云台轴、某轮接地点或车外点旋转。
     */
    alg_chassis_status_t alg_omni_inverse_with_center_of_rotation(
        const alg_omni_t *me, const alg_chassis_velocity_t *center_velocity,
        float center_of_rotation_x_m, float center_of_rotation_y_m, const bool *wheel_is_available,
        float *wheel_angular_velocities_rad_per_s, size_t output_capacity, float *applied_scale);

    /**
     * @brief 正运动学：从实测轮速估计车体速度（加权最小二乘）
     * @param me        模型对象
     * @param wheel_angular_velocities_rad_per_s  各轮实测角速度（长度 wheel_count）
     * @param wheel_is_available  各轮是否可用（NULL 表示全部可用）
     * @param known_component_mask  已知分量的掩码（bit0=vx, bit1=vy, bit2=wz）
     * @param known_velocity  已知分量值（若掩码非零，必须提供）
     * @param constraint_workspace  工作区（用于构建约束，长度至少 wheel_count）
     * @param workspace_capacity  工作区容量
     * @param solution  输出：估计速度及残差
     * @return 执行状态
     * @note 使用各轮的 odometry_weight 作为权重。
     *       可用轮数不足 3 时需提供已知分量以避免奇异。
     *       残差 RMS 可用于检测异常轮速。
     *       工作区由调用者提供，避免动态内存。
     */
    alg_chassis_status_t
    alg_omni_forward(const alg_omni_t *me, const float *wheel_angular_velocities_rad_per_s,
                     const bool *wheel_is_available, uint8_t known_component_mask,
                     const alg_chassis_velocity_t *known_velocity,
                     alg_chassis_constraint_t *constraint_workspace, size_t workspace_capacity,
                     alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_OMNI_H */