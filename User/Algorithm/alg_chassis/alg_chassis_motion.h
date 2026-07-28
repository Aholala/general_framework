/**
 * @file alg_chassis_motion.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 底盘运动学公共数学内核头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 各类底盘把轮子测量转换为线性约束，本模块负责速度求解、坐标变换、
 *       任意旋转中心转换、轮速统一缩放和里程计积分。
 *       无动态内存，所有输出空间由调用者提供。
 */

#ifndef ALG_CHASSIS_MOTION_H
#define ALG_CHASSIS_MOTION_H

#include <stdbool.h> // bool
#include <stddef.h>  // size_t
#include <stdint.h>  // uint8_t

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 速度分量位掩码 ======================== */

/** @brief X 方向速度分量位 */
#define ALG_CHASSIS_COMPONENT_VELOCITY_X (1U << 0)
/** @brief Y 方向速度分量位 */
#define ALG_CHASSIS_COMPONENT_VELOCITY_Y (1U << 1)
/** @brief 角速度分量位 */
#define ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY (1U << 2)
/** @brief 所有速度分量位掩码 */
#define ALG_CHASSIS_COMPONENT_ALL                                                                  \
    (ALG_CHASSIS_COMPONENT_VELOCITY_X | ALG_CHASSIS_COMPONENT_VELOCITY_Y |                         \
     ALG_CHASSIS_COMPONENT_ANGULAR_VELOCITY)

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 底盘算法状态码
     */
    typedef enum
    {
        ALG_CHASSIS_STATUS_OK = 0,           // 操作成功
        ALG_CHASSIS_STATUS_DEGRADED,         // 降级（可用约束数少于名义值）
        ALG_CHASSIS_STATUS_INVALID_ARGUMENT, // 参数非法
        ALG_CHASSIS_STATUS_NOT_INITIALIZED,  // 对象未初始化（本模块无状态）
        ALG_CHASSIS_STATUS_UNDERDETERMINED,  // 欠定（约束数少于未知分量数）
        ALG_CHASSIS_STATUS_SINGULAR,         // 奇异（无法求解）
        ALG_CHASSIS_STATUS_NUMERICAL_ERROR   // 数值错误
    } alg_chassis_status_t;

    /* ======================== 速度结构体 ======================== */

    /**
     * @brief 底盘速度（车体坐标系）
     * @note 所有速度量为车体坐标系下的值
     */
    typedef struct
    {
        float velocity_x_m_per_s;         // X 方向速度（m/s），沿车体前进方向
        float velocity_y_m_per_s;         // Y 方向速度（m/s），沿车体侧向
        float angular_velocity_rad_per_s; // 角速度（rad/s），绕 Z 轴（航向角变化率）
    } alg_chassis_velocity_t;

    /* ======================== 位姿结构体 ======================== */

    /**
     * @brief 底盘位姿（参考坐标系）
     * @note 用于里程计积分，保存世界坐标系下的位置和航向
     */
    typedef struct
    {
        float position_x_m; // X 位置（米）
        float position_y_m; // Y 位置（米）
        float heading_rad;  // 航向角（弧度）
    } alg_chassis_pose_t;

    /* ======================== 里程计积分方法枚举 ======================== */

    /**
     * @brief 里程计积分方法
     * @note Euler：计算量最低，精度最差
     *       Midpoint：常用折中，适合大多数场景
     *       Exact：恒定速度模型下的精确积分，计算量最大
     */
    typedef enum
    {
        ALG_CHASSIS_INTEGRATION_EULER = 0, // 欧拉积分（计算量最低）
        ALG_CHASSIS_INTEGRATION_MIDPOINT,  // 中点积分（常用折中）
        ALG_CHASSIS_INTEGRATION_EXACT      // 精确积分（恒定速度模型）
    } alg_chassis_integration_method_t;

    /* ======================== 约束结构体 ======================== */

    /**
     * @brief 速度约束
     * @note 约束方程：
     *       velocity_x_coefficient * vx +
     *       velocity_y_coefficient * vy +
     *       angular_velocity_coefficient_m * wz =
     *       measured_velocity_m_per_s
     *       每个轮子或传感器提供一个约束
     */
    typedef struct
    {
        float velocity_x_coefficient;         // vx 系数（方向投影）
        float velocity_y_coefficient;         // vy 系数（方向投影）
        float angular_velocity_coefficient_m; // wz 系数（乘以距离得到线速度）
        float measured_velocity_m_per_s;      // 测量速度（m/s）
        float weight;                         // 权重（>= 0，0 表示禁用）
        bool is_available;                    // 是否可用
    } alg_chassis_constraint_t;

    /* ======================== 求解结果结构体 ======================== */

    /**
     * @brief 速度求解结果
     */
    typedef struct
    {
        alg_chassis_velocity_t velocity;         // 求解出的速度
        float residual_root_mean_square_m_per_s; // 残差均方根（m/s），衡量拟合质量
        size_t used_constraint_count;            // 实际使用的约束数
        size_t unknown_component_count;          // 未知分量数
        bool is_degraded;                        // 是否降级（约束数少于名义值）
    } alg_chassis_solution_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 加权最小二乘求解底盘速度
     * @param constraints 约束数组
     * @param constraint_count 约束数量
     * @param known_component_mask 已知分量掩码（位掩码）
     * @param known_velocity 已知速度分量
     * @param nominal_constraint_count 名义约束数（用于判断降级）
     * @param solution 输出求解结果
     * @return 执行状态
     * @note 使用 QR 分解求解加权最小二乘问题
     *       可锁定一个或多个已知分量（如差速底盘固定横向速度为零）
     */
    alg_chassis_status_t alg_chassis_solve_velocity(const alg_chassis_constraint_t *constraints,
                                                    size_t constraint_count,
                                                    uint8_t known_component_mask,
                                                    const alg_chassis_velocity_t *known_velocity,
                                                    size_t nominal_constraint_count,
                                                    alg_chassis_solution_t *solution);

    /**
     * @brief 计算约束残差
     * @param constraints 约束数组
     * @param constraint_count 约束数量
     * @param velocity 速度值
     * @param residuals_m_per_s 输出残差数组（m/s）
     * @param residual_capacity 残差数组容量
     * @return 执行状态
     */
    alg_chassis_status_t alg_chassis_calculate_constraint_residuals(
        const alg_chassis_constraint_t *constraints, size_t constraint_count,
        const alg_chassis_velocity_t *velocity, float *residuals_m_per_s, size_t residual_capacity);

    /**
     * @brief 将参考坐标系速度变换到车体坐标系
     * @param reference_velocity 参考坐标系速度
     * @param reference_heading_rad 参考航向角（弧度）
     * @param body_velocity 输出车体速度
     * @return 执行状态
     * @note 将世界坐标系或云台坐标系下的速度命令旋转到车体坐标系
     *       角速度保持不变（航向角速度不受旋转影响）
     */
    alg_chassis_status_t
    alg_chassis_transform_reference_to_body(const alg_chassis_velocity_t *reference_velocity,
                                            float reference_heading_rad,
                                            alg_chassis_velocity_t *body_velocity);

    /**
     * @brief 将旋转中心处的速度转换到车体原点
     * @param center_velocity 旋转中心处速度
     * @param center_of_rotation_x_m 旋转中心 X 坐标
     * @param center_of_rotation_y_m 旋转中心 Y 坐标
     * @param origin_velocity 输出原点速度
     * @return 执行状态
     * @note v_origin = v_center + w × (-r_center)
     *       公式：v_origin.x = v_center.x + w * y_center
     *             v_origin.y = v_center.y - w * x_center
     */
    alg_chassis_status_t alg_chassis_convert_center_velocity_to_origin(
        const alg_chassis_velocity_t *center_velocity, float center_of_rotation_x_m,
        float center_of_rotation_y_m, alg_chassis_velocity_t *origin_velocity);

    /**
     * @brief 统一缩放轮速
     * @param wheel_velocities 轮速数组（会被修改）
     * @param wheel_is_available 轮子可用性数组（NULL 表示全部可用）
     * @param wheel_count 轮子数量
     * @param maximum_absolute_velocity 最大允许绝对速度
     * @param applied_scale 输出实际缩放系数（可为 NULL）
     * @return 执行状态
     * @note 找到最大轮速并统一缩放所有可用轮子，保持速度向量比例
     *       不可用轮子输出归零
     */
    alg_chassis_status_t alg_chassis_scale_wheel_velocities(float *wheel_velocities,
                                                            const bool *wheel_is_available,
                                                            size_t wheel_count,
                                                            float maximum_absolute_velocity,
                                                            float *applied_scale);

    /**
     * @brief 里程计积分
     * @param me 位姿对象（会被修改）
     * @param body_velocity 车体速度
     * @param delta_time_s 时间步长（秒）
     * @param integration_method 积分方法
     * @return 执行状态
     * @note 支持 Euler、Midpoint、Exact 三种积分方法
     *       长时间定位仍需 IMU、视觉或其他绝对观测修正漂移
     */
    alg_chassis_status_t
    alg_chassis_integrate_odometry(alg_chassis_pose_t *me,
                                   const alg_chassis_velocity_t *body_velocity, float delta_time_s,
                                   alg_chassis_integration_method_t integration_method);

#ifdef __cplusplus
}
#endif

#endif