/**
 * @file alg_mecanum.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 四轮麦克纳姆底盘运动学算法库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 纯 C11 实现，不依赖 HAL、CMSIS 或 RTOS。
 *       提供完整的逆运动学（速度→轮速）、正运动学（轮速→速度）、任意旋转中心支持、
 *       X/O 辊子布局、缺轮降级及轮速饱和保护。
 *       所有数据由调用者管理，不使用动态内存。
 *       依赖 alg_chassis_motion 模块提供的公共类型和辅助函数。
 */

#ifndef ALG_MECANUM_H
#define ALG_MECANUM_H

#include "alg_chassis_motion.h"

#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 麦克纳姆底盘轮子数量（固定为4） */
#define ALG_MECANUM_WHEEL_COUNT (4U)

    /**
     * @brief 辊子排列类型
     */
    typedef enum
    {
        ALG_MECANUM_ROLLER_X = 0, // X 型排列（常见）
        ALG_MECANUM_ROLLER_O      // O 型排列（横向运动系数反向）
    } alg_mecanum_roller_arrangement_t;

    /**
     * @brief 轮子索引（固定顺序）
     * @note 顺序为左前、右前、左后、右后。
     */
    typedef enum
    {
        ALG_MECANUM_WHEEL_FRONT_LEFT = 0, // 左前轮（FL）
        ALG_MECANUM_WHEEL_FRONT_RIGHT,    // 右前轮（FR）
        ALG_MECANUM_WHEEL_REAR_LEFT,      // 左后轮（RL）
        ALG_MECANUM_WHEEL_REAR_RIGHT      // 右后轮（RR）
    } alg_mecanum_wheel_index_t;

    /**
     * @brief 麦克纳姆底盘配置结构体
     * @note 所有几何参数均以米为单位，角度以弧度/秒为单位。
     *       调用者负责确保几何参数合理且权重大于零。
     */
    typedef struct
    {
        float wheel_radius_m;                                // 轮子半径（>0）
        float half_wheelbase_m;                              // 半轴距（纵向距离的一半，>0）
        float half_track_width_m;                            // 半轮距（横向距离的一半，>0）
        float direction_sign[ALG_MECANUM_WHEEL_COUNT];       // 各轮安装方向（+1 或 -1）
        float odometry_weight[ALG_MECANUM_WHEEL_COUNT];      // 正解时各轮权重（>0）
        float maximum_wheel_angular_velocity_rad_per_s;      // 轮角速度上限（>0）
        alg_mecanum_roller_arrangement_t roller_arrangement; // 辊子排列类型（X 或 O）
    } alg_mecanum_config_t;

    /**
     * @brief 麦克纳姆底盘运动学实例
     * @note 初始化后内部预计算横向和角速度系数，提高运行效率。
     */
    typedef struct
    {
        alg_mecanum_config_t config;                        // 用户配置
        float lateral_coefficient[ALG_MECANUM_WHEEL_COUNT]; // 横向速度系数（±1，由辊子布局决定）
        float angular_coefficient_m
            [ALG_MECANUM_WHEEL_COUNT]; // 角速度系数（单位米，由几何和辊子布局决定）
        bool is_initialized;           // 是否已初始化
    } alg_mecanum_t;

    /* ======================== 公开函数 ======================== */

    /**
     * @brief 初始化麦克纳姆底盘运动学模型
     * @param me     模型对象
     * @param config 配置参数
     * @return 执行状态
     * @note 预计算横向和角速度系数。
     *       若配置参数非法（半径≤0、权重≤0、方向符号非±1、辊子类型无效），返回错误。
     */
    alg_chassis_status_t alg_mecanum_init(alg_mecanum_t *me, const alg_mecanum_config_t *config);

    /**
     * @brief 逆运动学：将底盘原点速度映射到各轮角速度（绕底盘中心旋转）
     * @param me        模型对象
     * @param chassis_velocity  期望的底盘原点速度（vx, vy, wz）
     * @param wheel_is_available  各轮是否可用（长度为4的布尔数组）
     * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度（rad/s）
     * @param applied_scale  输出：实际应用的缩放比例（<=1.0），若无需缩放则为1.0
     * @return 执行状态
     * @note 若某轮不可用，其角速度输出为0。
     *       若任何可用轮超过速度上限，所有可用轮按相同比例缩放以保持运动方向。
     *       等价于调用 alg_mecanum_inverse_with_center_of_rotation 且旋转中心为(0,0)。
     */
    alg_chassis_status_t
    alg_mecanum_inverse(const alg_mecanum_t *me, const alg_chassis_velocity_t *chassis_velocity,
                        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                        float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                        float *applied_scale);

    /**
     * @brief 逆运动学：将指定旋转中心处的速度映射到各轮角速度
     * @param me        模型对象
     * @param center_velocity  旋转中心处的速度
     * @param center_of_rotation_x_m  旋转中心相对底盘原点的 X 坐标（米）
     * @param center_of_rotation_y_m  旋转中心相对底盘原点的 Y 坐标（米）
     * @param wheel_is_available  各轮是否可用
     * @param wheel_angular_velocities_rad_per_s  输出：各轮角速度
     * @param applied_scale  输出：实际缩放比例
     * @return 执行状态
     * @note 旋转中心可为底盘内或外的任意点。
     *       实现步骤：先将旋转中心速度转换到底盘原点，再应用标准逆解。
     *       这使得可以实现绕云台轴、车轮接地点等任意点旋转。
     */
    alg_chassis_status_t alg_mecanum_inverse_with_center_of_rotation(
        const alg_mecanum_t *me, const alg_chassis_velocity_t *center_velocity,
        float center_of_rotation_x_m, float center_of_rotation_y_m,
        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
        float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT], float *applied_scale);

    /**
     * @brief 正运动学：从实测轮速估计底盘速度（基于加权最小二乘）
     * @param me        模型对象
     * @param wheel_angular_velocities_rad_per_s  各轮实测角速度
     * @param wheel_is_available  各轮是否可用（NULL 表示全部可用）
     * @param known_component_mask  已知分量的掩码（bit0=vx, bit1=vy, bit2=wz），0表示无先验
     * @param known_velocity  已知分量值（若掩码非零，必须提供）
     * @param solution  输出：估计速度及残差
     * @return 执行状态
     * @note 使用配置中的 odometry_weight 作为各轮权重。
     *       当可用轮数不足或约束奇异时，返回 SINGULAR。
     *       若提供已知分量，可增强数值稳定性或处理缺轮。
     *       残差 rms 可用于检测轮速传感器异常。
     */
    alg_chassis_status_t
    alg_mecanum_forward(const alg_mecanum_t *me,
                        const float wheel_angular_velocities_rad_per_s[ALG_MECANUM_WHEEL_COUNT],
                        const bool wheel_is_available[ALG_MECANUM_WHEEL_COUNT],
                        uint8_t known_component_mask, const alg_chassis_velocity_t *known_velocity,
                        alg_chassis_solution_t *solution);

#ifdef __cplusplus
}
#endif

#endif /* ALG_MECANUM_H */