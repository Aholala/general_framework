/**
 * @file module_swerve.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 单个舵轮执行模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 把 alg_swerve_module_target_t 转换成驱动电机速度目标和舵向电机位置目标。
 *       底盘几何解算留在 alg_swerve，本模块只处理一个物理舵轮。
 */

#ifndef MODULE_SWERVE_H
#define MODULE_SWERVE_H

#include "alg_swerve.h"   // 舵轮运动学算法
#include "module_motor.h" // 电机基类

#include <stdbool.h> // bool

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 舵轮模块状态码
     */
    typedef enum
    {
        MODULE_SWERVE_STATUS_OK = 0,           // 操作成功
        MODULE_SWERVE_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_SWERVE_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_SWERVE_STATUS_NOT_READY,        // 未就绪（电机离线或禁用）
        MODULE_SWERVE_STATUS_ALGORITHM_ERROR,  // 运动学算法错误
        MODULE_SWERVE_STATUS_MOTOR_ERROR       // 电机操作错误
    } module_swerve_status_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 舵轮初始化配置
     */
    typedef struct
    {
        module_motor_t *drive_motor;    // 驱动电机基类指针
        module_motor_t *steering_motor; // 舵向电机基类指针
        float wheel_radius_m;           // 轮子半径（米）
        float drive_reduction_ratio;    // 驱动电机到轮子的减速比（电机转速/轮子转速）
        float steering_zero_offset_rad; // 舵向零位偏移（电机零位与物理零位的偏差）
        float drive_direction_sign;     // 驱动电机方向符号（+1 或 -1）
        float steering_direction_sign;  // 舵向电机方向符号（+1 或 -1）
    } module_swerve_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 舵轮设备对象
     */
    typedef struct
    {
        module_motor_t *drive_motor;    // 驱动电机
        module_motor_t *steering_motor; // 舵向电机
        float wheel_radius_m;           // 轮半径（米）
        float drive_reduction_ratio;    // 驱动减速比
        float steering_zero_offset_rad; // 舵向零位偏移（弧度）
        float drive_direction_sign;     // 驱动方向符号
        float steering_direction_sign;  // 舵向方向符号
        bool is_enabled;                // 是否已使能
        bool is_initialized;            // 是否已初始化
    } module_swerve_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化舵轮模块
     * @param me 舵轮对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_swerve_status_t module_swerve_init(module_swerve_t *me,
                                              const module_swerve_config_t *config);

    /**
     * @brief 使能舵轮（按顺序使能驱动电机和舵向电机）
     * @param me 舵轮对象
     * @return 执行状态
     * @note 若任一电机使能失败，已使能的电机将被禁用
     */
    module_swerve_status_t module_swerve_enable(module_swerve_t *me);

    /**
     * @brief 禁用舵轮（禁用驱动电机和舵向电机）
     * @param me 舵轮对象
     * @return 执行状态
     */
    module_swerve_status_t module_swerve_disable(module_swerve_t *me);

    /**
     * @brief 应用运动学目标到舵轮
     * @param me 舵轮对象
     * @param target 运动学目标（线速度 + 舵角）
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 内部调用 alg_swerve_optimize_target 优化目标
     *       将目标换算为电机目标并调用 module_motor_update
     */
    module_swerve_status_t module_swerve_apply_target(module_swerve_t *me,
                                                      const alg_swerve_module_target_t *target,
                                                      float delta_time_s);

    /**
     * @brief 获取当前舵向角度
     * @param me 舵轮对象
     * @param[out] steering_angle_rad 输出舵向角度（弧度）
     * @return 执行状态
     */
    module_swerve_status_t module_swerve_get_steering_angle(const module_swerve_t *me,
                                                            float *steering_angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SWERVE_H */