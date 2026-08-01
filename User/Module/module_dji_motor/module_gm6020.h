/**
 * @file module_gm6020.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 大疆 GM6020 云台电机专用派生模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 固定电机型号为 GM6020，防止调用者误填 M2006/M3508 型号。
 *       继承自 module_dji_motor_t，提供电压直通、电流、速度、角度四种控制模式。
 *       控制命令为有符号 16 位原始电压命令，范围 [-30000, 30000]。
 *       motor_identifier 只能为 1~7（GM6020 不支持 ID 8）。
 */

#ifndef MODULE_GM6020_H
#define MODULE_GM6020_H

#include "module_dji_motor.h" // 大疆电机基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct module_gm6020 module_gm6020_t;

    /* ======================== 控制模式枚举 ======================== */

    /**
     * @brief GM6020 控制模式
     * @note 与 module_dji_control_mode_t 一一对应，但独立命名以避免混淆
     *       VOLTAGE/CURRENT/VELOCITY/ANGLE 与通用 DJI 模式一一对应
     *       使用 module_gm6020_map_control_mode() 映射到通用 DJI 模式
     */
    typedef enum
    {
        MODULE_GM6020_CONTROL_VOLTAGE = 0, // 电压模式（直接输出原始电压命令）
        MODULE_GM6020_CONTROL_CURRENT,     // 电流 PID
        MODULE_GM6020_CONTROL_VELOCITY,    // 速度 PID + 电流 PID
        MODULE_GM6020_CONTROL_ANGLE        // 角度 PID + 速度 PID + 电流 PID
    } module_gm6020_control_mode_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief GM6020 初始化配置
     * @note motor_identifier 只能为 1~7（GM6020 不支持 ID 8）
     *       direction_sign 只能为 +1 或 -1
     *       GM6020 为直驱电机，减速比固定为 1.0F
     */
    typedef struct
    {
        const char *motor_name;                       // 调试可见的电机名称
        uint32_t registration_key;                    // 注册键值
        module_dji_motor_bus_t *motor_bus;            // DJI 电机总线（共享）
        module_gm6020_control_mode_t control_mode;    // 控制模式
        uint8_t motor_identifier;                     // 电机标识符（1~7）
        float direction_sign;                         // 方向符号（+1 或 -1）
        float maximum_temperature_c;                  // 最大允许温度（℃）
        float current_scale_a_per_count;              // 电流换算因子（A/原始值）
        module_motor_pid_config_t current_pid_config;
        module_motor_pid_config_t velocity_pid_config;
        module_motor_pid_config_t angle_pid_config;
    } module_gm6020_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief GM6020 设备对象
     * @note 直接继承 module_dji_motor_t，无额外字段
     *       继承链：module_motor_t → module_dji_motor_t → module_gm6020_t
     */
    struct module_gm6020
    {
        module_dji_motor_t super; // 大疆电机基类
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 GM6020 电机
     * @param me 电机对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_motor_status_t module_gm6020_init(module_gm6020_t *const me,
                                             const module_gm6020_config_t *const config);

    /**
     * @brief 注册电机到电机注册表
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_gm6020_register(module_gm6020_t *const me,
                                                 module_motor_registry_t *const registry);

    /**
     * @brief 从电机注册表注销电机
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_gm6020_unregister(module_gm6020_t *const me,
                                                   module_motor_registry_t *const registry);

    /**
     * @brief 向上转型为 module_motor_t 基类指针
     * @param me 电机对象
     * @return 基类指针
     */
    module_motor_t *module_gm6020_as_motor(module_gm6020_t *const me);

    /**
     * @brief 向上转型为 module_dji_motor_t 基类指针
     * @param me 电机对象
     * @return 大疆电机基类指针
     */
    module_dji_motor_t *module_gm6020_as_dji_motor(module_gm6020_t *const me);

    /**
     * @brief 使能电机
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_gm6020_enable(module_gm6020_t *const me);

    /**
     * @brief 禁用电机
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_gm6020_disable(module_gm6020_t *const me);

    /**
     * @brief 设置原始电压命令（电压模式）
     * @param me 电机对象
     * @param voltage_command_raw 电压命令（-30000~30000）
     * @return 执行状态
     * @note 仅当控制模式为 VOLTAGE 时有效
     */
    module_motor_status_t module_gm6020_set_voltage_command_raw(module_gm6020_t *const me,
                                                                int16_t voltage_command_raw);

    module_motor_status_t module_gm6020_set_current_a(module_gm6020_t *const me, float current_a);

    /**
     * @brief 设置速度目标（速度模式）
     * @param me 电机对象
     * @param velocity_rad_per_s 速度目标（rad/s）
     * @return 执行状态
     * @note 仅当控制模式为 VELOCITY 时有效
     */
    module_motor_status_t module_gm6020_set_velocity_rad_per_s(module_gm6020_t *const me,
                                                               float velocity_rad_per_s);

    /**
     * @brief 设置角度目标（角度模式）
     * @param me 电机对象
     * @param angle_rad 角度目标（弧度）
     * @return 执行状态
     * @note 仅当控制模式为 ANGLE 时有效
     */
    module_motor_status_t module_gm6020_set_angle_rad(module_gm6020_t *const me, float angle_rad);

    /**
     * @brief 周期更新电机
     * @param me 电机对象
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 调用后需通过 module_dji_motor_bus_flush 发送命令
     */
    module_motor_status_t module_gm6020_update(module_gm6020_t *const me, float delta_time_s);

    /**
     * @brief 获取电机反馈数据
     * @param me 电机对象
     * @return 反馈数据指针，未初始化或离线返回 NULL
     */
    const module_motor_feedback_t *module_gm6020_get_feedback(const module_gm6020_t *const me);

    /**
     * @brief 获取当前原始电压命令值
     * @param me 电机对象
     * @return 当前命令值（-30000~30000）
     */
    int16_t module_gm6020_get_voltage_command_raw(const module_gm6020_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_GM6020_H */
