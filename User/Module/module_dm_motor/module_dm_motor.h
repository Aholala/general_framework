/**
 * @file module_dm_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙电机 CAN 协议驱动头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持 MIT、速度和位置速度三种控制模式，以及使能、失能、保存零位和清故障命令。
 *       DM4310 通过派生配置复用本模块。
 */

#ifndef MODULE_DM_MOTOR_H
#define MODULE_DM_MOTOR_H

#include "bsp_can.h"      // CAN BSP 抽象层
#include "module_motor.h" // 电机基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 前向声明 ======================== */

    typedef struct module_dm_motor module_dm_motor_t;

    /* ======================== 枚举类型 ======================== */

    /**
     * @brief 达妙电机控制模式
     */
    typedef enum
    {
        MODULE_DM_MODE_MIT = 0,          // MIT 模式（位置+速度+Kp+Kd+扭矩）
        MODULE_DM_MODE_VELOCITY,         // 速度模式
        MODULE_DM_MODE_POSITION_VELOCITY // 位置+速度模式
    } module_dm_control_mode_t;

    /**
     * @brief 状态命令类型
     */
    typedef enum
    {
        MODULE_DM_COMMAND_DISABLE = 0, // 失能
        MODULE_DM_COMMAND_ENABLE,      // 使能
        MODULE_DM_COMMAND_SAVE_ZERO,   // 保存零位（修改驱动器持久状态，需谨慎）
        MODULE_DM_COMMAND_CLEAR_FAULT  // 清除故障
    } module_dm_state_command_t;

    /**
     * @brief 驱动器故障码
     */
    typedef enum
    {
        MODULE_DM_FAULT_NONE = 0,                    // 无故障
        MODULE_DM_FAULT_OVER_VOLTAGE = 8,            // 过压
        MODULE_DM_FAULT_UNDER_VOLTAGE = 9,           // 欠压
        MODULE_DM_FAULT_OVER_CURRENT = 10,           // 过流
        MODULE_DM_FAULT_MOS_OVER_TEMPERATURE = 11,   // MOS 管过温
        MODULE_DM_FAULT_MOTOR_OVER_TEMPERATURE = 12, // 电机过温
        MODULE_DM_FAULT_COMMUNICATION_LOST = 13,     // 通信丢失
        MODULE_DM_FAULT_OVERLOAD = 14                // 过载
    } module_dm_fault_t;

    /* ======================== 结构体类型 ======================== */

    /**
     * @brief 达妙电机限制参数
     * @note 用于浮点量量化到协议字段，必须与具体固件协议一致
     */
    typedef struct
    {
        float position_min_rad;       // 位置最小值（弧度）
        float position_max_rad;       // 位置最大值（弧度）
        float velocity_min_rad_per_s; // 速度最小值（rad/s）
        float velocity_max_rad_per_s; // 速度最大值（rad/s）
        float torque_min_nm;          // 扭矩最小值（Nm）
        float torque_max_nm;          // 扭矩最大值（Nm）
        float proportional_gain_min;  // Kp 最小值
        float proportional_gain_max;  // Kp 最大值
        float derivative_gain_min;    // Kd 最小值
        float derivative_gain_max;    // Kd 最大值
    } module_dm_limits_t;

    /**
     * @brief MIT 模式命令结构体
     */
    typedef struct
    {
        float position_rad;       // 目标位置（弧度）
        float velocity_rad_per_s; // 目标速度（rad/s）
        float proportional_gain;  // 比例增益 Kp
        float derivative_gain;    // 微分增益 Kd
        float torque_nm;          // 前馈扭矩（Nm）
    } module_dm_mit_command_t;

    /**
     * @brief 模式操作虚表（用于多态）
     * @note 不同控制模式以不同编码方式实现同名目标更新
     */
    typedef struct
    {
        module_motor_status_t (*encode_command)(module_dm_motor_t *const me,
                                                uint8_t transmit_data[8]);
        uint32_t (*get_transmit_identifier)(const module_dm_motor_t *const me);
    } module_dm_mode_ops_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 达妙电机初始化配置
     */
    typedef struct
    {
        const char *logical_name;              // 逻辑名称
        uint32_t registration_key;             // 注册键值
        bsp_can_t *can;                        // CAN BSP 基类
        module_dm_control_mode_t control_mode; // 控制模式
        uint32_t master_identifier;            // 主机标识符（CAN ID 基址）
        uint32_t feedback_identifier;          // 反馈标识符（CAN ID）
        uint32_t transmit_timeout_ms;          // CAN 发送超时（毫秒）
        module_dm_limits_t limits;             // 限制参数
    } module_dm_motor_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 达妙电机设备对象
     */
    struct module_dm_motor
    {
        module_motor_t super;                  // 电机基类
        const module_dm_mode_ops_t *mode_vptr; // 模式操作虚表
        bsp_can_t *can;                        // CAN BSP 基类
        module_dm_control_mode_t control_mode; // 控制模式
        module_dm_limits_t limits;             // 限制参数
        module_dm_mit_command_t mit_command;   // MIT 命令缓存
        float target_position_rad;             // 位置目标（位置速度模式）
        float target_velocity_rad_per_s;       // 速度目标（速度/位置速度模式）
        uint32_t master_identifier;            // 主机标识符
        uint32_t feedback_identifier;          // 反馈标识符
        uint32_t transmit_timeout_ms;          // 发送超时
        module_dm_fault_t fault;               // 当前故障码
        float mos_temperature_c;               // MOS 管温度（℃）
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化达妙电机
     * @param me 电机对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_init(module_dm_motor_t *const me,
                                               const module_dm_motor_config_t *const config);

    /**
     * @brief 注册电机到电机注册表
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_register(module_dm_motor_t *const me,
                                                   module_motor_registry_t *const registry);

    /**
     * @brief 从电机注册表注销电机
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_unregister(module_dm_motor_t *const me,
                                                     module_motor_registry_t *const registry);

    /**
     * @brief 将 module_dm_motor_t 向上转型为 module_motor_t
     * @param me 派生对象
     * @return 基类指针
     */
    module_motor_t *module_dm_motor_as_base(module_dm_motor_t *const me);

    /**
     * @brief 发送状态命令（使能/禁用/保存零位/清除故障）
     * @param me 电机对象
     * @param command 状态命令
     * @return 执行状态
     * @note SAVE_ZERO 会修改驱动器持久状态，需在机械位置确认、输出关闭后调用
     */
    module_motor_status_t module_dm_motor_send_state_command(module_dm_motor_t *const me,
                                                             module_dm_state_command_t command);

    /**
     * @brief 立即执行 MIT 命令（编码并发送）
     * @param me 电机对象
     * @param command MIT 命令
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_command_mit(module_dm_motor_t *const me,
                                                      const module_dm_mit_command_t *const command);

    /**
     * @brief 立即执行速度命令
     * @param me 电机对象
     * @param velocity_rad_per_s 速度目标（rad/s）
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_command_velocity(module_dm_motor_t *const me,
                                                           float velocity_rad_per_s);

    /**
     * @brief 立即执行位置+速度命令
     * @param me 电机对象
     * @param position_rad 位置目标（弧度）
     * @param velocity_rad_per_s 速度目标（rad/s）
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_command_position_velocity(module_dm_motor_t *const me,
                                                                    float position_rad,
                                                                    float velocity_rad_per_s);

    /**
     * @brief 设置 MIT 目标（由统一 update 调度发送）
     * @param me 电机对象
     * @param command MIT 命令
     * @return 执行状态
     */
    module_motor_status_t
    module_dm_motor_set_mit_target(module_dm_motor_t *const me,
                                   const module_dm_mit_command_t *const command);

    /**
     * @brief 设置速度目标（由统一 update 调度发送）
     * @param me 电机对象
     * @param velocity_rad_per_s 速度目标
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_set_velocity_target(module_dm_motor_t *const me,
                                                              float velocity_rad_per_s);

    /**
     * @brief 设置位置+速度目标（由统一 update 调度发送）
     * @param me 电机对象
     * @param position_rad 位置目标
     * @param velocity_rad_per_s 速度目标
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_set_position_velocity_target(module_dm_motor_t *const me,
                                                                       float position_rad,
                                                                       float velocity_rad_per_s);

    /**
     * @brief 处理 CAN 反馈帧
     * @param me 电机对象
     * @param frame CAN 帧
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_handle_feedback(module_dm_motor_t *const me,
                                                          const bsp_can_frame_t *const frame);

    /**
     * @brief 获取当前故障码
     * @param me 电机对象
     * @return 故障码
     */
    module_dm_fault_t module_dm_motor_get_fault(const module_dm_motor_t *const me);

    /**
     * @brief 获取 MOS 管温度
     * @param me 电机对象
     * @return 温度（℃）
     */
    float module_dm_motor_get_mos_temperature_c(const module_dm_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DM_MOTOR_H */