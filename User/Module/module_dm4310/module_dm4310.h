/**
 * @file module_dm4310.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙 DM4310 电机专用派生模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 继承自 module_dm_motor_t，支持 MIT、位置速度、速度和力位混合四种控制模式。
 *       协议范围（PMAX、VMAX、TMAX 等）由配置显式传入，不提供危险默认值。
 *       保存零点和清除故障通过专用接口调用，必须在电机禁用状态下执行。
 */

#ifndef MODULE_DM4310_H
#define MODULE_DM4310_H

#include "module_dm_motor.h" // 达妙电机基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明 */
    typedef struct module_dm4310 module_dm4310_t;

    /* ======================== 控制模式枚举 ======================== */

    /**
     * @brief DM4310 控制模式（型号专用枚举）
     * @note 与 module_dm_control_mode_t 一一对应，但独立命名以避免混淆
     *       使用 module_dm4310_map_control_mode() 映射到通用达妙模式
     */
    typedef enum
    {
        MODULE_DM4310_CONTROL_MIT = 0,          // MIT 模式（位置+速度+Kp+Kd+扭矩）
        MODULE_DM4310_CONTROL_VELOCITY,         // 速度模式
        MODULE_DM4310_CONTROL_POSITION_VELOCITY, // 位置+速度模式
        MODULE_DM4310_CONTROL_FORCE_POSITION     // 力位混合模式
    } module_dm4310_control_mode_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief DM4310 初始化配置
     * @note protocol_limits 必须与电机固件实际参数一致。
     *       PMAX、VMAX、TMAX 等可通过调试工具或寄存器修改，不能写死默认值。
     *       配置中应使用从调试工具读取的实际值。
     */
    typedef struct
    {
        const char *logical_name;                  // 逻辑名称
        uint32_t registration_key;                 // 注册键值
        bsp_can_t *can;                            // CAN BSP 基类
        module_dm4310_control_mode_t control_mode; // 控制模式
        uint32_t base_command_identifier;          // MIT 模式命令 ID 基址（其他模式自动偏移）
        uint32_t feedback_identifier;              // 反馈 CAN ID（必须与电机配置一致）
        uint32_t transmit_timeout_ms;              // CAN 发送超时（毫秒）
        module_dm_limits_t protocol_limits;        // 协议范围（位置/速度/力矩/Kp/Kd 的 min/max）
    } module_dm4310_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief DM4310 设备对象
     * @note 直接继承 module_dm_motor_t，无额外字段
     *       继承链：module_motor_t → module_dm_motor_t → module_dm4310_t
     */
    struct module_dm4310
    {
        module_dm_motor_t super; // 达妙电机基类
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 DM4310 电机
     * @param me 电机对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_init(module_dm4310_t *const me,
                                             const module_dm4310_config_t *const config);

    /**
     * @brief 注册电机到电机注册表
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_register(module_dm4310_t *const me,
                                                 module_motor_registry_t *const registry);

    /**
     * @brief 从电机注册表注销电机
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_unregister(module_dm4310_t *const me,
                                                   module_motor_registry_t *const registry);

    /**
     * @brief 向上转型为 module_motor_t 基类指针
     * @param me 电机对象
     * @return 基类指针
     */
    module_motor_t *module_dm4310_as_motor(module_dm4310_t *const me);

    /**
     * @brief 向上转型为 module_dm_motor_t 基类指针
     * @param me 电机对象
     * @return 达妙电机基类指针
     */
    module_dm_motor_t *module_dm4310_as_dm_motor(module_dm4310_t *const me);

    /**
     * @brief 使能电机
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_enable(module_dm4310_t *const me);

    /**
     * @brief 禁用电机
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_disable(module_dm4310_t *const me);

    /**
     * @brief 执行 MIT 模式命令
     * @param me 电机对象
     * @param command MIT 命令
     * @return 执行状态
     * @note 仅当控制模式为 MIT 时有效
     */
    module_motor_status_t module_dm4310_command_mit(module_dm4310_t *const me,
                                                    const module_dm_mit_command_t *const command);

    /**
     * @brief 执行速度模式命令
     * @param me 电机对象
     * @param velocity_rad_per_s 速度目标（rad/s）
     * @return 执行状态
     * @note 仅当控制模式为 VELOCITY 时有效
     */
    module_motor_status_t module_dm4310_command_velocity(module_dm4310_t *const me,
                                                         float velocity_rad_per_s);

    /**
     * @brief 执行位置速度模式命令
     * @param me 电机对象
     * @param position_rad 位置目标（弧度）
     * @param velocity_rad_per_s 速度目标（rad/s）
     * @return 执行状态
     * @note 仅当控制模式为 POSITION_VELOCITY 时有效
     */
    module_motor_status_t module_dm4310_command_position_velocity(module_dm4310_t *const me,
                                                                  float position_rad,
                                                                  float velocity_rad_per_s);

    /**
     * @brief 执行力位混合模式命令
     * @param me 电机对象
     * @param command 位置、速度限制和电流限制命令
     * @return 执行状态
     */
    module_motor_status_t
    module_dm4310_command_force_position(module_dm4310_t *const me,
                                         const module_dm_force_position_command_t *const command);

    /**
     * @brief 将当前输出轴位置设为零位
     * @param me 电机对象
     * @return 执行状态
     * @note 仅允许在 DISABLED 状态下调用
     */
    module_motor_status_t module_dm4310_set_zero_position(module_dm4310_t *const me);

    /**
     * @brief 兼容旧接口：将当前输出轴位置设为零位
     * @param me 电机对象
     * @return 执行状态
     * @note 建议新代码使用 module_dm4310_set_zero_position()
     */
    module_motor_status_t module_dm4310_save_zero_position(module_dm4310_t *const me);

    /**
     * @brief 清除驱动器故障
     * @param me 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_clear_fault(module_dm4310_t *const me);

    /**
     * @brief 处理 CAN 反馈帧
     * @param me 电机对象
     * @param frame CAN 帧
     * @return 执行状态
     */
    module_motor_status_t module_dm4310_handle_feedback(module_dm4310_t *const me,
                                                        const bsp_can_frame_t *const frame);

    /**
     * @brief 获取电机反馈数据
     * @param me 电机对象
     * @return 反馈数据指针，未初始化或离线返回 NULL
     */
    const module_motor_feedback_t *module_dm4310_get_feedback(const module_dm4310_t *const me);

    /**
     * @brief 获取故障码
     * @param me 电机对象
     * @return 故障码
     */
    module_dm_fault_t module_dm4310_get_fault(const module_dm4310_t *const me);

    /**
     * @brief 获取 MOS 管温度
     * @param me 电机对象
     * @return 温度（℃）
     */
    float module_dm4310_get_mos_temperature_c(const module_dm4310_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DM4310_H */
