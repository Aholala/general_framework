/**
 * @file module_dji_motor.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 大疆 M2006、M3508 和 GM6020 电机 CAN 协议驱动头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 负责总线注册、反馈解码、编码器多圈累计、控制模式和分组电流帧发送。
 *       具体型号派生模块只补充型号参数与专用语义。
 */

#ifndef MODULE_DJI_MOTOR_H
#define MODULE_DJI_MOTOR_H

#include "alg_pid.h"      // PID 控制器算法
#include "bsp_can.h"      // CAN BSP 抽象层
#include "module_motor.h" // 电机基类

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 宏定义 ======================== */

/** @brief CAN 发送组数量（3 组：0x1FF, 0x200, 0x2FF） */
#define MODULE_DJI_MOTOR_GROUP_COUNT (3U)
/** @brief 每组电机数量（4 个） */
#define MODULE_DJI_MOTOR_PER_GROUP (4U)

    /* ======================== 前向声明 ======================== */

    typedef struct module_dji_motor module_dji_motor_t;

    /* ======================== 枚举类型 ======================== */

    /**
     * @brief DJI 电机型号
     */
    typedef enum
    {
        MODULE_DJI_MOTOR_M2006 = 0, // M2006 直流无刷电机（36:1 减速）
        MODULE_DJI_MOTOR_M3508,     // M3508 直流无刷电机（19:1 减速）
        MODULE_DJI_MOTOR_GM6020     // GM6020 云台电机（直驱，1:1）
    } module_dji_motor_model_t;

    /**
     * @brief DJI 电机控制模式
     */
    typedef enum
    {
        MODULE_DJI_CONTROL_DIRECT = 0, // 直通模式：目标值直接作为电流命令
        MODULE_DJI_CONTROL_VELOCITY,   // 速度控制：速度 PID
        MODULE_DJI_CONTROL_POSITION    // 位置控制：位置/速度串级 PID
    } module_dji_control_mode_t;

    /* ======================== 总线对象 ======================== */

    /**
     * @brief DJI 电机总线对象
     * @note 每条 CAN 网络创建一个总线对象，管理 3 个发送组 × 4 个槽位
     */
    typedef struct
    {
        bsp_can_t *can; // CAN BSP 基类
        module_dji_motor_t
            *motor_slots[MODULE_DJI_MOTOR_GROUP_COUNT][MODULE_DJI_MOTOR_PER_GROUP]; // 电机槽位
        bool group_is_used[MODULE_DJI_MOTOR_GROUP_COUNT]; // 各组是否被使用
        uint32_t transmit_timeout_ms;                     // CAN 发送超时（毫秒）
        bool is_initialized;                              // 是否已初始化
    } module_dji_motor_bus_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief DJI 电机初始化配置
     */
    typedef struct
    {
        const char *logical_name;                     // 逻辑名称
        uint32_t registration_key;                    // 注册键值
        module_dji_motor_bus_t *motor_bus;            // 所属总线
        module_dji_motor_model_t motor_model;         // 电机型号
        module_dji_control_mode_t control_mode;       // 控制模式
        uint8_t motor_identifier;                     // 电机标识符（1~8）
        float direction_sign;                         // 方向符号（+1 或 -1）
        float maximum_temperature_c;                  // 最大允许温度（℃）
        float current_scale_a_per_count;              // 电流换算因子（A/原始值）
        alg_pid_config_t velocity_pid_config;         // 速度 PID 配置
        alg_pid_cascade_config_t position_pid_config; // 位置串级 PID 配置
    } module_dji_motor_config_t;

    /* ======================== 电机对象 ======================== */

    /**
     * @brief DJI 电机设备对象
     */
    struct module_dji_motor
    {
        module_motor_t super;                   // 电机基类
        module_dji_motor_bus_t *motor_bus;      // 所属总线
        module_dji_motor_model_t motor_model;   // 电机型号
        module_dji_control_mode_t control_mode; // 控制模式
        alg_pid_t velocity_controller;          // 速度 PID 控制器
        alg_pid_cascade_t position_controller;  // 位置串级 PID 控制器
        float target_value;                     // 目标值（含义取决于控制模式）
        float direction_sign;                   // 方向符号
        float gear_ratio;                       // 减速比
        float maximum_temperature_c;            // 最大允许温度
        float current_scale_a_per_count;        // 电流换算因子
        int16_t command_value;                  // 当前命令值（CAN 电流命令）
        int16_t maximum_command_value;          // 最大命令值（型号相关）
        uint16_t previous_encoder_count;        // 上次编码器值（用于回绕计算）
        int64_t accumulated_encoder_count;      // 累积编码器值（多圈）
        uint32_t receive_identifier;            // CAN 接收 ID
        uint8_t group_index;                    // 发送组索引（0,1,2）
        uint8_t group_slot;                     // 组内槽位（0~3）
        bool has_previous_encoder_count;        // 是否有上次编码器值（首次反馈后置 true）
    };

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 DJI 电机总线
     * @param me 总线对象
     * @param can CAN BSP 基类
     * @param transmit_timeout_ms CAN 发送超时
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_bus_init(module_dji_motor_bus_t *const me,
                                                    bsp_can_t *const can,
                                                    uint32_t transmit_timeout_ms);

    /**
     * @brief 初始化 DJI 电机实例
     * @param me 电机对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_init(module_dji_motor_t *const me,
                                                const module_dji_motor_config_t *const config);

    /**
     * @brief 注册电机到总线槽位和电机注册表
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_register(module_dji_motor_t *const me,
                                                    module_motor_registry_t *const registry);

    /**
     * @brief 从总线槽位和电机注册表注销电机
     * @param me 电机对象
     * @param registry 电机注册表
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_unregister(module_dji_motor_t *const me,
                                                      module_motor_registry_t *const registry);

    /**
     * @brief 将 module_dji_motor_t 向上转型为 module_motor_t
     * @param me 派生对象
     * @return 基类指针
     */
    module_motor_t *module_dji_motor_as_base(module_dji_motor_t *const me);

    /**
     * @brief 处理 CAN 反馈帧
     * @param me 总线对象
     * @param frame CAN 帧
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_bus_handle_feedback(module_dji_motor_bus_t *const me,
                                                               const bsp_can_frame_t *const frame);

    /**
     * @brief 刷新总线：将各槽位命令打包成 CAN 帧发送
     * @param me 总线对象
     * @return 执行状态
     */
    module_motor_status_t module_dji_motor_bus_flush(module_dji_motor_bus_t *const me);

    /**
     * @brief 获取当前命令值（用于调试）
     * @param me 电机对象
     * @return 当前命令值
     */
    int16_t module_dji_motor_get_command(const module_dji_motor_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DJI_MOTOR_H */