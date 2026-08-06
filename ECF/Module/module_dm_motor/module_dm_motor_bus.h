/**
 * @file module_dm_motor_bus.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 达妙电机总线管理头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 为达妙多实例提供反馈 ID 路由、重复 ID 检查和轮询发送预算。
 *       maximum_transmits_per_cycle 用于限制一个控制周期内的独立 CAN 帧数量。
 */

#ifndef MODULE_DM_MOTOR_BUS_H
#define MODULE_DM_MOTOR_BUS_H

#include "module_dm_motor.h" // 达妙电机对象

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief 达妙电机总线对象
     * @note 管理多个达妙电机实例，提供反馈路由和轮询发送
     */
    typedef struct
    {
        bsp_can_t *can;                     // CAN BSP 基类
        module_dm_motor_t **motor_storage;  // 电机数组（由调用者分配）
        size_t motor_capacity;              // 数组容量
        size_t motor_count;                 // 当前电机数量
        size_t next_transmit_index;         // 下一个要发送的电机索引（轮询）
        size_t maximum_transmits_per_cycle; // 每周期最大发送帧数
        uint32_t routed_frame_count;        // 成功路由的反馈帧计数
        uint32_t unknown_frame_count;       // 未知反馈帧计数
        uint32_t transmit_error_count;      // 发送错误计数
        bool is_initialized;                // 是否已初始化
    } module_dm_motor_bus_t;

    /**
     * @brief 初始化达妙电机总线
     * @param me 总线对象
     * @param can CAN BSP 基类
     * @param motor_storage 电机存储数组（由调用者分配）
     * @param motor_capacity 数组容量
     * @param maximum_transmits_per_cycle 每周期最大发送帧数
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_bus_init(module_dm_motor_bus_t *me, bsp_can_t *can,
                                                   module_dm_motor_t **motor_storage,
                                                   size_t motor_capacity,
                                                   size_t maximum_transmits_per_cycle);

    /**
     * @brief 注册电机到总线
     * @param me 总线对象
     * @param motor 电机对象
     * @return 执行状态
     * @note 检查重复的反馈 ID 和发送 ID
     */
    module_motor_status_t module_dm_motor_bus_register(module_dm_motor_bus_t *me,
                                                       module_dm_motor_t *motor);

    /**
     * @brief 从总线注销电机
     * @param me 总线对象
     * @param motor 电机对象
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_bus_unregister(module_dm_motor_bus_t *me,
                                                         module_dm_motor_t *motor);

    /**
     * @brief 处理 CAN 反馈帧（路由到对应的电机）
     * @param me 总线对象
     * @param frame CAN 帧
     * @return 执行状态
     */
    module_motor_status_t module_dm_motor_bus_handle_feedback(module_dm_motor_bus_t *me,
                                                              const bsp_can_frame_t *frame);

    /**
     * @brief 更新总线（轮询发送电机命令）
     * @param me 总线对象
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 每周期最多发送 maximum_transmits_per_cycle 帧
     */
    module_motor_status_t module_dm_motor_bus_update(module_dm_motor_bus_t *me, float delta_time_s);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DM_MOTOR_BUS_H */