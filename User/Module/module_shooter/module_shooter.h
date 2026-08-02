/**
 * @file module_shooter.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief RoboMaster 发射机构状态机头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 组合左右摩擦轮和拨弹电机，提供摩擦轮启停、排队发射、位置步进、
 *       堵转确认、自动回退、有限重试和故障锁存。
 *       依赖三个 module_motor_t * 对象。
 */

#ifndef MODULE_SHOOTER_H
#define MODULE_SHOOTER_H

#include "module_motor.h" // 电机基类

#include <stdbool.h> // bool
#include <stdint.h>  // uint8_t, uint16_t

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 发射机构模块状态码
     */
    typedef enum
    {
        MODULE_SHOOTER_STATUS_OK = 0,           // 操作成功
        MODULE_SHOOTER_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_SHOOTER_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_SHOOTER_STATUS_NOT_READY,        // 未就绪（电机离线或禁用）
        MODULE_SHOOTER_STATUS_MOTOR_ERROR,      // 电机操作错误
        MODULE_SHOOTER_STATUS_FAULT             // 故障锁存（超过最大重试）
    } module_shooter_status_t;

    /* ======================== 状态机枚举 ======================== */

    /**
     * @brief 发射机构运行状态
     */
    typedef enum
    {
        MODULE_SHOOTER_STATE_DISABLED = 0, // 禁用（所有目标归零）
        MODULE_SHOOTER_STATE_READY,        // 就绪（等待射击）
        MODULE_SHOOTER_STATE_FEEDING,      // 送弹中（拨弹盘运动）
        MODULE_SHOOTER_STATE_ROLLBACK,     // 回退中（堵转后退）
        MODULE_SHOOTER_STATE_FAULT         // 故障锁存
    } module_shooter_state_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 发射机构初始化配置
     * @note 所有阈值需实车标定
     */
    typedef struct
    {
        module_motor_t *left_friction_motor;         // 左摩擦轮电机
        module_motor_t *right_friction_motor;        // 右摩擦轮电机
        module_motor_t *feeder_motor;                // 拨弹电机
        float left_friction_direction_sign;          // 左摩擦轮方向符号（+1 或 -1）
        float right_friction_direction_sign;         // 右摩擦轮方向符号
        float feeder_direction_sign;                 // 拨弹盘方向符号（+1 正向，-1 反向）
        float feeder_step_rad;                       // 单次射击拨弹盘步进角度（弧度）
        float feeder_position_tolerance_rad;         // 拨弹盘位置到达容差（弧度）
        float jam_velocity_threshold_rad_per_s;      // 堵转速度阈值（rad/s，低于此值视为堵转）
        float jam_current_threshold_a;               // 堵转电流阈值（安培，优先使用）
        int16_t jam_current_threshold_raw;           // 堵转电流原始阈值（备用）
        float jam_confirmation_time_s;               // 堵转确认时间（秒）
        float rollback_angle_rad;                    // 堵转回退角度（弧度）
        float rollback_position_tolerance_rad;       // 回退到位容差（弧度）
        float rollback_timeout_s;                    // 单次回退超时（秒）
        float friction_velocity_tolerance_rad_per_s; // 摩擦轮到速容差
        float friction_ready_time_s;                 // 摩擦轮连续到速确认时间
        float fire_stable_time_s;                    // 开火条件连续稳定时间
        float automatic_shot_interval_s;             // 自动射击最小间隔
        uint8_t maximum_jam_retries;                 // 最大堵转重试次数
        uint16_t maximum_pending_shots;              // 最大待发弹量（防溢出）
    } module_shooter_config_t;

    /** @brief 自瞄火控每周期输入；许可只产生新的单发请求，不直接启停拨弹电机 */
    typedef struct
    {
        bool automatic_fire_enabled; // 操作手/上层是否允许自动开火
        bool tracking_ready;         // 视觉/云台层已完成目标与姿态判定
        bool referee_allows_fire;
    } module_shooter_fire_control_input_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 发射机构设备对象
     */
    typedef struct
    {
        module_motor_t *left_friction_motor;    // 左摩擦轮电机
        module_motor_t *right_friction_motor;   // 右摩擦轮电机
        module_motor_t *feeder_motor;           // 拨弹电机
        float left_friction_direction_sign;     // 左摩擦轮方向
        float right_friction_direction_sign;    // 右摩擦轮方向
        float feeder_direction_sign;            // 拨弹盘方向
        float feeder_step_rad;                  // 步进角度
        float feeder_position_tolerance_rad;    // 到位容差
        float jam_velocity_threshold_rad_per_s; // 堵转速度阈值
        float jam_current_threshold_a;          // 堵转电流阈值（安培）
        int16_t jam_current_threshold_raw;      // 堵转电流阈值（原始值）
        float jam_confirmation_time_s;          // 堵转确认时间
        float rollback_angle_rad;               // 回退角度
        float rollback_position_tolerance_rad;  // 回退容差
        float rollback_timeout_s;               // 回退超时
        float rollback_elapsed_time_s;          // 当前回退耗时
        float friction_velocity_tolerance_rad_per_s;
        float friction_ready_time_s;
        float friction_ready_elapsed_time_s;
        float fire_stable_time_s;
        float fire_stable_elapsed_time_s;
        float automatic_shot_interval_s;
        float automatic_shot_elapsed_time_s;
        float friction_target_velocity_rad_per_s; // 摩擦轮目标速度（rad/s）
        float feeder_target_position_rad;         // 拨弹盘目标位置（弧度）
        float feeder_forward_target_position_rad; // 当前单发原始前进目标（回退后恢复）
        float jam_elapsed_time_s;                 // 堵转已累积时间（秒）
        uint16_t pending_shots;                   // 待发射数量
        uint16_t maximum_pending_shots;           // 最大待发弹量
        uint8_t jam_retry_count;                  // 当前堵转重试次数
        uint8_t maximum_jam_retries;              // 最大重试次数
        module_shooter_state_t state;             // 当前状态
        bool friction_enabled;                    // 摩擦轮是否使能
        bool friction_ready;                      // 左右摩擦轮已稳定达到目标转速
        bool fire_permission;                     // 当前自瞄火控许可
        bool is_initialized;                      // 是否已初始化
    } module_shooter_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化发射机构
     * @param me 发射机构对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_shooter_status_t module_shooter_init(module_shooter_t *me,
                                                const module_shooter_config_t *config);

    /**
     * @brief 使能发射机构（使能三个电机，读取拨弹当前位置作为初始目标）
     * @param me 发射机构对象
     * @return 执行状态
     */
    module_shooter_status_t module_shooter_enable(module_shooter_t *me);

    /**
     * @brief 禁用发射机构（禁用三个电机，清空待发队列）
     * @param me 发射机构对象
     * @return 执行状态
     */
    module_shooter_status_t module_shooter_disable(module_shooter_t *me);

    /**
     * @brief 设置摩擦轮使能状态和目标速度
     * @param me 发射机构对象
     * @param is_enabled true=使能，false=停止
     * @param target_velocity_rad_per_s 目标速度（rad/s，非负）
     * @return 执行状态
     */
    module_shooter_status_t module_shooter_set_friction(module_shooter_t *me, bool is_enabled,
                                                        float target_velocity_rad_per_s);

    /**
     * @brief 请求发射指定数量的弹丸（加入队列）
     * @param me 发射机构对象
     * @param shot_count 请求发射数量
     * @return 执行状态
     * @note 会检查是否超过最大待发量
     */
    module_shooter_status_t module_shooter_request_shots(module_shooter_t *me, uint16_t shot_count);

    /**
     * @brief 取消所有待发射请求
     * @param me 发射机构对象
     * @return 执行状态
     * @note 不清除摩擦轮状态
     */
    module_shooter_status_t module_shooter_cancel_shots(module_shooter_t *me);

    /**
     * @brief 清除故障状态（需电机在线且就绪）
     * @param me 发射机构对象
     * @return 执行状态
     * @note 需要所有电机在线、反馈有效，并将目标位置对齐当前反馈
     */
    module_shooter_status_t module_shooter_reset_fault(module_shooter_t *me);

    /**
     * @brief 周期更新状态机（推进送弹、堵转检测、回退等）
     * @param me 发射机构对象
     * @param delta_time_s 时间步长（秒）
     * @return 执行状态
     * @note 只设置电机目标，底层电机仍需由统一调度器 update/flush
     */
    module_shooter_status_t module_shooter_update(module_shooter_t *me, float delta_time_s);

    /**
     * @brief 更新自瞄火控并在满足条件时原子地排入一发
     * @note 视觉许可消失只禁止创建下一发，不会中断已经开始的位置步进
     */
    module_shooter_status_t module_shooter_update_fire_control(
        module_shooter_t *me, const module_shooter_fire_control_input_t *input, float delta_time_s);

    /**
     * @brief 获取当前状态
     * @param me 发射机构对象
     * @return 当前状态
     */
    module_shooter_state_t module_shooter_get_state(const module_shooter_t *me);

    /**
     * @brief 获取待发弹量
     * @param me 发射机构对象
     * @return 待发弹量
     */
    uint16_t module_shooter_get_pending_shots(const module_shooter_t *me);

    /**
     * @brief 获取当前堵重重试次数
     * @param me 发射机构对象
     * @return 重试次数
     */
    uint8_t module_shooter_get_jam_retry_count(const module_shooter_t *me);

    bool module_shooter_get_friction_ready(const module_shooter_t *me);
    bool module_shooter_get_fire_permission(const module_shooter_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SHOOTER_H */
