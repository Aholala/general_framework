/**
 * @file module_referee_data.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 裁判系统强类型数据仓库头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 提供比赛状态、机器人状态、底盘功率与枪口热量、位置、
 *       受击、射击和剩余弹量等常用命令的强类型结构。
 *       使用更新位图允许控制任务只消费本周期的新数据。
 *       协议每赛季可能调整，底层帧解析、CRC 和路由保持稳定。
 */

#ifndef MODULE_REFEREE_DATA_H
#define MODULE_REFEREE_DATA_H

#include "module_referee.h" // 依赖裁判系统核心框架

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 命令 ID 宏 ======================== */

#define MODULE_REFEREE_COMMAND_GAME_STATUS (0x0001U)          // 比赛状态
#define MODULE_REFEREE_COMMAND_GAME_RESULT (0x0002U)          // 比赛结果
#define MODULE_REFEREE_COMMAND_ROBOT_HEALTH (0x0003U)         // 机器人血量
#define MODULE_REFEREE_COMMAND_EVENT_DATA (0x0101U)           // 事件数据
#define MODULE_REFEREE_COMMAND_REFEREE_WARNING (0x0104U)      // 裁判系统警告
#define MODULE_REFEREE_COMMAND_ROBOT_STATUS (0x0201U)         // 机器人状态
#define MODULE_REFEREE_COMMAND_POWER_HEAT (0x0202U)           // 功率与热量
#define MODULE_REFEREE_COMMAND_ROBOT_POSITION (0x0203U)       // 机器人位置
#define MODULE_REFEREE_COMMAND_ROBOT_BUFF (0x0204U)           // 机器人增益
#define MODULE_REFEREE_COMMAND_ROBOT_HURT (0x0206U)           // 受击信息
#define MODULE_REFEREE_COMMAND_SHOOT_DATA (0x0207U)           // 射击数据
#define MODULE_REFEREE_COMMAND_PROJECTILE_ALLOWANCE (0x0208U) // 弹量剩余
#define MODULE_REFEREE_COMMAND_RFID_STATUS (0x0209U)          // RFID 状态

    /* ======================== 强类型数据结构 ======================== */

    /**
     * @brief 比赛状态
     */
    typedef struct
    {
        uint8_t game_type;                     // 比赛类型
        uint8_t game_progress;                 // 比赛进度
        uint16_t remaining_time_s;             // 剩余时间（秒）
        uint64_t synchronization_timestamp_us; // 同步时间戳（微秒）
    } module_referee_game_status_t;

    /**
     * @brief 机器人状态
     */
    typedef struct
    {
        uint8_t robot_id;                     // 机器人 ID
        uint8_t robot_level;                  // 机器人等级
        uint16_t current_hp;                  // 当前血量
        uint16_t maximum_hp;                  // 最大血量
        uint16_t shooter_barrel_cooling_rate; // 枪口冷却速率
        uint16_t shooter_barrel_heat_limit;   // 枪口热量上限
        uint16_t chassis_power_limit_w;       // 底盘功率限制（瓦）
        bool gimbal_power_enabled;            // 云台功率是否使能
        bool chassis_power_enabled;           // 底盘功率是否使能
        bool shooter_power_enabled;           // 射击功率是否使能
    } module_referee_robot_status_t;

    /**
     * @brief 功率与热量数据
     */
    typedef struct
    {
        uint16_t chassis_voltage_mv;      // 底盘电压（毫伏）
        uint16_t chassis_current_ma;      // 底盘电流（毫安）
        float chassis_power_w;            // 底盘功率（瓦）
        uint16_t chassis_buffer_energy_j; // 底盘缓冲能量（焦耳）
        uint16_t shooter_17_mm_1_heat;    // 17mm 1 号枪口热量
        uint16_t shooter_17_mm_2_heat;    // 17mm 2 号枪口热量
        uint16_t shooter_42_mm_heat;      // 42mm 枪口热量
    } module_referee_power_heat_t;

    /**
     * @brief 机器人位置
     */
    typedef struct
    {
        float position_x_m; // X 轴位置（米）
        float position_y_m; // Y 轴位置（米）
        float yaw_rad;      // 偏航角（弧度）
    } module_referee_robot_position_t;

    /**
     * @brief 受击信息
     */
    typedef struct
    {
        uint8_t armor_id;  // 装甲板 ID
        uint8_t hurt_type; // 伤害类型
    } module_referee_hurt_t;

    /**
     * @brief 射击数据
     */
    typedef struct
    {
        uint8_t bullet_type;    // 弹丸类型
        uint8_t shooter_number; // 发射器编号
        uint8_t frequency_hz;   // 射速（Hz）
        float speed_m_per_s;    // 弹速（米/秒）
    } module_referee_shoot_process_data_t;

    /**
     * @brief 弹量剩余
     */
    typedef struct
    {
        uint16_t projectile_17_mm_remaining; // 17mm 弹丸剩余
        uint16_t projectile_42_mm_remaining; // 42mm 弹丸剩余
        uint16_t coin_remaining;             // 金币剩余
    } module_referee_projectile_allowance_t;

    /* ======================== 数据仓库对象 ======================== */

    /**
     * @brief 裁判系统数据仓库
     * @note 存储所有已解析的裁判系统数据，使用 update_mask 标记哪些命令已更新
     */
    typedef struct
    {
        module_referee_game_status_t game_status;                   // 比赛状态
        module_referee_robot_status_t robot_status;                 // 机器人状态
        module_referee_power_heat_t power_heat;                     // 功率与热量
        module_referee_robot_position_t robot_position;             // 机器人位置
        module_referee_hurt_t hurt;                                 // 受击信息
        module_referee_shoot_process_data_t shoot_data;                     // 射击数据
        module_referee_projectile_allowance_t projectile_allowance; // 弹量剩余
        uint32_t robot_health[16];                                  // 各机器人血量
        uint32_t event_data;                                        // 事件数据
        uint32_t rfid_status;                                       // RFID 状态
        uint32_t update_mask;        // 更新位图（标记哪些命令有新数据）
        uint32_t decode_error_count; // 解码错误计数
        uint8_t game_result;         // 比赛结果
        uint8_t warning_level;       // 警告等级
        uint8_t warning_robot_id;    // 警告机器人 ID
    } module_referee_process_data_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 重置数据仓库（清零所有字段）
     * @param me 数据仓库对象
     */
    void module_referee_data_reset(module_referee_process_data_t *me);

    /**
     * @brief 裁判系统数据命令路由处理器
     * @param command_id 命令 ID
     * @param payload 负载数据
     * @param payload_size 负载大小
     * @param sequence 序列号
     * @param user_context 用户上下文（数据仓库对象）
     * @note 可作为默认路由或对应命令路由注册到 module_referee
     *       成功解码后设置对应的 update_mask 位
     */
    void module_referee_data_route_handler(uint16_t command_id, const uint8_t *payload,
                                           size_t payload_size, uint8_t sequence,
                                           void *user_context);

    /**
     * @brief 检查指定命令是否已有新数据
     * @param me 数据仓库对象
     * @param command_id 命令 ID（使用 MODULE_REFEREE_COMMAND_xxx 宏）
     * @return true=自上次清除后收到过该命令的新数据
     */
    bool module_referee_data_has_update(const module_referee_process_data_t *me, uint16_t command_id);

    /**
     * @brief 清除所有更新标记
     * @param me 数据仓库对象
     * @note 在完成当前周期数据消费后调用，准备接收下一周期数据
     */
    void module_referee_data_clear_updates(module_referee_process_data_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_REFEREE_DATA_H */