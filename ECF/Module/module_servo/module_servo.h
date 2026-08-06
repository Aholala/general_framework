/**
 * @file module_servo.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 标准 PWM 舵机控制模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 把弧度角度或归一化命令映射为微秒脉宽，并通过 bsp_pwm_t 输出。
 *       普通 PWM 舵机没有位置反馈，本模块只是开环命令转换。
 */

#ifndef MODULE_SERVO_H
#define MODULE_SERVO_H

#include "bsp_pwm.h"       // PWM BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 舵机模块状态码
     */
    typedef enum
    {
        MODULE_SERVO_STATUS_OK = 0,           // 操作成功
        MODULE_SERVO_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_SERVO_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_SERVO_STATUS_NOT_STARTED,      // 未启动
        MODULE_SERVO_STATUS_TRANSPORT_ERROR   // PWM 操作错误
    } module_servo_status_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 舵机初始化配置
     * @note 中位脉宽单独配置，可适配机械中心不对称的舵机
     */
    typedef struct
    {
        bsp_pwm_t *pwm;               // PWM BSP 基类（必须已初始化）
        uint32_t frequency_hz;        // PWM 频率（Hz），常见 50Hz（舵机）或 400Hz（数字舵机）
        float minimum_pulse_width_us; // 最小脉宽（微秒），对应 minimum_angle
        float neutral_pulse_width_us; // 中位脉宽（微秒），对应 0°/中位
        float maximum_pulse_width_us; // 最大脉宽（微秒），对应 maximum_angle
        float minimum_angle_rad;      // 最小机械角度（弧度）
        float maximum_angle_rad;      // 最大机械角度（弧度）
        const char *logical_name;     // 设备逻辑名称
        uint32_t registration_key;    // 注册键值
    } module_servo_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 舵机设备对象
     */
    typedef struct
    {
        module_device_t super;        // 设备基类
        bsp_pwm_t *pwm;               // PWM BSP 基类
        uint32_t frequency_hz;        // PWM 频率（Hz）
        float minimum_pulse_width_us; // 最小脉宽（微秒）
        float neutral_pulse_width_us; // 中位脉宽（微秒）
        float maximum_pulse_width_us; // 最大脉宽（微秒）
        float minimum_angle_rad;      // 最小机械角度（弧度）
        float maximum_angle_rad;      // 最大机械角度（弧度）
        float commanded_angle_rad;    // 最后一次命令角度（弧度），不是实测值
        bool is_started;              // 是否已启动
    } module_servo_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化舵机模块
     * @param me 舵机对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_servo_status_t module_servo_init(module_servo_t *me,
                                            const module_servo_config_t *config);

    /**
     * @brief 启动舵机（配置频率并输出中位脉宽）
     * @param me 舵机对象
     * @return 执行状态
     */
    module_servo_status_t module_servo_start(module_servo_t *me);

    /**
     * @brief 停止舵机（关闭 PWM 输出）
     * @param me 舵机对象
     * @return 执行状态
     */
    module_servo_status_t module_servo_stop(module_servo_t *me);

    /**
     * @brief 设置舵机角度（弧度）
     * @param me 舵机对象
     * @param angle_rad 目标角度（弧度）
     * @return 执行状态
     * @note 角度会被钳位到配置的最小/最大角度范围
     *       舵机无反馈，不能将命令角度视为实际角度
     */
    module_servo_status_t module_servo_set_angle(module_servo_t *me, float angle_rad);

    /**
     * @brief 设置归一化输出（速度/位置控制器输出）
     * @param me 舵机对象
     * @param normalized_output 归一化值 [-1.0, 1.0]
     * @return 执行状态
     * @note positive → 中位到最大脉宽，negative → 中位到最小脉宽
     */
    module_servo_status_t module_servo_set_normalized_output(module_servo_t *me,
                                                             float normalized_output);

    /**
     * @brief 直接设置脉冲宽度（微秒）
     * @param me 舵机对象
     * @param pulse_width_us 脉宽（微秒）
     * @return 执行状态
     * @note 脉宽会被校验是否在最小/最大范围内
     */
    module_servo_status_t module_servo_set_pulse_width(module_servo_t *me, float pulse_width_us);

    /**
     * @brief 获取最后一次命令角度
     * @param me 舵机对象
     * @param angle_rad 输出角度（弧度）
     * @return 执行状态
     * @note 返回的是命令值，不是传感器实测角度
     */
    module_servo_status_t module_servo_get_commanded_angle(const module_servo_t *me,
                                                           float *angle_rad);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_SERVO_H */