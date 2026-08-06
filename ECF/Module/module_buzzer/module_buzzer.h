/**
 * @file module_buzzer.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 蜂鸣器控制模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_pwm_t 的有源/无源蜂鸣器控制，支持持续音调、音符序列、
 *       静音、循环播放和非阻塞时间推进。
 */

#ifndef MODULE_BUZZER_H
#define MODULE_BUZZER_H

#include "bsp_pwm.h"       // PWM BSP 抽象层（提供频率和占空比控制）
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief 蜂鸣器模块状态码
     */
    typedef enum
    {
        MODULE_BUZZER_STATUS_OK = 0,           // 操作成功（正在播放）
        MODULE_BUZZER_STATUS_FINISHED,         // 序列播放完成（非循环模式）
        MODULE_BUZZER_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_BUZZER_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_BUZZER_STATUS_NOT_STARTED,      // 未启动（未调用 start）
        MODULE_BUZZER_STATUS_TRANSPORT_ERROR   // PWM 操作错误
    } module_buzzer_status_t;

    /* ======================== 音符结构体 ======================== */

    /**
     * @brief 音符定义（用于序列播放）
     */
    typedef struct
    {
        uint32_t frequency_hz;    // 频率（Hz），0 表示静音（但通常用 silence 函数）
        uint32_t sound_time_ms;   // 发声持续时间（毫秒），必须 > 0
        uint32_t silence_time_ms; // 发声后的静音持续时间（毫秒），可为 0
    } module_buzzer_note_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief 蜂鸣器初始化配置
     */
    typedef struct
    {
        bsp_pwm_t *pwm;            // PWM BSP 基类（必须已初始化）
        const char *logical_name;  // 设备逻辑名称
        uint32_t registration_key; // 注册键值（用于设备管理）
        float duty_cycle;          // PWM 占空比（0.0~1.0），无源蜂鸣器常用 0.5
    } module_buzzer_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief 蜂鸣器设备对象
     */
    typedef struct
    {
        module_device_t super;             // 设备基类
        bsp_pwm_t *pwm;                    // PWM BSP 基类
        const module_buzzer_note_t *notes; // 当前播放的序列（外部引用，不复制）
        size_t note_count;                 // 序列长度
        size_t note_index;                 // 当前播放的音符索引
        uint32_t phase_elapsed_time_ms;    // 当前相位（发声/静音）已累积时间（ms）
        float duty_cycle;                  // PWM 占空比
        bool is_sound_phase;               // 当前是否在发声阶段（true=发声，false=静音）
        bool is_looping;                   // 是否循环播放
        bool is_playing;                   // 是否正在播放
        bool is_started;                   // 是否已启动（PWM 已使能）
    } module_buzzer_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化蜂鸣器模块
     * @param me 蜂鸣器对象
     * @param config 初始化配置
     * @return 执行状态
     */
    module_buzzer_status_t module_buzzer_init(module_buzzer_t *me,
                                              const module_buzzer_config_t *config);

    /**
     * @brief 启动蜂鸣器（使能 PWM 输出，默认静音）
     * @param me 蜂鸣器对象
     * @return 执行状态
     */
    module_buzzer_status_t module_buzzer_start(module_buzzer_t *me);

    /**
     * @brief 停止蜂鸣器（关闭 PWM 输出）
     * @param me 蜂鸣器对象
     * @return 执行状态
     */
    module_buzzer_status_t module_buzzer_stop(module_buzzer_t *me);

    /**
     * @brief 播放持续单音（频率固定，直到调用 silence 或 stop）
     * @param me 蜂鸣器对象
     * @param frequency_hz 频率（Hz），必须大于 0
     * @return 执行状态
     */
    module_buzzer_status_t module_buzzer_play_tone(module_buzzer_t *me, uint32_t frequency_hz);

    /**
     * @brief 播放音符序列（非阻塞）
     * @param me 蜂鸣器对象
     * @param notes 音符数组指针（外部引用，播放期间必须保持有效）
     * @param note_count 音符数量
     * @param is_looping 是否循环播放
     * @return 执行状态
     * @note 音符数组推荐 static const，播放期间不可释放
     */
    module_buzzer_status_t module_buzzer_play_sequence(module_buzzer_t *me,
                                                       const module_buzzer_note_t *notes,
                                                       size_t note_count, bool is_looping);

    /**
     * @brief 立即静音（停止当前播放并关闭声音）
     * @param me 蜂鸣器对象
     * @return 执行状态
     */
    module_buzzer_status_t module_buzzer_silence(module_buzzer_t *me);

    /**
     * @brief 周期更新（推进播放状态）
     * @param me 蜂鸣器对象
     * @param elapsed_time_ms 距上次调用的时间（毫秒）
     * @return 执行状态（FINISHED 表示非循环序列播放完毕）
     * @note 应在主循环或任务中周期性调用
     */
    module_buzzer_status_t module_buzzer_update(module_buzzer_t *me, uint32_t elapsed_time_ms);

    /**
     * @brief 查询是否正在播放
     * @param me 蜂鸣器对象
     * @return true=正在播放，false=停止或静音
     */
    bool module_buzzer_is_playing(const module_buzzer_t *me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_BUZZER_H */