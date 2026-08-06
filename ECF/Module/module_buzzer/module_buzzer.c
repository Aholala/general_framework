/**
 * @file module_buzzer.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 蜂鸣器控制模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 bsp_pwm_t 的有源/无源蜂鸣器控制，支持持续音调、音符序列、
 *       静音、循环播放和非阻塞时间推进。
 */

#include "module_buzzer.h"

#include <math.h>   // isfinite()
#include <stddef.h> // NULL, size_t

MODULE_STATIC_ASSERT_SUPER_FIRST(module_buzzer_t);

/**
 * @brief 应用静音（设置 PWM 占空比为 0）
 * @param me 蜂鸣器对象
 * @return 执行状态
 */
static module_buzzer_status_t module_buzzer_apply_silence(module_buzzer_t *me)
{
    // 设置占空比为 0（静音），若失败则返回传输错误
    return (bsp_pwm_set_duty_cycle(me->pwm, 0.0F) == BSP_STATUS_OK)
               ? MODULE_BUZZER_STATUS_OK
               : MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 应用一个音符（设置频率和占空比）
 * @param me 蜂鸣器对象
 * @param note 音符指针（包含频率）
 * @return 执行状态
 */
static module_buzzer_status_t module_buzzer_apply_note(module_buzzer_t *me,
                                                       const module_buzzer_note_t *note)
{
    // 频率为 0 表示静音
    if (note->frequency_hz == 0U)
    {
        return module_buzzer_apply_silence(me);
    }
    // 设置 PWM 频率和占空比，任一失败返回传输错误
    if ((bsp_pwm_set_frequency(me->pwm, note->frequency_hz) != BSP_STATUS_OK) ||
        (bsp_pwm_set_duty_cycle(me->pwm, me->duty_cycle) != BSP_STATUS_OK))
    {
        return MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_BUZZER_STATUS_OK;
}

/* ======================== module_device 回调函数 ======================== */

/**
 * @brief 设备启动回调（转发至 module_buzzer_start）
 */
static module_device_status_t module_buzzer_device_start(module_device_t *const device_base)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    return (module_buzzer_start(me) == MODULE_BUZZER_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_buzzer_stop）
 */
static module_device_status_t module_buzzer_device_stop(module_device_t *const device_base)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    return (module_buzzer_stop(me) == MODULE_BUZZER_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（转发至 module_buzzer_update）
 */
static module_device_status_t module_buzzer_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_buzzer_t *const me = MODULE_CONTAINER_OF(device_base, module_buzzer_t, super);
    const module_buzzer_status_t status = module_buzzer_update(me, elapsed_time_ms);
    // 无论是 OK 还是 FINISHED 都视为设备状态正常
    return ((status == MODULE_BUZZER_STATUS_OK) || (status == MODULE_BUZZER_STATUS_FINISHED))
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/** 蜂鸣器的设备操作表 */
static const module_device_ops_t s_module_buzzer_ops = {
    .start = module_buzzer_device_start,
    .stop = module_buzzer_device_stop,
    .update = module_buzzer_device_update,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化蜂鸣器模块
 */
module_buzzer_status_t module_buzzer_init(module_buzzer_t *me, const module_buzzer_config_t *config)
{
    // 参数校验：对象、配置、PWM 基类（已初始化）、占空比有效性（有限且 >0 且 <=1）
    if ((me == NULL) || (config == NULL) || (config->pwm == NULL) ||
        !bsp_pwm_is_initialized(config->pwm) || !isfinite(config->duty_cycle) ||
        (config->duty_cycle <= 0.0F) || (config->duty_cycle > 1.0F))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }

    // 清零对象
    *me = (module_buzzer_t){0};

    // 复制配置
    me->pwm = config->pwm;
    me->duty_cycle = config->duty_cycle;

    // 两阶段设备初始化
    if (module_device_init_base(&me->super, &s_module_buzzer_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_BUZZER_STATUS_OK;
}

/**
 * @brief 启动蜂鸣器（使能 PWM 输出，默认静音）
 */
module_buzzer_status_t module_buzzer_start(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    // 启动 PWM 输出
    if (bsp_pwm_start(me->pwm) != BSP_STATUS_OK)
    {
        return MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    // 初始静音
    return module_buzzer_apply_silence(me);
}

/**
 * @brief 停止蜂鸣器（关闭 PWM 输出）
 */
module_buzzer_status_t module_buzzer_stop(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    // 清除播放状态
    me->is_playing = false;
    me->is_started = false;
    // 停止 PWM 输出
    return (bsp_pwm_stop(me->pwm) == BSP_STATUS_OK) ? MODULE_BUZZER_STATUS_OK
                                                    : MODULE_BUZZER_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 播放持续单音（频率固定）
 */
module_buzzer_status_t module_buzzer_play_tone(module_buzzer_t *me, uint32_t frequency_hz)
{
    module_buzzer_note_t note;

    if ((me == NULL) || (frequency_hz == 0U))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }

    // 构造一个只发声不静音的单音音符
    note = (module_buzzer_note_t){
        .frequency_hz = frequency_hz,
        .sound_time_ms = 0U, // 0 表示无限持续（不用于序列）
        .silence_time_ms = 0U,
    };
    me->is_playing = true;
    me->notes = NULL; // 清除序列模式
    return module_buzzer_apply_note(me, &note);
}

/**
 * @brief 播放音符序列（非阻塞）
 */
module_buzzer_status_t module_buzzer_play_sequence(module_buzzer_t *me,
                                                   const module_buzzer_note_t *notes,
                                                   size_t note_count, bool is_looping)
{
    size_t note_index;

    if ((me == NULL) || (notes == NULL) || (note_count == 0U))
    {
        return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }

    // 校验每个音符：频率和发声时间必须大于 0
    for (note_index = 0U; note_index < note_count; ++note_index)
    {
        if ((notes[note_index].frequency_hz == 0U) || (notes[note_index].sound_time_ms == 0U))
        {
            return MODULE_BUZZER_STATUS_INVALID_ARGUMENT;
        }
    }

    // 初始化播放状态
    me->notes = notes;
    me->note_count = note_count;
    me->note_index = 0U;
    me->phase_elapsed_time_ms = 0U;
    me->is_sound_phase = true; // 从发声阶段开始
    me->is_looping = is_looping;
    me->is_playing = true;

    // 应用第一个音符
    return module_buzzer_apply_note(me, &notes[0]);
}

/**
 * @brief 立即静音
 */
module_buzzer_status_t module_buzzer_silence(module_buzzer_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    // 清除播放状态
    me->is_playing = false;
    me->notes = NULL;
    return module_buzzer_apply_silence(me);
}

/**
 * @brief 周期更新（推进播放状态）
 */
module_buzzer_status_t module_buzzer_update(module_buzzer_t *me, uint32_t elapsed_time_ms)
{
    const module_buzzer_note_t *note;
    uint32_t phase_duration_ms;

    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_BUZZER_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_BUZZER_STATUS_NOT_STARTED;
    }
    if (!me->is_playing || (me->notes == NULL))
    {
        return MODULE_BUZZER_STATUS_FINISHED;
    }

    // 累加相位时间
    if (UINT32_MAX - me->phase_elapsed_time_ms < elapsed_time_ms)
    {
        me->phase_elapsed_time_ms = UINT32_MAX; // 饱和
    }
    else
    {
        me->phase_elapsed_time_ms += elapsed_time_ms;
    }

    // 获取当前音符和相位持续时间
    note = &me->notes[me->note_index];
    phase_duration_ms = me->is_sound_phase ? note->sound_time_ms : note->silence_time_ms;

    // 未到切换时间则返回
    if (me->phase_elapsed_time_ms < phase_duration_ms)
    {
        return MODULE_BUZZER_STATUS_OK;
    }

    // 相位切换
    me->phase_elapsed_time_ms = 0U;

    if (me->is_sound_phase && (note->silence_time_ms > 0U))
    {
        // 发声结束，切换到静音相位
        me->is_sound_phase = false;
        return module_buzzer_apply_silence(me);
    }

    // 静音结束或静音时间为 0，切换到下一个音符的发声相位
    me->is_sound_phase = true;
    ++me->note_index;

    if (me->note_index >= me->note_count)
    {
        // 序列播放完毕
        if (!me->is_looping)
        {
            me->is_playing = false;
            me->notes = NULL;
            (void)module_buzzer_apply_silence(me);
            return MODULE_BUZZER_STATUS_FINISHED;
        }
        // 循环播放，重置索引
        me->note_index = 0U;
    }

    // 应用下一个音符
    return module_buzzer_apply_note(me, &me->notes[me->note_index]);
}

/**
 * @brief 查询是否正在播放
 */
bool module_buzzer_is_playing(const module_buzzer_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_playing;
}
