/**
 * @file module_ws2812.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief WS2812 灯带驱动模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 通过 SPI 以编码波形驱动 WS2812 灯带，支持像素缓冲、全局亮度、
 *       异步传输、内置灯光效果引擎。
 */

#include "module_ws2812.h"

#include <stddef.h> // NULL, size_t
#include <string.h> // memcpy, memset

/* ======================== 内部辅助函数 ======================== */

/**
 * @brief 对颜色分量应用全局亮度（软件缩放）
 * @param component 原始分量值（0~255）
 * @param brightness 全局亮度（0~255）
 * @return 缩放后的分量值（0~255）
 * @note 使用 `(component * brightness + 127) / 255` 四舍五入
 */
static uint8_t module_ws2812_scale_component(uint8_t component, uint8_t brightness)
{
    return (uint8_t)(((uint16_t)component * brightness + 127U) / 255U);
}

/**
 * @brief 对完整颜色应用全局亮度
 * @param color 原始颜色
 * @param brightness 亮度值
 * @return 缩放后的颜色
 */
static module_ws2812_color_t module_ws2812_scale_color(module_ws2812_color_t color,
                                                       uint8_t brightness)
{
    return module_ws2812_make_color(module_ws2812_scale_component(color.red, brightness),
                                    module_ws2812_scale_component(color.green, brightness),
                                    module_ws2812_scale_component(color.blue, brightness));
}

/**
 * @brief 配置效果（通用内部函数）
 * @param me 设备对象
 * @param effect_type 效果类型
 * @param color 颜色（部分效果使用）
 * @param step_time_ms 步进时间（毫秒）
 * @return 执行状态
 */
static module_ws2812_status_t module_ws2812_configure_effect(module_ws2812_t *me,
                                                             module_ws2812_effect_t effect_type,
                                                             module_ws2812_color_t color,
                                                             uint32_t step_time_ms)
{
    // 参数校验
    if ((me == NULL) || (step_time_ms == 0U) || (effect_type == MODULE_WS2812_EFFECT_NONE) ||
        (effect_type > MODULE_WS2812_EFFECT_THEATER_CHASE))
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }

    // 初始化效果状态
    me->effect = (module_ws2812_effect_state_t){
        .type = effect_type,
        .color = color,
        .step_time_ms = step_time_ms,
        .brightness = 0U,
        .brightness_direction = 1,
        .is_enabled = true,
    };
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 将单个字节编码为 3 个 SPI 字节（WS2812 时序）
 * @param value 原始字节（0~255）
 * @param encoded_bytes 输出数组（3 字节）
 * @note 每 bit：1 编码为 0b110（3 个 SPI bit），0 编码为 0b100
 *       具体时序依赖 SPI 频率，此处仅作逻辑编码
 */
static void module_ws2812_encode_byte(uint8_t value, uint8_t encoded_bytes[3])
{
    uint32_t encoded_value = 0U;
    uint8_t bit_index;

    // 逐 bit 编码，高位在前
    for (bit_index = 0U; bit_index < 8U; ++bit_index)
    {
        encoded_value <<= 3U; // 腾出 3 位空间
        // 当前 bit 为 1 时编码为 0b110（6），否则为 0b100（4）
        encoded_value |= ((value & (uint8_t)(0x80U >> bit_index)) != 0U) ? 0x06U : 0x04U;
    }
    // 拆分为 3 个字节（大端序）
    encoded_bytes[0] = (uint8_t)(encoded_value >> 16U);
    encoded_bytes[1] = (uint8_t)(encoded_value >> 8U);
    encoded_bytes[2] = (uint8_t)encoded_value;
}

/* ======================== module_device 回调函数 ======================== */

/**
 * @brief 设备启动回调
 */
static module_device_status_t module_ws2812_device_start(module_device_t *const device_base)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    return (module_ws2812_start(me) == MODULE_WS2812_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调
 */
static module_device_status_t module_ws2812_device_stop(module_device_t *const device_base)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    return (module_ws2812_stop(me) == MODULE_WS2812_STATUS_OK)
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（调用效果更新）
 */
static module_device_status_t module_ws2812_device_update(module_device_t *const device_base,
                                                          uint32_t elapsed_time_ms)
{
    module_ws2812_t *const me = MODULE_CONTAINER_OF(device_base, module_ws2812_t, super);
    const module_ws2812_status_t status = module_ws2812_update(me, elapsed_time_ms);
    return ((status == MODULE_WS2812_STATUS_OK) || (status == MODULE_WS2812_STATUS_BUSY))
               ? MODULE_DEVICE_STATUS_OK
               : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/** WS2812 的设备操作表 */
static const module_device_ops_t s_module_ws2812_ops = {
    .start = module_ws2812_device_start,
    .stop = module_ws2812_device_stop,
    .update = module_ws2812_device_update,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 WS2812 设备
 */
module_ws2812_status_t module_ws2812_init(module_ws2812_t *me, const module_ws2812_config_t *config)
{
    size_t required_buffer_size;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->spi == NULL) ||
        !bsp_device_is_initialized(&config->spi->super) || (config->pixels == NULL) ||
        (config->led_count == 0U) || (config->transmit_buffer == NULL) ||
        (config->reset_byte_count == 0U) || !bsp_transfer_mode_is_valid(config->transfer_mode))
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }

    // 检查发送缓冲区是否足够大
    required_buffer_size =
        MODULE_WS2812_REQUIRED_BUFFER_SIZE(config->led_count, config->reset_byte_count);
    if (config->transmit_buffer_size < required_buffer_size)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (module_ws2812_t){0};
    me->spi = config->spi;
    me->pixels = config->pixels;
    me->led_count = config->led_count;
    me->transmit_buffer = config->transmit_buffer;
    me->transmit_buffer_size = required_buffer_size;
    me->reset_byte_count = config->reset_byte_count;
    me->transmit_timeout_ms = config->transmit_timeout_ms;
    me->transfer_mode = config->transfer_mode;
    me->brightness = 255U;

    // 清空像素和发送缓冲区
    memset(me->pixels, 0, me->led_count * sizeof(*me->pixels));
    memset(me->transmit_buffer, 0, required_buffer_size);

    // ---- 初始化基类 ----
    if (module_device_init_base(&me->super, &s_module_ws2812_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 启动灯带
 */
module_ws2812_status_t module_ws2812_start(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->is_started = true;
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 停止灯带
 */
module_ws2812_status_t module_ws2812_stop(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    // 如果正在发送，中止 SPI
    if (me->is_busy)
    {
        (void)bsp_spi_abort(me->spi);
    }
    me->is_busy = false;
    me->effect.is_enabled = false;
    me->is_started = false;
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 设置单个 LED
 */
module_ws2812_status_t module_ws2812_set_pixel(module_ws2812_t *me, size_t led_index,
                                               module_ws2812_color_t color)
{
    if (me == NULL)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (led_index >= me->led_count)
    {
        return MODULE_WS2812_STATUS_INVALID_ARGUMENT;
    }
    me->pixels[led_index] = color;
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 填充所有 LED
 */
module_ws2812_status_t module_ws2812_fill(module_ws2812_t *me, module_ws2812_color_t color)
{
    size_t led_index;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    for (led_index = 0U; led_index < me->led_count; ++led_index)
    {
        me->pixels[led_index] = color;
    }
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 清空所有 LED
 */
module_ws2812_status_t module_ws2812_clear(module_ws2812_t *me)
{
    return module_ws2812_fill(me, (module_ws2812_color_t){0U, 0U, 0U});
}

/**
 * @brief 设置全局亮度
 */
module_ws2812_status_t module_ws2812_set_brightness(module_ws2812_t *me, uint8_t brightness)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->brightness = brightness;
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 编码并发送像素数据
 */
module_ws2812_status_t module_ws2812_show(module_ws2812_t *me)
{
    size_t led_index;
    size_t output_index = 0U;
    bsp_status_t status;

    // ---- 状态检查 ----
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }
    if (me->is_busy)
    {
        return MODULE_WS2812_STATUS_BUSY;
    }

    // ---- 编码每颗 LED ----
    for (led_index = 0U; led_index < me->led_count; ++led_index)
    {
        uint8_t encoded_bytes[3];
        const module_ws2812_color_t color = me->pixels[led_index];
        // WS2812 的数据顺序为 GRB，因此先编码绿色，再红色，再蓝色
        const uint8_t components[3] = {
            module_ws2812_scale_component(color.green, me->brightness),
            module_ws2812_scale_component(color.red, me->brightness),
            module_ws2812_scale_component(color.blue, me->brightness),
        };
        size_t component_index;

        for (component_index = 0U; component_index < 3U; ++component_index)
        {
            module_ws2812_encode_byte(components[component_index], encoded_bytes);
            // 拷贝编码后的 3 字节到发送缓冲区
            memcpy(&me->transmit_buffer[output_index], encoded_bytes, sizeof(encoded_bytes));
            output_index += sizeof(encoded_bytes);
        }
    }

    // ---- 追加复位低电平 ----
    memset(&me->transmit_buffer[output_index], 0, me->reset_byte_count);

    // ---- 发送 ----
    me->is_busy = me->transfer_mode != BSP_TRANSFER_MODE_BLOCKING; // 非阻塞模式标记忙
    status = bsp_spi_transmit(me->spi, me->transmit_buffer, me->transmit_buffer_size,
                              me->transfer_mode, me->transmit_timeout_ms);
    if (status != BSP_STATUS_OK)
    {
        me->is_busy = false;
        return MODULE_WS2812_STATUS_TRANSPORT_ERROR;
    }
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief 周期更新效果
 */
module_ws2812_status_t module_ws2812_update(module_ws2812_t *me, uint32_t elapsed_time_ms)
{
    module_ws2812_effect_state_t *effect;

    // ---- 状态检查 ----
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_WS2812_STATUS_NOT_STARTED;
    }
    if (!me->effect.is_enabled)
    {
        return MODULE_WS2812_STATUS_OK;
    }
    if (me->is_busy)
    {
        return MODULE_WS2812_STATUS_BUSY;
    }

    effect = &me->effect;

    // 累积时间（防溢出）
    if (UINT32_MAX - effect->elapsed_time_ms < elapsed_time_ms)
    {
        effect->elapsed_time_ms = UINT32_MAX;
    }
    else
    {
        effect->elapsed_time_ms += elapsed_time_ms;
    }

    // 未到步进时间则等待
    if (effect->elapsed_time_ms < effect->step_time_ms)
    {
        return MODULE_WS2812_STATUS_OK;
    }
    effect->elapsed_time_ms %= effect->step_time_ms;

    // ---- 根据效果类型执行更新 ----
    if (effect->type == MODULE_WS2812_EFFECT_BLINK)
    {
        // 闪烁：相位翻转（亮/灭交替）
        effect->phase ^= 1U;
        (void)module_ws2812_fill(me, (effect->phase != 0U) ? effect->color
                                                           : module_ws2812_make_color(0U, 0U, 0U));
    }
    else if (effect->type == MODULE_WS2812_EFFECT_COLOR_WIPE)
    {
        // 流水：逐个点亮
        (void)module_ws2812_set_pixel(me, effect->led_index, effect->color);
        ++effect->led_index;
        if (effect->led_index >= me->led_count)
        {
            effect->is_enabled = false; // 全部点亮后结束
        }
    }
    else if (effect->type == MODULE_WS2812_EFFECT_BREATH)
    {
        // 呼吸：亮度渐变（步进 5）
        int32_t new_brightness =
            (int32_t)effect->brightness + ((int32_t)effect->brightness_direction * 5);
        if (new_brightness >= 255)
        {
            new_brightness = 255;
            effect->brightness_direction = -1; // 变暗
        }
        else if (new_brightness <= 0)
        {
            new_brightness = 0;
            effect->brightness_direction = 1; // 变亮
        }
        effect->brightness = (uint8_t)new_brightness;
        // 应用亮度缩放并填充
        (void)module_ws2812_fill(me, module_ws2812_scale_color(effect->color, effect->brightness));
    }
    else if (effect->type == MODULE_WS2812_EFFECT_RAINBOW)
    {
        // 彩虹：色环偏移
        size_t led_index;
        for (led_index = 0U; led_index < me->led_count; ++led_index)
        {
            const uint8_t color_position =
                (uint8_t)(effect->color_offset + (uint16_t)((led_index * 256U) / me->led_count));
            me->pixels[led_index] = module_ws2812_color_wheel(color_position);
        }
        ++effect->color_offset;
    }
    else if (effect->type == MODULE_WS2812_EFFECT_THEATER_CHASE)
    {
        // 剧院追逐：每隔 2 颗点亮一颗，轮流
        size_t led_index;
        (void)module_ws2812_clear(me);
        for (led_index = effect->phase; led_index < me->led_count; led_index += 3U)
        {
            me->pixels[led_index] = effect->color;
        }
        effect->phase = (uint8_t)((effect->phase + 1U) % 3U);
    }

    // 更新完成后发送
    return module_ws2812_show(me);
}

/**
 * @brief 启动闪烁效果
 */
module_ws2812_status_t module_ws2812_start_blink(module_ws2812_t *me, module_ws2812_color_t color,
                                                 uint32_t half_period_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_BLINK, color, half_period_ms);
}

/**
 * @brief 启动流水效果
 */
module_ws2812_status_t module_ws2812_start_color_wipe(module_ws2812_t *me,
                                                      module_ws2812_color_t color,
                                                      uint32_t step_time_ms)
{
    const module_ws2812_status_t status =
        module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_COLOR_WIPE, color, step_time_ms);
    if (status == MODULE_WS2812_STATUS_OK)
    {
        (void)module_ws2812_clear(me); // 清空后开始逐个点亮
    }
    return status;
}

/**
 * @brief 启动呼吸效果
 */
module_ws2812_status_t module_ws2812_start_breath(module_ws2812_t *me, module_ws2812_color_t color,
                                                  uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_BREATH, color, step_time_ms);
}

/**
 * @brief 启动彩虹效果
 */
module_ws2812_status_t module_ws2812_start_rainbow(module_ws2812_t *me, uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_RAINBOW,
                                          module_ws2812_make_color(0U, 0U, 0U), step_time_ms);
}

/**
 * @brief 启动剧院追逐效果
 */
module_ws2812_status_t module_ws2812_start_theater_chase(module_ws2812_t *me,
                                                         module_ws2812_color_t color,
                                                         uint32_t step_time_ms)
{
    return module_ws2812_configure_effect(me, MODULE_WS2812_EFFECT_THEATER_CHASE, color,
                                          step_time_ms);
}

/**
 * @brief 停止当前效果
 */
module_ws2812_status_t module_ws2812_stop_effect(module_ws2812_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_WS2812_STATUS_NOT_INITIALIZED;
    }
    me->effect.is_enabled = false;
    me->effect.type = MODULE_WS2812_EFFECT_NONE;
    return MODULE_WS2812_STATUS_OK;
}

/**
 * @brief SPI 传输完成通知
 */
void module_ws2812_notify_transmit_complete(module_ws2812_t *me, bsp_status_t status)
{
    if ((me != NULL) && module_device_is_initialized(&me->super))
    {
        me->is_busy = false;
        if (status != BSP_STATUS_OK)
        {
            (void)bsp_spi_abort(me->spi);
        }
    }
}

/**
 * @brief 查询是否忙
 */
bool module_ws2812_is_busy(const module_ws2812_t *me)
{
    return (me != NULL) && module_device_is_initialized(&me->super) && me->is_busy;
}

/**
 * @brief 构造颜色
 */
module_ws2812_color_t module_ws2812_make_color(uint8_t red, uint8_t green, uint8_t blue)
{
    return (module_ws2812_color_t){.red = red, .green = green, .blue = blue};
}

/**
 * @brief 色环生成器
 */
module_ws2812_color_t module_ws2812_color_wheel(uint8_t position)
{
    // 色环分为三段：0~84，85~169，170~255
    if (position < 85U)
    {
        return module_ws2812_make_color((uint8_t)(position * 3U), (uint8_t)(255U - position * 3U),
                                        0U);
    }
    if (position < 170U)
    {
        position = (uint8_t)(position - 85U);
        return module_ws2812_make_color((uint8_t)(255U - position * 3U), 0U,
                                        (uint8_t)(position * 3U));
    }
    position = (uint8_t)(position - 170U);
    return module_ws2812_make_color(0U, (uint8_t)(position * 3U), (uint8_t)(255U - position * 3U));
}