/**
 * @file module_oled.c
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief OLED 显示模块实现
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 I2C 的 SSD1306 类 OLED 驱动，提供页式帧缓冲和基本绘图功能。
 *       帧缓冲按页组织，调用 flush 将全部数据发送到屏幕。
 */

#include "module_oled.h"

#include <stddef.h> // NULL
#include <string.h> // memset, memcpy

/* ======================== 内部常量 ======================== */

/** @brief I2C 控制字节：命令 */
#define MODULE_OLED_CONTROL_COMMAND (0x00U)
/** @brief I2C 控制字节：数据 */
#define MODULE_OLED_CONTROL_DATA (0x40U)
/** @brief 最大宽度（像素） */
#define MODULE_OLED_MAXIMUM_WIDTH_PIXELS (128U)
/** @brief 最大页数（高度/8） */
#define MODULE_OLED_MAXIMUM_PAGE_COUNT (8U)

/* ======================== 内部函数 ======================== */

/**
 * @brief 向 OLED 写入数据（命令或数据）
 * @param me OLED 对象
 * @param control_byte 控制字节（0x00=命令，0x40=数据）
 * @param payload 数据负载
 * @param payload_size 负载大小（字节）
 * @return 执行状态
 */
static module_oled_status_t module_oled_write(module_oled_t *me, uint8_t control_byte,
                                              const uint8_t *payload, size_t payload_size)
{
    uint8_t transfer_buffer[MODULE_OLED_MAXIMUM_WIDTH_PIXELS + 1U];

    // 参数校验：负载非空，大小不超过最大宽度（一页数据）
    if ((payload == NULL) || (payload_size == 0U) ||
        (payload_size > MODULE_OLED_MAXIMUM_WIDTH_PIXELS))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    // 第一个字节是控制字节，之后是数据
    transfer_buffer[0] = control_byte;
    memcpy(&transfer_buffer[1], payload, payload_size);
    // 通过 I2C 发送（阻塞模式）
    return (bsp_i2c_transmit(me->i2c, me->address_7bit, transfer_buffer, payload_size + 1U,
                             BSP_TRANSFER_MODE_BLOCKING, me->timeout_ms) == BSP_STATUS_OK)
               ? MODULE_OLED_STATUS_OK
               : MODULE_OLED_STATUS_TRANSPORT_ERROR;
}

/**
 * @brief 写入命令序列
 * @param me OLED 对象
 * @param commands 命令数组
 * @param command_count 命令数量
 * @return 执行状态
 */
static module_oled_status_t module_oled_write_commands(module_oled_t *me, const uint8_t *commands,
                                                       size_t command_count)
{
    return module_oled_write(me, MODULE_OLED_CONTROL_COMMAND, commands, command_count);
}

/* ======================== module_device 虚函数实现 ======================== */

/**
 * @brief 设备启动回调（转发至 module_oled_start）
 */
static module_device_status_t module_oled_device_start(module_device_t *const device_base)
{
    module_oled_t *const me = MODULE_CONTAINER_OF(device_base, module_oled_t, super);
    return (module_oled_start(me) == MODULE_OLED_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                            : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备停止回调（转发至 module_oled_stop）
 */
static module_device_status_t module_oled_device_stop(module_device_t *const device_base)
{
    module_oled_t *const me = MODULE_CONTAINER_OF(device_base, module_oled_t, super);
    return (module_oled_stop(me) == MODULE_OLED_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                           : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

/**
 * @brief 设备更新回调（OLED 不需要周期更新，空操作）
 */
static module_device_status_t module_oled_device_update(module_device_t *const device_base,
                                                        uint32_t elapsed_time_ms)
{
    (void)device_base;
    (void)elapsed_time_ms;
    return MODULE_DEVICE_STATUS_OK;
}

/** OLED 的设备操作表 */
static const module_device_ops_t s_module_oled_ops = {
    .start = module_oled_device_start,
    .stop = module_oled_device_stop,
    .update = module_oled_device_update,
};

/* ======================== 公共 API ======================== */

/**
 * @brief 初始化 OLED 设备
 */
module_oled_status_t module_oled_init(module_oled_t *me, const module_oled_config_t *config)
{
    size_t required_buffer_size;

    // ---- 参数校验 ----
    if ((me == NULL) || (config == NULL) || (config->i2c == NULL) ||
        (config->address_7bit > 0x7FU) || (config->width_pixels == 0U) ||
        (config->width_pixels > MODULE_OLED_MAXIMUM_WIDTH_PIXELS) ||
        (config->height_pixels == 0U) || (config->height_pixels > 64U) ||
        ((config->height_pixels % 8U) != 0U) || (config->frame_buffer == NULL))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    // 计算所需缓冲区大小：宽度 * (高度/8)
    required_buffer_size = (size_t)config->width_pixels * ((size_t)config->height_pixels / 8U);
    if (config->frame_buffer_size < required_buffer_size)
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }

    // ---- 初始化对象 ----
    *me = (module_oled_t){0};
    me->i2c = config->i2c;
    me->address_7bit = config->address_7bit;
    me->width_pixels = config->width_pixels;
    me->height_pixels = config->height_pixels;
    me->frame_buffer = config->frame_buffer;
    me->frame_buffer_size = required_buffer_size;
    me->timeout_ms = config->timeout_ms;
    memset(me->frame_buffer, 0, required_buffer_size); // 清空帧缓冲

    // ---- 初始化基类 ----
    if (module_device_init_base(&me->super, &s_module_oled_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_complete_init(&me->super) != MODULE_DEVICE_STATUS_OK)
    {
        module_device_abort_init(&me->super);
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 启动 OLED（发送初始化序列）
 * @param me OLED 对象
 * @return 执行状态
 * @note 初始化序列包含显示关闭、时钟分频、复用率、偏移、显示开始线、电荷泵、寻址模式、
 *       段重映射、COM 扫描方向、COM 引脚配置、对比度、预充电周期、VCOM 检测、全局显示
 *       开启、正常显示等。
 */
module_oled_status_t module_oled_start(module_oled_t *me)
{
    // 标准的 SSD1306 初始化命令序列（适用于 128x64/128x32 等）
    uint8_t commands[] = {
        0xAEU,                                                      // 显示关闭
        0xD5U, 0x80U,                                               // 时钟分频（0x80）
        0xA8U, (uint8_t)(me != NULL ? me->height_pixels - 1U : 0U), // 复用率
        0xD3U, 0x00U,                                               // 显示偏移
        0x40U,                                                      // 显示开始线（行 0）
        0x8DU, 0x14U,                                               // 电荷泵使能
        0x20U, 0x02U,                                               // 寻址模式（页寻址）
        0xA1U,                                                      // 段重映射（列 127→0）
        0xC8U,                                                      // COM 输出扫描方向（反向）
        0xDAU, (uint8_t)((me != NULL && me->height_pixels == 64U) ? 0x12U : 0x02U), // COM 引脚配置
        0x81U, 0x7FU, // 对比度（0x7F）
        0xD9U, 0xF1U, // 预充电周期
        0xDBU, 0x40U, // VCOM 检测
        0xA4U,        // 显示全部开启（跟随 RAM）
        0xA6U,        // 正常显示（非反色）
        0xAFU,        // 显示开启
    };

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    // 发送初始化命令序列
    if (module_oled_write_commands(me, commands, sizeof(commands)) != MODULE_OLED_STATUS_OK)
    {
        return MODULE_OLED_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = true;
    // 立即刷新一次，显示帧缓冲内容
    return module_oled_flush(me);
}

/**
 * @brief 停止 OLED（关闭显示）
 */
module_oled_status_t module_oled_stop(module_oled_t *me)
{
    const uint8_t command = 0xAEU; // 显示关闭命令

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_OK;
    }
    if (module_oled_write_commands(me, &command, 1U) != MODULE_OLED_STATUS_OK)
    {
        return MODULE_OLED_STATUS_TRANSPORT_ERROR;
    }
    me->is_started = false;
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 清屏
 */
module_oled_status_t module_oled_clear(module_oled_t *me, bool is_on)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    // 全亮（0xFF）或全灭（0x00）
    memset(me->frame_buffer, is_on ? 0xFF : 0x00, me->frame_buffer_size);
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 设置像素
 */
module_oled_status_t module_oled_set_pixel(module_oled_t *me, int32_t position_x,
                                           int32_t position_y, bool is_on)
{
    size_t buffer_index;
    uint8_t pixel_mask;

    // 状态检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    // 边界检查
    if ((position_x < 0) || (position_y < 0) || (position_x >= (int32_t)me->width_pixels) ||
        (position_y >= (int32_t)me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    // 计算帧缓冲索引：页（行/8）* 宽度 + 列
    buffer_index = (size_t)position_x + ((size_t)position_y / 8U) * me->width_pixels;
    // 计算该像素在字节中的掩码（位序：高位对应低行？SSD1306
    // 页式寻址中，位0对应最低行，位7对应最高行）
    pixel_mask = (uint8_t)(1U << ((uint32_t)position_y & 7U));
    if (is_on)
    {
        me->frame_buffer[buffer_index] |= pixel_mask;
    }
    else
    {
        me->frame_buffer[buffer_index] &= (uint8_t)(~pixel_mask);
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 绘制直线（Bresenham 算法）
 */
module_oled_status_t module_oled_draw_line(module_oled_t *me, int32_t start_x, int32_t start_y,
                                           int32_t end_x, int32_t end_y, bool is_on)
{
    // 计算 delta（绝对值）
    int32_t delta_x = (end_x > start_x) ? end_x - start_x : start_x - end_x;
    int32_t delta_y = (end_y > start_y) ? start_y - end_y : end_y - start_y;
    const int32_t step_x = (start_x < end_x) ? 1 : -1;
    const int32_t step_y = (start_y < end_y) ? 1 : -1;
    int32_t error = delta_x + delta_y;

    // 状态和边界检查
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if ((start_x < 0) || (start_y < 0) || (end_x < 0) || (end_y < 0) ||
        (start_x >= (int32_t)me->width_pixels) || (end_x >= (int32_t)me->width_pixels) ||
        (start_y >= (int32_t)me->height_pixels) || (end_y >= (int32_t)me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    // Bresenham 算法
    for (;;)
    {
        (void)module_oled_set_pixel(me, start_x, start_y, is_on);
        if ((start_x == end_x) && (start_y == end_y))
            break;
        if ((2 * error) >= delta_y)
        {
            error += delta_y;
            start_x += step_x;
        }
        if ((2 * error) <= delta_x)
        {
            error += delta_x;
            start_y += step_y;
        }
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 绘制矩形
 */
module_oled_status_t module_oled_draw_rectangle(module_oled_t *me, int32_t position_x,
                                                int32_t position_y, uint16_t width_pixels,
                                                uint16_t height_pixels, bool is_filled, bool is_on)
{
    uint16_t row_index;

    // 参数校验
    if ((me == NULL) || (width_pixels == 0U) || (height_pixels == 0U))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    // 边界检查
    if ((position_x < 0) || (position_y < 0) ||
        ((uint32_t)position_x + width_pixels > me->width_pixels) ||
        ((uint32_t)position_y + height_pixels > me->height_pixels))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (is_filled)
    {
        // 填充矩形：逐行绘制水平线
        for (row_index = 0U; row_index < height_pixels; ++row_index)
        {
            (void)module_oled_draw_line(me, position_x, position_y + row_index,
                                        position_x + width_pixels - 1, position_y + row_index,
                                        is_on);
        }
    }
    else
    {
        // 空心矩形：四条边
        (void)module_oled_draw_line(me, position_x, position_y, position_x + width_pixels - 1,
                                    position_y, is_on);
        (void)module_oled_draw_line(me, position_x, position_y + height_pixels - 1,
                                    position_x + width_pixels - 1, position_y + height_pixels - 1,
                                    is_on);
        (void)module_oled_draw_line(me, position_x, position_y, position_x,
                                    position_y + height_pixels - 1, is_on);
        (void)module_oled_draw_line(me, position_x + width_pixels - 1, position_y,
                                    position_x + width_pixels - 1, position_y + height_pixels - 1,
                                    is_on);
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 绘制位图
 */
module_oled_status_t module_oled_draw_bitmap(module_oled_t *me, int32_t position_x,
                                             int32_t position_y, uint16_t width_pixels,
                                             uint16_t height_pixels, const uint8_t *bitmap,
                                             size_t bitmap_size, bool is_on)
{
    // 计算所需位图大小（按行存储，高位在前）
    size_t required_size = ((size_t)width_pixels * height_pixels + 7U) / 8U;
    uint16_t pixel_y;
    uint16_t pixel_x;

    // 参数校验
    if ((me == NULL) || (bitmap == NULL) || (width_pixels == 0U) || (height_pixels == 0U) ||
        (bitmap_size < required_size))
    {
        return MODULE_OLED_STATUS_INVALID_ARGUMENT;
    }
    if (!module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }

    // 遍历像素，仅绘制位图中为 1 的像素（或 is_on 控制是否绘制）
    for (pixel_y = 0U; pixel_y < height_pixels; ++pixel_y)
    {
        for (pixel_x = 0U; pixel_x < width_pixels; ++pixel_x)
        {
            const size_t bit_index = (size_t)pixel_y * width_pixels + pixel_x;
            if ((bitmap[bit_index / 8U] & (uint8_t)(0x80U >> (bit_index & 7U))) != 0U)
            {
                const int32_t target_x = position_x + (int32_t)pixel_x;
                const int32_t target_y = position_y + (int32_t)pixel_y;
                // 裁剪到屏幕范围内
                if ((target_x >= 0) && (target_y >= 0) && (target_x < (int32_t)me->width_pixels) &&
                    (target_y < (int32_t)me->height_pixels))
                {
                    (void)module_oled_set_pixel(me, target_x, target_y, is_on);
                }
            }
        }
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 刷新整屏
 */
module_oled_status_t module_oled_flush(module_oled_t *me)
{
    uint16_t page_index;
    const uint16_t page_count = (me != NULL) ? me->height_pixels / 8U : 0U;

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_NOT_STARTED;
    }
    // 逐页发送：设置页地址 → 列地址 → 写入该页所有列的数据
    for (page_index = 0U; page_index < page_count; ++page_index)
    {
        // 设置页地址（0xB0 + page_index），列地址低字节（0x00），列地址高字节（0x10）
        const uint8_t commands[3] = {(uint8_t)(0xB0U + page_index), 0x00U, 0x10U};
        if ((module_oled_write_commands(me, commands, 3U) != MODULE_OLED_STATUS_OK) ||
            (module_oled_write(me, MODULE_OLED_CONTROL_DATA,
                               &me->frame_buffer[(size_t)page_index * me->width_pixels],
                               me->width_pixels) != MODULE_OLED_STATUS_OK))
        {
            return MODULE_OLED_STATUS_TRANSPORT_ERROR;
        }
    }
    return MODULE_OLED_STATUS_OK;
}

/**
 * @brief 设置对比度
 */
module_oled_status_t module_oled_set_contrast(module_oled_t *me, uint8_t contrast)
{
    const uint8_t commands[2] = {0x81U, contrast};

    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_OLED_STATUS_NOT_INITIALIZED;
    }
    if (!me->is_started)
    {
        return MODULE_OLED_STATUS_NOT_STARTED;
    }
    return module_oled_write_commands(me, commands, 2U);
}