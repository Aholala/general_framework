/**
 * @file module_oled.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 基于 I2C 的单色页式 OLED 帧缓冲驱动头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 支持常见 SSD1306 类控制器，提供像素、直线、矩形、位图、清屏、对比度和整帧刷新。
 *       帧缓冲按页组织，由调用者静态分配。
 */

#ifndef MODULE_OLED_H
#define MODULE_OLED_H

#include "bsp_i2c.h"       // I2C BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief OLED 模块状态码
     */
    typedef enum
    {
        MODULE_OLED_STATUS_OK = 0,           // 操作成功
        MODULE_OLED_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_OLED_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_OLED_STATUS_NOT_STARTED,      // 未启动
        MODULE_OLED_STATUS_TRANSPORT_ERROR   // I2C 传输错误
    } module_oled_status_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief OLED 初始化配置
     */
    typedef struct
    {
        bsp_i2c_t *i2c;            // I2C BSP 基类（必须已初始化）
        uint16_t address_7bit;     // 7 位 I2C 地址（0~0x7F）
        uint16_t width_pixels;     // 屏幕宽度（像素），最大 128
        uint16_t height_pixels;    // 屏幕高度（像素），最大 64，且为 8 的倍数
        uint8_t *frame_buffer;     // 帧缓冲区（调用者分配）
        size_t frame_buffer_size;  // 缓冲区大小（必须 >= width * height / 8）
        uint32_t timeout_ms;       // I2C 超时（毫秒）
        const char *logical_name;  // 逻辑名称
        uint32_t registration_key; // 注册键值
    } module_oled_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief OLED 设备对象
     */
    typedef struct
    {
        module_device_t super;    // 设备基类
        bsp_i2c_t *i2c;           // I2C BSP 基类
        uint16_t address_7bit;    // I2C 地址
        uint16_t width_pixels;    // 宽度
        uint16_t height_pixels;   // 高度
        uint8_t *frame_buffer;    // 帧缓冲区指针
        size_t frame_buffer_size; // 缓冲区大小
        uint32_t timeout_ms;      // I2C 超时
        bool is_started;          // 是否已启动
    } module_oled_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 OLED 设备
     * @param me 设备对象
     * @param config 配置参数
     * @return 执行状态
     */
    module_oled_status_t module_oled_init(module_oled_t *me, const module_oled_config_t *config);

    /**
     * @brief 启动 OLED（发送初始化序列）
     * @param me 设备对象
     * @return 执行状态
     */
    module_oled_status_t module_oled_start(module_oled_t *me);

    /**
     * @brief 停止 OLED（关闭显示）
     * @param me 设备对象
     * @return 执行状态
     */
    module_oled_status_t module_oled_stop(module_oled_t *me);

    /**
     * @brief 清屏（全亮或全灭）
     * @param me 设备对象
     * @param is_on true=全亮，false=全灭
     * @return 执行状态
     */
    module_oled_status_t module_oled_clear(module_oled_t *me, bool is_on);

    /**
     * @brief 设置单个像素
     * @param me 设备对象
     * @param position_x X 坐标
     * @param position_y Y 坐标
     * @param is_on true=点亮，false=熄灭
     * @return 执行状态
     */
    module_oled_status_t module_oled_set_pixel(module_oled_t *me, int32_t position_x,
                                               int32_t position_y, bool is_on);

    /**
     * @brief 绘制直线（Bresenham 算法）
     * @param me 设备对象
     * @param start_x 起点 X
     * @param start_y 起点 Y
     * @param end_x 终点 X
     * @param end_y 终点 Y
     * @param is_on true=绘制，false=擦除
     * @return 执行状态
     */
    module_oled_status_t module_oled_draw_line(module_oled_t *me, int32_t start_x, int32_t start_y,
                                               int32_t end_x, int32_t end_y, bool is_on);

    /**
     * @brief 绘制矩形（空心或填充）
     * @param me 设备对象
     * @param position_x 左上角 X
     * @param position_y 左上角 Y
     * @param width_pixels 宽度
     * @param height_pixels 高度
     * @param is_filled true=填充，false=空心
     * @param is_on true=点亮，false=熄灭
     * @return 执行状态
     */
    module_oled_status_t module_oled_draw_rectangle(module_oled_t *me, int32_t position_x,
                                                    int32_t position_y, uint16_t width_pixels,
                                                    uint16_t height_pixels, bool is_filled,
                                                    bool is_on);

    /**
     * @brief 绘制位图（按页式布局）
     * @param me 设备对象
     * @param position_x 左上角 X
     * @param position_y 左上角 Y
     * @param width_pixels 位图宽度
     * @param height_pixels 位图高度
     * @param bitmap 位图数据（逐行，高位在前）
     * @param bitmap_size 数据大小（字节）
     * @param is_on true=点亮位图像素，false=熄灭位图像素
     * @return 执行状态
     * @note 位图大小需 >= (width * height + 7) / 8
     */
    module_oled_status_t module_oled_draw_bitmap(module_oled_t *me, int32_t position_x,
                                                 int32_t position_y, uint16_t width_pixels,
                                                 uint16_t height_pixels, const uint8_t *bitmap,
                                                 size_t bitmap_size, bool is_on);

    /**
     * @brief 将帧缓冲区刷新到屏幕
     * @param me 设备对象
     * @return 执行状态
     */
    module_oled_status_t module_oled_flush(module_oled_t *me);

    /**
     * @brief 设置对比度
     * @param me 设备对象
     * @param contrast 对比度值（0~255）
     * @return 执行状态
     */
    module_oled_status_t module_oled_set_contrast(module_oled_t *me, uint8_t contrast);

#ifdef __cplusplus
}
#endif

#endif