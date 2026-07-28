/**
 * @file module_ws2812.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief WS2812 灯带驱动模块头文件
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 *
 * @note 基于 SPI 编码波形的 WS2812 灯带驱动，支持像素缓冲、全局亮度、
 *       异步刷新以及多种灯光效果（闪烁、流水、呼吸、彩虹、剧院追逐）。
 */

#ifndef MODULE_WS2812_H
#define MODULE_WS2812_H

#include "bsp_spi.h"       // SPI BSP 抽象层
#include "module_device.h" // 模块设备基类

#ifdef __cplusplus
extern "C"
{
#endif

/* ======================== 编码常量 ======================== */

/**
 * @brief 每颗 LED 编码后的字节数
 * @note 每个原始字节（8 位）编码为 3 个 SPI 字节（每 bit 用 3 个 SPI bit 表示）
 *       每颗 LED 需要 3 个颜色分量（G、R、B），共 3×8=24 位，编码后为 9 字节
 */
#define MODULE_WS2812_ENCODED_BYTES_PER_LED (9U)

/**
 * @brief 计算发送缓冲区所需最小字节数
 * @param led_count LED 数量
 * @param reset_byte_count 复位低电平字节数（通常 1~2 字节）
 * @return 所需缓冲区大小（字节）
 * @note 公式：led_count × 9 + reset_byte_count
 */
#define MODULE_WS2812_REQUIRED_BUFFER_SIZE(led_count, reset_byte_count)                            \
    ((size_t)(led_count) * MODULE_WS2812_ENCODED_BYTES_PER_LED + (size_t)(reset_byte_count))

    /* ======================== 状态码枚举 ======================== */

    /**
     * @brief WS2812 模块状态码
     */
    typedef enum
    {
        MODULE_WS2812_STATUS_OK = 0,           // 操作成功
        MODULE_WS2812_STATUS_INVALID_ARGUMENT, // 参数非法
        MODULE_WS2812_STATUS_NOT_INITIALIZED,  // 对象未初始化
        MODULE_WS2812_STATUS_NOT_STARTED,      // 未启动（未调用 start）
        MODULE_WS2812_STATUS_BUSY,             // SPI 传输忙
        MODULE_WS2812_STATUS_TRANSPORT_ERROR   // SPI 传输错误
    } module_ws2812_status_t;

    /* ======================== 颜色结构体 ======================== */

    /**
     * @brief WS2812 颜色结构体（RGB 顺序）
     * @note 注意 WS2812 的数据顺序为 GRB（绿色、红色、蓝色）
     *       但本结构体按红绿蓝顺序存储，编码时自动调整
     */
    typedef struct
    {
        uint8_t red;   // 红色分量 0~255
        uint8_t green; // 绿色分量 0~255
        uint8_t blue;  // 蓝色分量 0~255
    } module_ws2812_color_t;

    /* ======================== 效果类型枚举 ======================== */

    /**
     * @brief 支持的内置灯光效果类型
     */
    typedef enum
    {
        MODULE_WS2812_EFFECT_NONE = 0,     // 无效果（手动控制）
        MODULE_WS2812_EFFECT_BLINK,        // 闪烁（亮灭交替）
        MODULE_WS2812_EFFECT_COLOR_WIPE,   // 流水（逐个点亮）
        MODULE_WS2812_EFFECT_BREATH,       // 呼吸（亮度渐变）
        MODULE_WS2812_EFFECT_RAINBOW,      // 彩虹（颜色循环）
        MODULE_WS2812_EFFECT_THEATER_CHASE // 剧院追逐（间隔亮灯）
    } module_ws2812_effect_t;

    /* ======================== 效果状态结构体 ======================== */

    /**
     * @brief 灯光效果运行状态
     * @note 每个效果类型使用不同的字段组合，但统一存储在此结构中
     */
    typedef struct
    {
        module_ws2812_effect_t type; // 当前效果类型
        module_ws2812_color_t color; // 效果使用的颜色（部分效果）
        uint32_t step_time_ms;       // 每步时间（毫秒）
        uint32_t elapsed_time_ms;    // 已累积时间（毫秒）
        size_t led_index;            // 流水效果当前索引
        uint16_t color_offset;       // 彩虹效果颜色偏移
        uint8_t phase;               // 剧院追逐/闪烁相位
        uint8_t brightness;          // 呼吸效果当前亮度
        int8_t brightness_direction; // 呼吸效果方向（+1 变亮，-1 变暗）
        bool is_enabled;             // 效果是否启用
    } module_ws2812_effect_state_t;

    /* ======================== 配置结构体 ======================== */

    /**
     * @brief WS2812 初始化配置
     */
    typedef struct
    {
        bsp_spi_t *spi;                    // SPI BSP 基类（必须已初始化）
        module_ws2812_color_t *pixels;     // 像素颜色数组（调用者分配）
        size_t led_count;                  // LED 数量
        uint8_t *transmit_buffer;          // 编码发送缓冲区（调用者分配）
        size_t transmit_buffer_size;       // 发送缓冲区大小（必须 >= REQUIRED_BUFFER_SIZE）
        size_t reset_byte_count;           // 复位低电平字节数（至少 1）
        uint32_t transmit_timeout_ms;      // SPI 传输超时（毫秒）
        bsp_transfer_mode_t transfer_mode; // 传输模式（BLOCKING/INTERRUPT/DMA）
        const char *logical_name;          // 设备逻辑名称
        uint32_t registration_key;         // 注册键值
    } module_ws2812_config_t;

    /* ======================== 对象结构体 ======================== */

    /**
     * @brief WS2812 设备对象
     */
    typedef struct
    {
        module_device_t super;               // 设备基类
        bsp_spi_t *spi;                      // SPI BSP 基类
        module_ws2812_color_t *pixels;       // 像素颜色数组（引用外部）
        size_t led_count;                    // LED 数量
        uint8_t *transmit_buffer;            // 编码发送缓冲区（引用外部）
        size_t transmit_buffer_size;         // 实际发送缓冲区大小
        size_t reset_byte_count;             // 复位字节数
        uint32_t transmit_timeout_ms;        // SPI 超时
        bsp_transfer_mode_t transfer_mode;   // 传输模式
        uint8_t brightness;                  // 全局亮度（0~255）
        module_ws2812_effect_state_t effect; // 效果状态
        volatile bool is_busy;               // SPI 发送是否忙（ISR/任务共享）
        bool is_started;                     // 是否已启动
    } module_ws2812_t;

    /* ======================== 公共 API ======================== */

    /**
     * @brief 初始化 WS2812 设备
     * @param me 设备对象指针
     * @param config 初始化配置
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_init(module_ws2812_t *me,
                                              const module_ws2812_config_t *config);

    /**
     * @brief 启动灯带（允许 show 和效果运行）
     * @param me 设备对象指针
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start(module_ws2812_t *me);

    /**
     * @brief 停止灯带（停止发送并禁用效果）
     * @param me 设备对象指针
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_stop(module_ws2812_t *me);

    /**
     * @brief 设置单个 LED 的颜色
     * @param me 设备对象指针
     * @param led_index LED 索引（从 0 开始）
     * @param color 颜色值
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_set_pixel(module_ws2812_t *me, size_t led_index,
                                                   module_ws2812_color_t color);

    /**
     * @brief 将所有 LED 填充为相同颜色
     * @param me 设备对象指针
     * @param color 颜色值
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_fill(module_ws2812_t *me, module_ws2812_color_t color);

    /**
     * @brief 清除所有 LED（全部熄灭）
     * @param me 设备对象指针
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_clear(module_ws2812_t *me);

    /**
     * @brief 设置全局亮度（软件缩放）
     * @param me 设备对象指针
     * @param brightness 亮度值（0~255）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_set_brightness(module_ws2812_t *me, uint8_t brightness);

    /**
     * @brief 将像素缓冲区编码并通过 SPI 发送
     * @param me 设备对象指针
     * @return 执行状态
     * @note 如果传输模式为 INTERRUPT/DMA，则异步发送，通过 notify 回调清除 busy
     */
    module_ws2812_status_t module_ws2812_show(module_ws2812_t *me);

    /**
     * @brief 周期更新效果（需在任务中周期性调用）
     * @param me 设备对象指针
     * @param elapsed_time_ms 距上次调用的时间（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_update(module_ws2812_t *me, uint32_t elapsed_time_ms);

    /**
     * @brief 启动闪烁效果
     * @param me 设备对象指针
     * @param color 闪烁颜色
     * @param half_period_ms 亮/灭半周期（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start_blink(module_ws2812_t *me,
                                                     module_ws2812_color_t color,
                                                     uint32_t half_period_ms);

    /**
     * @brief 启动流水效果（逐个点亮）
     * @param me 设备对象指针
     * @param color 点亮颜色
     * @param step_time_ms 每步时间（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start_color_wipe(module_ws2812_t *me,
                                                          module_ws2812_color_t color,
                                                          uint32_t step_time_ms);

    /**
     * @brief 启动呼吸效果（亮度渐变）
     * @param me 设备对象指针
     * @param color 呼吸颜色
     * @param step_time_ms 每步时间（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start_breath(module_ws2812_t *me,
                                                      module_ws2812_color_t color,
                                                      uint32_t step_time_ms);

    /**
     * @brief 启动彩虹效果（颜色循环）
     * @param me 设备对象指针
     * @param step_time_ms 每步时间（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start_rainbow(module_ws2812_t *me, uint32_t step_time_ms);

    /**
     * @brief 启动剧院追逐效果
     * @param me 设备对象指针
     * @param color 追逐颜色
     * @param step_time_ms 每步时间（毫秒）
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_start_theater_chase(module_ws2812_t *me,
                                                             module_ws2812_color_t color,
                                                             uint32_t step_time_ms);

    /**
     * @brief 停止当前效果（恢复手动控制）
     * @param me 设备对象指针
     * @return 执行状态
     */
    module_ws2812_status_t module_ws2812_stop_effect(module_ws2812_t *me);

    /**
     * @brief SPI 传输完成通知（由平台 SPI 回调调用）
     * @param me 设备对象指针
     * @param status SPI 传输状态
     * @note 用于清除 busy 标志，通常在 ISR 中调用
     */
    void module_ws2812_notify_transmit_complete(module_ws2812_t *me, bsp_status_t status);

    /**
     * @brief 查询是否正在发送
     * @param me 设备对象指针
     * @return true=忙，false=空闲
     */
    bool module_ws2812_is_busy(const module_ws2812_t *me);

    /**
     * @brief 构造 RGB 颜色
     * @param red 红色分量
     * @param green 绿色分量
     * @param blue 蓝色分量
     * @return 颜色结构体
     */
    module_ws2812_color_t module_ws2812_make_color(uint8_t red, uint8_t green, uint8_t blue);

    /**
     * @brief 色环生成（用于彩虹效果）
     * @param position 色环位置（0~255）
     * @return 对应的 RGB 颜色
     */
    module_ws2812_color_t module_ws2812_color_wheel(uint8_t position);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_WS2812_H */