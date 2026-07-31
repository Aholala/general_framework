/**
 * @file module_referee_ui.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 裁判系统客户端图形 UI 模块头文件
 *        支持图形队列、批量编码和发送限频
 * @version 1.0
 * @date 2026-07-28
 * @copyright Copyright (c) 2026
 */

#ifndef MODULE_REFEREE_UI_H
#define MODULE_REFEREE_UI_H

#include "module_referee.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

/** @brief 交互数据命令 ID */
#define MODULE_REFEREE_UI_COMMAND_INTERACTION (0x0301U)
/** @brief 图形名称长度 */
#define MODULE_REFEREE_UI_NAME_SIZE (3U)

    /**
     * @brief 图形操作类型
     */
    typedef enum
    {
        MODULE_REFEREE_UI_OPERATION_NONE = 0,   // 无操作
        MODULE_REFEREE_UI_OPERATION_ADD = 1,     // 添加
        MODULE_REFEREE_UI_OPERATION_CHANGE = 2,  // 修改
        MODULE_REFEREE_UI_OPERATION_DELETE = 3   // 删除
    } module_referee_ui_operation_t;

    /**
     * @brief 图形类型
     */
    typedef enum
    {
        MODULE_REFEREE_UI_GRAPHIC_LINE = 0,      // 直线
        MODULE_REFEREE_UI_GRAPHIC_RECTANGLE,      // 矩形
        MODULE_REFEREE_UI_GRAPHIC_CIRCLE,         // 圆
        MODULE_REFEREE_UI_GRAPHIC_ELLIPSE,        // 椭圆
        MODULE_REFEREE_UI_GRAPHIC_ARC,            // 圆弧
        MODULE_REFEREE_UI_GRAPHIC_FLOAT,          // 浮点数
        MODULE_REFEREE_UI_GRAPHIC_INTEGER,        // 整数
        MODULE_REFEREE_UI_GRAPHIC_STRING          // 字符串
    } module_referee_ui_graphic_type_t;

    /**
     * @brief 图形颜色
     */
    typedef enum
    {
        MODULE_REFEREE_UI_COLOR_MAIN = 0,  // 主色（橙色）
        MODULE_REFEREE_UI_COLOR_YELLOW,    // 黄
        MODULE_REFEREE_UI_COLOR_GREEN,     // 绿
        MODULE_REFEREE_UI_COLOR_ORANGE,    // 橙
        MODULE_REFEREE_UI_COLOR_PURPLE,    // 紫
        MODULE_REFEREE_UI_COLOR_PINK,      // 粉
        MODULE_REFEREE_UI_COLOR_CYAN,      // 青
        MODULE_REFEREE_UI_COLOR_BLACK,     // 黑
        MODULE_REFEREE_UI_COLOR_WHITE      // 白
    } module_referee_ui_color_t;

    /**
     * @brief 图形结构体（对应裁判系统协议中的图形定义）
     */
    typedef struct
    {
        uint8_t name[MODULE_REFEREE_UI_NAME_SIZE];  // 图形名称（3 字节）
        module_referee_ui_operation_t operation;       // 操作类型
        module_referee_ui_graphic_type_t type;         // 图形类型
        uint8_t layer;                                 // 图层 (0~9)
        module_referee_ui_color_t color;               // 颜色
        uint16_t start_angle;                          // 起始角度 (0~511)
        uint16_t end_angle;                            // 终止角度 (0~511)
        uint16_t width;                                // 线宽 (0~1023)
        uint16_t start_x;                              // 起点 X (0~2047)
        uint16_t start_y;                              // 起点 Y (0~2047)
        uint16_t radius;                               // 半径 (0~1023)
        uint16_t end_x;                                // 终点 X (0~2047)
        uint16_t end_y;                                // 终点 Y (0~2047)
    } module_referee_ui_graphic_t;

    /**
     * @brief 裁判系统 UI 初始化配置
     */
    typedef struct
    {
        module_referee_t *referee;                      // 裁判系统对象指针
        module_referee_ui_graphic_t *queue_storage;      // 图形队列存储
        size_t queue_capacity;                           // 队列容量
        uint16_t sender_id;                              // 发送端 ID
        uint16_t receiver_id;                            // 接收端 ID
        uint32_t minimum_transmit_interval_ms;           // 最小发送间隔 (ms)
        const char *logical_name;                        // 设备逻辑名称
        uint32_t registration_key;                       // 注册键值
    } module_referee_ui_config_t;

    /**
     * @brief 裁判系统 UI 设备对象
     */
    typedef struct
    {
        module_device_t super;                    // 设备基类
        module_referee_t *referee;                 // 裁判系统对象
        module_referee_ui_graphic_t *queue;        // 图形环形队列
        size_t queue_capacity;                     // 队列容量
        size_t read_index;                         // 读索引（发送指针）
        size_t write_index;                        // 写索引（入队指针）
        size_t queue_count;                        // 队列中待发送数
        uint16_t sender_id;                        // 发送端 ID
        uint16_t receiver_id;                      // 接收端 ID
        uint32_t minimum_transmit_interval_ms;     // 最小发送间隔 (ms)
        uint32_t transmit_elapsed_time_ms;         // 距上次发送的时间 (ms)
        uint32_t dropped_graphic_count;            // 因队列满丢弃的图形数
        uint8_t payload_buffer[111];               // 交互数据负载缓冲区
        bool is_started;                            // 是否已启动
    } module_referee_ui_t;

        /**
     * @brief 初始化 UI 模块
     * @param me UI 对象
     * @param config 配置
     * @return 执行状态
     */
    module_device_status_t module_referee_ui_init(module_referee_ui_t *me,
                                                  const module_referee_ui_config_t *config);
        /**
     * @brief 入队一个图形（字符串类型需使用 send_string）
     * @param me UI 对象
     * @param graphic 图形结构体
     * @return 执行状态
     */
    module_device_status_t module_referee_ui_enqueue(module_referee_ui_t *me,
                                                     const module_referee_ui_graphic_t *graphic);
        /**
     * @brief 删除指定图层的所有图形
     * @param me UI 对象
     * @param layer 图层号 (0~9)
     * @return 执行状态
     */
    module_device_status_t module_referee_ui_delete_layer(module_referee_ui_t *me, uint8_t layer);
        /**
     * @brief 删除所有图形
     * @param me UI 对象
     * @return 执行状态
     */
    module_device_status_t module_referee_ui_delete_all(module_referee_ui_t *me);
        /**
     * @brief 发送字符串文本（立即发送，不入队）
     * @param me UI 对象
     * @param graphic 字符串图形定义
     * @param text 文本内容
     * @return 执行状态
     */
    module_device_status_t module_referee_ui_send_string(module_referee_ui_t *me,
                                                         const module_referee_ui_graphic_t *graphic,
                                                         const char *text);

#ifdef __cplusplus
}
#endif

#endif
