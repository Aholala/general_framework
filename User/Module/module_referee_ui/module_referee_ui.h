#ifndef MODULE_REFEREE_UI_H
#define MODULE_REFEREE_UI_H

#include "module_referee.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_REFEREE_UI_COMMAND_INTERACTION (0x0301U)
#define MODULE_REFEREE_UI_NAME_SIZE (3U)

    typedef enum
    {
        MODULE_REFEREE_UI_OPERATION_NONE = 0,
        MODULE_REFEREE_UI_OPERATION_ADD = 1,
        MODULE_REFEREE_UI_OPERATION_CHANGE = 2,
        MODULE_REFEREE_UI_OPERATION_DELETE = 3
    } module_referee_ui_operation_t;

    typedef enum
    {
        MODULE_REFEREE_UI_GRAPHIC_LINE = 0,
        MODULE_REFEREE_UI_GRAPHIC_RECTANGLE,
        MODULE_REFEREE_UI_GRAPHIC_CIRCLE,
        MODULE_REFEREE_UI_GRAPHIC_ELLIPSE,
        MODULE_REFEREE_UI_GRAPHIC_ARC,
        MODULE_REFEREE_UI_GRAPHIC_FLOAT,
        MODULE_REFEREE_UI_GRAPHIC_INTEGER,
        MODULE_REFEREE_UI_GRAPHIC_STRING
    } module_referee_ui_graphic_type_t;

    typedef enum
    {
        MODULE_REFEREE_UI_COLOR_MAIN = 0,
        MODULE_REFEREE_UI_COLOR_YELLOW,
        MODULE_REFEREE_UI_COLOR_GREEN,
        MODULE_REFEREE_UI_COLOR_ORANGE,
        MODULE_REFEREE_UI_COLOR_PURPLE,
        MODULE_REFEREE_UI_COLOR_PINK,
        MODULE_REFEREE_UI_COLOR_CYAN,
        MODULE_REFEREE_UI_COLOR_BLACK,
        MODULE_REFEREE_UI_COLOR_WHITE
    } module_referee_ui_color_t;

    typedef struct
    {
        uint8_t name[MODULE_REFEREE_UI_NAME_SIZE];
        module_referee_ui_operation_t operation;
        module_referee_ui_graphic_type_t type;
        uint8_t layer;
        module_referee_ui_color_t color;
        uint16_t start_angle;
        uint16_t end_angle;
        uint16_t width;
        uint16_t start_x;
        uint16_t start_y;
        uint16_t radius;
        uint16_t end_x;
        uint16_t end_y;
    } module_referee_ui_graphic_t;

    typedef struct
    {
        module_referee_t *referee;
        module_referee_ui_graphic_t *queue_storage;
        size_t queue_capacity;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint32_t minimum_transmit_interval_ms;
        const char *logical_name;
        uint32_t registration_key;
    } module_referee_ui_config_t;

    typedef struct
    {
        module_device_t super;
        module_referee_t *referee;
        module_referee_ui_graphic_t *queue;
        size_t queue_capacity;
        size_t read_index;
        size_t write_index;
        size_t queue_count;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint32_t minimum_transmit_interval_ms;
        uint32_t transmit_elapsed_time_ms;
        uint32_t dropped_graphic_count;
        uint8_t payload_buffer[111];
        bool is_started;
    } module_referee_ui_t;

    module_device_status_t module_referee_ui_init(module_referee_ui_t *me,
                                                  const module_referee_ui_config_t *config);
    module_device_status_t module_referee_ui_enqueue(module_referee_ui_t *me,
                                                     const module_referee_ui_graphic_t *graphic);
    module_device_status_t module_referee_ui_delete_layer(module_referee_ui_t *me, uint8_t layer);
    module_device_status_t module_referee_ui_delete_all(module_referee_ui_t *me);
    module_device_status_t module_referee_ui_send_string(module_referee_ui_t *me,
                                                         const module_referee_ui_graphic_t *graphic,
                                                         const char *text);

#ifdef __cplusplus
}
#endif

#endif
