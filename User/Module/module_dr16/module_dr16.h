#ifndef MODULE_DR16_H
#define MODULE_DR16_H

#include "bsp_usart.h"
#include "module_device.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define MODULE_DR16_FRAME_SIZE (18U)
#define MODULE_DR16_CHANNEL_COUNT (4U)

    typedef enum
    {
        MODULE_DR16_STATUS_OK = 0,
        MODULE_DR16_STATUS_INVALID_ARGUMENT,
        MODULE_DR16_STATUS_NOT_INITIALIZED,
        MODULE_DR16_STATUS_TRANSPORT_ERROR,
        MODULE_DR16_STATUS_INVALID_FRAME
    } module_dr16_status_t;

    typedef enum
    {
        MODULE_DR16_SWITCH_INVALID = 0,
        MODULE_DR16_SWITCH_UP = 1,
        MODULE_DR16_SWITCH_DOWN = 2,
        MODULE_DR16_SWITCH_MIDDLE = 3
    } module_dr16_switch_t;

    typedef enum
    {
        MODULE_DR16_KEY_W = (1U << 0),
        MODULE_DR16_KEY_S = (1U << 1),
        MODULE_DR16_KEY_A = (1U << 2),
        MODULE_DR16_KEY_D = (1U << 3),
        MODULE_DR16_KEY_SHIFT = (1U << 4),
        MODULE_DR16_KEY_CONTROL = (1U << 5),
        MODULE_DR16_KEY_Q = (1U << 6),
        MODULE_DR16_KEY_E = (1U << 7),
        MODULE_DR16_KEY_R = (1U << 8),
        MODULE_DR16_KEY_F = (1U << 9),
        MODULE_DR16_KEY_G = (1U << 10),
        MODULE_DR16_KEY_Z = (1U << 11),
        MODULE_DR16_KEY_X = (1U << 12),
        MODULE_DR16_KEY_C = (1U << 13),
        MODULE_DR16_KEY_V = (1U << 14),
        MODULE_DR16_KEY_B = (1U << 15)
    } module_dr16_key_t;

    typedef struct
    {
        int16_t channel[MODULE_DR16_CHANNEL_COUNT];
        float normalized_channel[MODULE_DR16_CHANNEL_COUNT];
        module_dr16_switch_t left_switch;
        module_dr16_switch_t right_switch;
        int16_t mouse_x;
        int16_t mouse_y;
        int16_t mouse_z;
        bool mouse_left_pressed;
        bool mouse_right_pressed;
        uint16_t keyboard;
        int16_t dial;
        float normalized_dial;
        uint32_t valid_frame_count;
        uint32_t invalid_frame_count;
        uint32_t receive_overrun_count;
        uint32_t transport_error_count;
        bool is_online;
    } module_dr16_data_t;

    typedef void (*module_dr16_frame_callback_t)(const module_dr16_data_t *data,
                                                 void *user_context);

    typedef struct
    {
        const char *logical_name;
        uint32_t registration_key;
        bsp_usart_t *usart;
        int16_t channel_deadband;
        uint32_t offline_timeout_ms;
        module_dr16_frame_callback_t frame_callback;
        void *user_context;
    } module_dr16_config_t;

    typedef struct
    {
        module_device_t super;
        bsp_usart_t *usart;
        module_dr16_data_t data;
        module_dr16_frame_callback_t frame_callback;
        void *user_context;
        uint8_t receive_buffer[MODULE_DR16_FRAME_SIZE * 2U];
        uint8_t pending_buffer[MODULE_DR16_FRAME_SIZE * 2U];
        uint8_t stream_window[MODULE_DR16_FRAME_SIZE];
        size_t stream_size;
        int16_t channel_deadband;
        uint32_t offline_timeout_ms;
        uint32_t time_since_frame_ms;
        volatile size_t pending_receive_size;
        volatile bool is_receive_pending;
        bool is_receiving;
    } module_dr16_t;

    module_dr16_status_t module_dr16_init(module_dr16_t *const me,
                                          const module_dr16_config_t *const config);
    module_dr16_status_t module_dr16_start(module_dr16_t *const me);
    module_dr16_status_t module_dr16_stop(module_dr16_t *const me);
    module_dr16_status_t module_dr16_process(module_dr16_t *const me);
    module_dr16_status_t module_dr16_feed_data(module_dr16_t *const me, const uint8_t *receive_data,
                                               size_t data_size);
    void module_dr16_update_time(module_dr16_t *const me, uint32_t elapsed_time_ms);
    const module_dr16_data_t *module_dr16_get_data(const module_dr16_t *const me);
    bool module_dr16_is_key_pressed(const module_dr16_t *const me, module_dr16_key_t key);
    float module_dr16_normalize_channel_value(int16_t channel_value);
    module_device_t *module_dr16_as_device(module_dr16_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* MODULE_DR16_H */
