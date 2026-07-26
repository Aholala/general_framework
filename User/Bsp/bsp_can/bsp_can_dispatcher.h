#ifndef BSP_CAN_DISPATCHER_H
#define BSP_CAN_DISPATCHER_H

#include "bsp_can.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_can_dispatcher bsp_can_dispatcher_t;

    typedef void (*bsp_can_frame_callback_t)(const bsp_can_frame_t *frame, void *user_context);

    typedef struct
    {
        uint32_t identifier;
        uint32_t mask;
        bsp_can_id_type_t id_type;
        bsp_can_frame_callback_t callback;
        void *user_context;
        bool is_enabled;
    } bsp_can_route_t;

    typedef struct
    {
        bsp_can_t *can;
        bsp_can_receive_fifo_t receive_fifo;
        bsp_can_route_t *route_storage;
        size_t route_capacity;
        size_t maximum_frames_per_process;
    } bsp_can_dispatcher_config_t;

    struct bsp_can_dispatcher
    {
        bsp_can_t *can;
        bsp_can_receive_fifo_t receive_fifo;
        bsp_can_route_t *route_storage;
        size_t route_capacity;
        size_t route_count;
        size_t maximum_frames_per_process;
        uint32_t received_frame_count;
        uint32_t unmatched_frame_count;
        uint32_t receive_error_count;
        volatile bool receive_pending;
        bool is_processing;
        bool is_initialized;
    };

    bsp_status_t bsp_can_dispatcher_init(bsp_can_dispatcher_t *const me,
                                         const bsp_can_dispatcher_config_t *const config);
    bsp_status_t bsp_can_dispatcher_add_route(bsp_can_dispatcher_t *const me, uint32_t identifier,
                                              uint32_t mask, bsp_can_id_type_t id_type,
                                              bsp_can_frame_callback_t callback, void *user_context,
                                              size_t *route_index);
    bsp_status_t bsp_can_dispatcher_remove_route(bsp_can_dispatcher_t *const me,
                                                 size_t route_index);
    bsp_status_t bsp_can_dispatcher_clear_routes(bsp_can_dispatcher_t *const me);
    bsp_status_t bsp_can_dispatcher_deinit(bsp_can_dispatcher_t *const me);
    bsp_status_t bsp_can_dispatcher_set_route_enabled(bsp_can_dispatcher_t *const me,
                                                      size_t route_index, bool is_enabled);
    bsp_status_t bsp_can_dispatcher_process(bsp_can_dispatcher_t *const me,
                                            size_t *processed_frame_count);
    bool bsp_can_dispatcher_has_pending_receive(const bsp_can_dispatcher_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_CAN_DISPATCHER_H */
