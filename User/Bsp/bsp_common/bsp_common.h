#ifndef BSP_COMMON_H
#define BSP_COMMON_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

    typedef enum
    {
        BSP_STATUS_OK = 0,
        BSP_STATUS_INVALID_ARGUMENT,
        BSP_STATUS_OUT_OF_RANGE,
        BSP_STATUS_NOT_INITIALIZED,
        BSP_STATUS_BUSY,
        BSP_STATUS_TIMEOUT,
        BSP_STATUS_IO_ERROR,
        BSP_STATUS_NO_RESOURCE,
        BSP_STATUS_UNSUPPORTED
    } bsp_status_t;

    typedef enum
    {
        BSP_TRANSFER_MODE_BLOCKING = 0,
        BSP_TRANSFER_MODE_INTERRUPT,
        BSP_TRANSFER_MODE_DMA
    } bsp_transfer_mode_t;

    typedef enum
    {
        BSP_EVENT_TRANSMIT_COMPLETE = 0,
        BSP_EVENT_RECEIVE_COMPLETE,
        BSP_EVENT_TRANSFER_COMPLETE,
        BSP_EVENT_RECEIVE_PENDING,
        BSP_EVENT_ABORT_COMPLETE,
        BSP_EVENT_ERROR
    } bsp_event_t;

    typedef void (*bsp_event_callback_t)(bsp_event_t event, bsp_status_t status,
                                         size_t transferred_size, void *user_context);

#define BSP_CONTAINER_OF(pointer, type, member)                                                    \
    ((type *)((uint8_t *)(pointer) - offsetof(type, member)))
#define BSP_CONTAINER_OF_CONST(pointer, type, member)                                              \
    ((const type *)((const uint8_t *)(pointer) - offsetof(type, member)))

    typedef struct bsp_device bsp_device_t;

    typedef struct
    {
        bsp_status_t (*deinit)(bsp_device_t *const me);
    } bsp_device_ops_t;

    struct bsp_device
    {
        const bsp_device_ops_t *vptr;
        void *device_handle;
        uint32_t object_magic;
        bool is_initialized;
    };

    bsp_status_t bsp_device_init(bsp_device_t *const me, const bsp_device_ops_t *const vptr,
                                 void *const device_handle);
    bsp_status_t bsp_device_deinit(bsp_device_t *const me);
    bool bsp_device_is_initialized(const bsp_device_t *const me);
    bool bsp_transfer_mode_is_valid(bsp_transfer_mode_t transfer_mode);
    void *bsp_device_get_handle(const bsp_device_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_COMMON_H */
