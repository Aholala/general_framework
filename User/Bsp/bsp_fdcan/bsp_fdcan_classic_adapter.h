#ifndef BSP_FDCAN_CLASSIC_ADAPTER_H
#define BSP_FDCAN_CLASSIC_ADAPTER_H

#include "bsp_can.h"
#include "bsp_fdcan.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        bsp_can_device_t super;
        bsp_fdcan_t *fdcan;
    } bsp_fdcan_classic_adapter_t;

    typedef struct
    {
        bsp_fdcan_t *fdcan;
        bsp_event_callback_t callback;
        void *user_context;
    } bsp_fdcan_classic_adapter_config_t;

    bsp_status_t
    bsp_fdcan_classic_adapter_init(bsp_fdcan_classic_adapter_t *const me,
                                   const bsp_fdcan_classic_adapter_config_t *const config);
    bsp_can_t *bsp_fdcan_classic_adapter_as_can(bsp_fdcan_classic_adapter_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_FDCAN_CLASSIC_ADAPTER_H */
