#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_exti bsp_exti_t;
    typedef struct bsp_exti_device bsp_exti_device_t;
    typedef void (*bsp_exti_callback_t)(bsp_exti_t *const me, void *user_context);

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*enable)(bsp_exti_t *const me);
        bsp_status_t (*disable)(bsp_exti_t *const me);
    } bsp_exti_ops_t;

    struct bsp_exti
    {
        bsp_device_t super;
        bsp_exti_callback_t callback;
        void *user_context;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *device_handle);
        bsp_status_t (*deinit)(void *device_handle);
        bsp_status_t (*enable)(void *device_handle);
        bsp_status_t (*disable)(void *device_handle);
    } bsp_exti_driver_ops_t;

    struct bsp_exti_device
    {
        bsp_exti_t super;
        const bsp_exti_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_exti_driver_ops_t *driver_ops;
        bsp_exti_callback_t callback;
        void *user_context;
    } bsp_exti_config_t;

    bsp_status_t bsp_exti_init(bsp_exti_device_t *const me, const bsp_exti_config_t *const config);
    bsp_exti_t *bsp_exti_as_base(bsp_exti_device_t *const me);
    bsp_status_t bsp_exti_set_callback(bsp_exti_t *const me, bsp_exti_callback_t callback,
                                       void *user_context);
    bsp_status_t bsp_exti_enable(bsp_exti_t *const me);
    bsp_status_t bsp_exti_disable(bsp_exti_t *const me);
    void bsp_exti_notify(bsp_exti_t *const me);

#ifdef __cplusplus
}
#endif

#endif /* BSP_EXTI_H */
