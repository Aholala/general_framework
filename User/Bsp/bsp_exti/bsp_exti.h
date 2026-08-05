#ifndef BSP_EXTI_H
#define BSP_EXTI_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct bsp_exti bsp_exti_t;
typedef void (*bsp_exti_callback_t)(bsp_exti_t *me, void *user_context);

typedef struct
{
    bsp_status_t (*init)(void *handle);
    bsp_status_t (*deinit)(void *handle);
    bsp_status_t (*enable)(void *handle);
    bsp_status_t (*disable)(void *handle);
} bsp_exti_driver_ops_t;

struct bsp_exti
{
    void *device_handle;
    bsp_exti_callback_t callback;
    void *user_context;
    bool is_initialized;
};

typedef bsp_exti_t bsp_exti_device_t;

typedef struct
{
    void *device_handle;
    const bsp_exti_driver_ops_t *driver_ops;
    bsp_exti_callback_t callback;
    void *user_context;
} bsp_exti_config_t;

bsp_status_t bsp_exti_bind_platform(const bsp_exti_driver_ops_t *driver_ops);
bsp_status_t bsp_exti_init(bsp_exti_t *me, const bsp_exti_config_t *config);
bsp_status_t bsp_exti_deinit(bsp_exti_t *me);
bool bsp_exti_is_initialized(const bsp_exti_t *me);
bsp_exti_t *bsp_exti_as_base(bsp_exti_device_t *me);
bsp_status_t bsp_exti_set_callback(bsp_exti_t *me, bsp_exti_callback_t callback,
                                   void *user_context);
bsp_status_t bsp_exti_enable(bsp_exti_t *me);
bsp_status_t bsp_exti_disable(bsp_exti_t *me);
void bsp_exti_notify(bsp_exti_t *me);

#ifdef __cplusplus
}
#endif
#endif
