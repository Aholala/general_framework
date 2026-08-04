#ifndef APP_SAFE_H
#define APP_SAFE_H

#include "bsp_common.h"
#include "bsp_watchdog.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define APP_SAFE_MAX_MONITOR_COUNT (16U)
#define APP_SAFE_DEFAULT_TASK_PERIOD_MS (5U)

    typedef enum
    {
        APP_SAFE_STATE_STARTING = 0,
        APP_SAFE_STATE_ONLINE,
        APP_SAFE_STATE_OFFLINE
    } app_safe_state_t;

    typedef void (*app_safe_callback_t)(void *user_context);

    typedef struct
    {
        const char *name;
        uint32_t timeout_ms;
        bool required;
        app_safe_callback_t offline_callback;
        app_safe_callback_t online_callback;
        void *user_context;
    } app_safe_monitor_config_t;

    typedef struct
    {
        app_safe_monitor_config_t config;
        volatile uint32_t last_online_tick;
        volatile bool heartbeat_received;
        uint32_t offline_time_ms;
        app_safe_state_t state;
        bool is_registered;
    } app_safe_monitor_t;

    bsp_status_t app_safe_init(bsp_watchdog_t *watchdog);
    bsp_status_t app_safe_set_watchdog(bsp_watchdog_t *watchdog);
    bsp_status_t app_safe_monitor_init(app_safe_monitor_t *me,
                                       const app_safe_monitor_config_t *config);
    bsp_status_t app_safe_register(app_safe_monitor_t *monitor);
    void app_safe_notify_online(app_safe_monitor_t *monitor);
    void app_safe_process(void);
    app_safe_state_t app_safe_get_state(const app_safe_monitor_t *monitor);
    uint32_t app_safe_get_offline_time_ms(const app_safe_monitor_t *monitor);
    bool app_safe_all_required_online(void);
    void app_safe_task(void *argument);

#ifdef __cplusplus
}
#endif

#endif
