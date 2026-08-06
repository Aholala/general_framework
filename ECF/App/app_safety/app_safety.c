#include "app_safety.h"

#include "bsp_log.h"
#include "cmsis_os2.h"

typedef struct
{
    app_safety_monitor_t *monitors[APP_SAFETY_MAX_MONITOR_COUNT];
    size_t monitor_count;
    bsp_watchdog_t *watchdog;
    bool is_initialized;
} app_safety_manager_t;

static app_safety_manager_t app_safety_manager;

static uint32_t app_safety_ticks_to_ms(uint32_t tick_count)
{
    const uint32_t tick_frequency_hz = osKernelGetTickFreq();

    if (tick_frequency_hz == 0U)
    {
        return tick_count;
    }
    return (uint32_t)(((uint64_t)tick_count * 1000ULL) / tick_frequency_hz);
}

static void app_safety_set_offline(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_OFFLINE)
    {
        return;
    }

    monitor->state = APP_SAFETY_STATE_OFFLINE;
    BSP_LOG_WARNING("%s offline", monitor->config.name);
    if (monitor->config.offline_callback != NULL)
    {
        monitor->config.offline_callback(monitor->config.user_context);
    }
}

static void app_safety_set_online(app_safety_monitor_t *monitor)
{
    if (monitor->state == APP_SAFETY_STATE_ONLINE)
    {
        return;
    }

    monitor->state = APP_SAFETY_STATE_ONLINE;
    monitor->offline_time_ms = 0U;
    BSP_LOG_INFO("%s online", monitor->config.name);
    if (monitor->config.online_callback != NULL)
    {
        monitor->config.online_callback(monitor->config.user_context);
    }
}

bsp_status_t app_safety_init(bsp_watchdog_t *watchdog)
{
    size_t index;

    for (index = 0U; index < APP_SAFETY_MAX_MONITOR_COUNT; ++index)
    {
        app_safety_manager.monitors[index] = NULL;
    }
    app_safety_manager.monitor_count = 0U;
    app_safety_manager.watchdog = watchdog;
    app_safety_manager.is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_set_watchdog(bsp_watchdog_t *watchdog)
{
    if (!app_safety_manager.is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    app_safety_manager.watchdog = watchdog;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_monitor_init(app_safety_monitor_t *me,
                                   const app_safety_monitor_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->name == NULL) ||
        (config->timeout_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }

    me->config = *config;
    me->last_online_tick = osKernelGetTickCount();
    me->heartbeat_received = false;
    me->offline_time_ms = 0U;
    me->state = APP_SAFETY_STATE_STARTING;
    me->is_registered = false;
    return BSP_STATUS_OK;
}

bsp_status_t app_safety_register(app_safety_monitor_t *monitor)
{
    if (!app_safety_manager.is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    if ((monitor == NULL) || (monitor->config.name == NULL) ||
        (monitor->config.timeout_ms == 0U))
    {
        return BSP_STATUS_INVALID_ARGUMENT;
    }
    if (monitor->is_registered)
    {
        return BSP_STATUS_BUSY;
    }
    if (app_safety_manager.monitor_count >= APP_SAFETY_MAX_MONITOR_COUNT)
    {
        return BSP_STATUS_NO_RESOURCE;
    }

    monitor->last_online_tick = osKernelGetTickCount();
    app_safety_manager.monitors[app_safety_manager.monitor_count] = monitor;
    ++app_safety_manager.monitor_count;
    monitor->is_registered = true;
    return BSP_STATUS_OK;
}

void app_safety_notify_online(app_safety_monitor_t *monitor)
{
    if ((monitor == NULL) || !monitor->is_registered)
    {
        return;
    }

    monitor->last_online_tick = osKernelGetTickCount();
    monitor->heartbeat_received = true;
}

void app_safety_process(void)
{
    const uint32_t current_tick = osKernelGetTickCount();
    size_t index;

    if (!app_safety_manager.is_initialized)
    {
        return;
    }

    for (index = 0U; index < app_safety_manager.monitor_count; ++index)
    {
        app_safety_monitor_t *const monitor = app_safety_manager.monitors[index];
        const uint32_t elapsed_ticks = current_tick - monitor->last_online_tick;
        const uint32_t elapsed_time_ms = app_safety_ticks_to_ms(elapsed_ticks);

        monitor->offline_time_ms = elapsed_time_ms;
        if (monitor->heartbeat_received && (elapsed_time_ms <= monitor->config.timeout_ms))
        {
            app_safety_set_online(monitor);
        }
        else if (elapsed_time_ms > monitor->config.timeout_ms)
        {
            app_safety_set_offline(monitor);
        }
    }

    if (app_safety_manager.watchdog != NULL)
    {
        (void)bsp_watchdog_refresh(app_safety_manager.watchdog);
    }
}

app_safety_state_t app_safety_get_state(const app_safety_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->state : APP_SAFETY_STATE_OFFLINE;
}

uint32_t app_safety_get_offline_time_ms(const app_safety_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->offline_time_ms : UINT32_MAX;
}

bool app_safety_all_required_online(void)
{
    size_t index;

    if (!app_safety_manager.is_initialized)
    {
        return false;
    }
    for (index = 0U; index < app_safety_manager.monitor_count; ++index)
    {
        const app_safety_monitor_t *const monitor = app_safety_manager.monitors[index];
        if (monitor->config.required && (monitor->state != APP_SAFETY_STATE_ONLINE))
        {
            return false;
        }
    }
    return true;
}
