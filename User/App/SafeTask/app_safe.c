#include "app_safe.h"

#include "bsp_log.h"
#include "cmsis_os2.h"

typedef struct
{
    app_safe_monitor_t *monitors[APP_SAFE_MAX_MONITOR_COUNT];
    size_t monitor_count;
    bsp_watchdog_t *watchdog;
    bool is_initialized;
} app_safe_manager_t;

static app_safe_manager_t app_safe_manager;

static uint32_t app_safe_ticks_to_ms(uint32_t tick_count)
{
    const uint32_t tick_frequency_hz = osKernelGetTickFreq();

    if (tick_frequency_hz == 0U)
    {
        return tick_count;
    }
    return (uint32_t)(((uint64_t)tick_count * 1000ULL) / tick_frequency_hz);
}

static uint32_t app_safe_ms_to_ticks(uint32_t time_ms)
{
    const uint32_t tick_frequency_hz = osKernelGetTickFreq();
    uint32_t tick_count;

    if (tick_frequency_hz == 0U)
    {
        return time_ms;
    }
    tick_count = (uint32_t)((((uint64_t)time_ms * tick_frequency_hz) + 999ULL) / 1000ULL);
    return (tick_count > 0U) ? tick_count : 1U;
}

static void app_safe_set_offline(app_safe_monitor_t *monitor)
{
    if (monitor->state == APP_SAFE_STATE_OFFLINE)
    {
        return;
    }

    monitor->state = APP_SAFE_STATE_OFFLINE;
    BSP_LOG_WARNING("%s offline", monitor->config.name);
    if (monitor->config.offline_callback != NULL)
    {
        monitor->config.offline_callback(monitor->config.user_context);
    }
}

static void app_safe_set_online(app_safe_monitor_t *monitor)
{
    if (monitor->state == APP_SAFE_STATE_ONLINE)
    {
        return;
    }

    monitor->state = APP_SAFE_STATE_ONLINE;
    monitor->offline_time_ms = 0U;
    BSP_LOG_INFO("%s online", monitor->config.name);
    if (monitor->config.online_callback != NULL)
    {
        monitor->config.online_callback(monitor->config.user_context);
    }
}

bsp_status_t app_safe_init(bsp_watchdog_t *watchdog)
{
    size_t index;

    for (index = 0U; index < APP_SAFE_MAX_MONITOR_COUNT; ++index)
    {
        app_safe_manager.monitors[index] = NULL;
    }
    app_safe_manager.monitor_count = 0U;
    app_safe_manager.watchdog = watchdog;
    app_safe_manager.is_initialized = true;
    return BSP_STATUS_OK;
}

bsp_status_t app_safe_set_watchdog(bsp_watchdog_t *watchdog)
{
    if (!app_safe_manager.is_initialized)
    {
        return BSP_STATUS_NOT_INITIALIZED;
    }
    app_safe_manager.watchdog = watchdog;
    return BSP_STATUS_OK;
}

bsp_status_t app_safe_monitor_init(app_safe_monitor_t *me,
                                   const app_safe_monitor_config_t *config)
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
    me->state = APP_SAFE_STATE_STARTING;
    me->is_registered = false;
    return BSP_STATUS_OK;
}

bsp_status_t app_safe_register(app_safe_monitor_t *monitor)
{
    if (!app_safe_manager.is_initialized)
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
    if (app_safe_manager.monitor_count >= APP_SAFE_MAX_MONITOR_COUNT)
    {
        return BSP_STATUS_NO_RESOURCE;
    }

    monitor->last_online_tick = osKernelGetTickCount();
    app_safe_manager.monitors[app_safe_manager.monitor_count] = monitor;
    ++app_safe_manager.monitor_count;
    monitor->is_registered = true;
    return BSP_STATUS_OK;
}

void app_safe_notify_online(app_safe_monitor_t *monitor)
{
    if ((monitor == NULL) || !monitor->is_registered)
    {
        return;
    }

    monitor->last_online_tick = osKernelGetTickCount();
    monitor->heartbeat_received = true;
}

void app_safe_process(void)
{
    const uint32_t current_tick = osKernelGetTickCount();
    size_t index;

    if (!app_safe_manager.is_initialized)
    {
        return;
    }

    for (index = 0U; index < app_safe_manager.monitor_count; ++index)
    {
        app_safe_monitor_t *const monitor = app_safe_manager.monitors[index];
        const uint32_t elapsed_ticks = current_tick - monitor->last_online_tick;
        const uint32_t elapsed_time_ms = app_safe_ticks_to_ms(elapsed_ticks);

        monitor->offline_time_ms = elapsed_time_ms;
        if (monitor->heartbeat_received && (elapsed_time_ms <= monitor->config.timeout_ms))
        {
            app_safe_set_online(monitor);
        }
        else if (elapsed_time_ms > monitor->config.timeout_ms)
        {
            app_safe_set_offline(monitor);
        }
    }

    if (app_safe_manager.watchdog != NULL)
    {
        (void)bsp_watchdog_refresh(app_safe_manager.watchdog);
    }
}

app_safe_state_t app_safe_get_state(const app_safe_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->state : APP_SAFE_STATE_OFFLINE;
}

uint32_t app_safe_get_offline_time_ms(const app_safe_monitor_t *monitor)
{
    return (monitor != NULL) ? monitor->offline_time_ms : UINT32_MAX;
}

bool app_safe_all_required_online(void)
{
    size_t index;

    if (!app_safe_manager.is_initialized)
    {
        return false;
    }
    for (index = 0U; index < app_safe_manager.monitor_count; ++index)
    {
        const app_safe_monitor_t *const monitor = app_safe_manager.monitors[index];
        if (monitor->config.required && (monitor->state != APP_SAFE_STATE_ONLINE))
        {
            return false;
        }
    }
    return true;
}

void app_safe_task(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    const uint32_t period_ticks = app_safe_ms_to_ticks(APP_SAFE_DEFAULT_TASK_PERIOD_MS);

    (void)argument;
    for (;;)
    {
        app_safe_process();
        wake_tick += period_ticks;
        (void)osDelayUntil(wake_tick);
    }
}
