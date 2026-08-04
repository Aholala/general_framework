# app_safe

`app_safe` provides centralized heartbeat and offline detection for application devices and
tasks. Monitor storage is caller-owned and no dynamic memory is used.

## Usage

Create a persistent monitor object, initialize it during application startup, then register it:

```c
static app_safe_monitor_t remote_monitor;

static void remote_offline(void *user_context)
{
    (void)user_context;
    /* Publish an emergency-stop request. Keep callbacks non-blocking. */
}

void app_command_init_safety(void)
{
    const app_safe_monitor_config_t config = {
        .name = "DR16",
        .timeout_ms = 200U,
        .required = true,
        .offline_callback = remote_offline,
        .online_callback = NULL,
        .user_context = NULL,
    };

    (void)app_safe_monitor_init(&remote_monitor, &config);
    (void)app_safe_register(&remote_monitor);
}
```

After a complete frame has passed protocol validation, refresh its monitor:

```c
app_safe_notify_online(&remote_monitor);
```

Do not refresh on raw interrupt activity or an invalid frame. Offline and recovery callbacks run
once per state transition in the SafeTask context. They must only update state or publish a stop
request; they must not block.

## Recommended validation

- A monitor remains STARTING until its first valid heartbeat.
- A never-online monitor becomes OFFLINE after its timeout.
- Offline and online callbacks each run once per transition.
- Tick counter wraparound does not cause a false recovery.
- Optional monitors do not block `app_safe_all_required_online()`.
- Registration beyond `APP_SAFE_MAX_MONITOR_COUNT` is rejected.
- Hardware watchdog refresh is performed only after a watchdog is attached.
