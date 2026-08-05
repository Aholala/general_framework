#include "task_shooter.h"

#include "app_config.h"
#include "app_shooter.h"
#include "cmsis_os2.h"

void task_shooter_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_shooter_update((float)APP_SHOOTER_PERIOD_MS * 0.001F);
        wake_tick += APP_SHOOTER_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
