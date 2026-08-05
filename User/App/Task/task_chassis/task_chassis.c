#include "task_chassis.h"

#include "app_chassis.h"
#include "app_config.h"
#include "cmsis_os2.h"

void task_chassis_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_chassis_update((float)APP_CHASSIS_PERIOD_MS * 0.001F);
        wake_tick += APP_CHASSIS_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
