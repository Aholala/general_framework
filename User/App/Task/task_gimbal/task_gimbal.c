#include "task_gimbal.h"

#include "app_config.h"
#include "app_gimbal.h"
#include "cmsis_os2.h"

void task_gimbal_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_gimbal_update((float)APP_GIMBAL_PERIOD_MS * 0.001F);
        wake_tick += APP_GIMBAL_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
