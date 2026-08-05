#include "task_safety.h"

#include "app_safety.h"
#include "cmsis_os2.h"

void task_safety_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_safety_process();
        wake_tick += APP_SAFETY_DEFAULT_TASK_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
