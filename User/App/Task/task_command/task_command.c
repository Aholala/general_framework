#include "task_command.h"

#include "app_command.h"
#include "app_config.h"
#include "app_robot.h"
#include "app_vision.h"
#include "cmsis_os2.h"

void task_command_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_robot_communication_update(APP_COMMAND_PERIOD_MS);
        app_command_update((float)APP_COMMAND_PERIOD_MS * 0.001F);
        app_vision_update(APP_COMMAND_PERIOD_MS);
        wake_tick += APP_COMMAND_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
