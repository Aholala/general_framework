#include "task_imu.h"

#include "app_config.h"
#include "app_imu.h"
#include "cmsis_os2.h"

void task_imu_run(void *argument)
{
    uint32_t wake_tick = osKernelGetTickCount();
    (void)argument;
    for (;;)
    {
        app_imu_update((float)APP_IMU_PERIOD_MS * 0.001F);
        wake_tick += APP_IMU_PERIOD_MS;
        (void)osDelayUntil(wake_tick);
    }
}
