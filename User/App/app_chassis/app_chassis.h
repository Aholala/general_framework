#ifndef APP_CHASSIS_H
#define APP_CHASSIS_H

#include "alg_swerve.h"
#include "module_board_comm.h"
#include "module_swerve.h"

#include <stdbool.h>

typedef struct
{
    alg_swerve_t *kinematics;
    module_swerve_t *modules[ALG_SWERVE_RECTANGULAR_MODULE_COUNT];
    module_board_comm_t *board_comm;
} app_chassis_config_t;

bool app_chassis_init(const app_chassis_config_t *config);
void app_chassis_update(float delta_time_s);

#endif
