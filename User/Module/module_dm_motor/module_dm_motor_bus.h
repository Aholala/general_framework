#ifndef MODULE_DM_MOTOR_BUS_H
#define MODULE_DM_MOTOR_BUS_H

#include "module_dm_motor.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        bsp_can_t *can;
        module_dm_motor_t **motor_storage;
        size_t motor_capacity;
        size_t motor_count;
        size_t next_transmit_index;
        size_t maximum_transmits_per_cycle;
        uint32_t routed_frame_count;
        uint32_t unknown_frame_count;
        uint32_t transmit_error_count;
        bool is_initialized;
    } module_dm_motor_bus_t;

    module_motor_status_t module_dm_motor_bus_init(module_dm_motor_bus_t *me, bsp_can_t *can,
                                                   module_dm_motor_t **motor_storage,
                                                   size_t motor_capacity,
                                                   size_t maximum_transmits_per_cycle);
    module_motor_status_t module_dm_motor_bus_register(module_dm_motor_bus_t *me,
                                                       module_dm_motor_t *motor);
    module_motor_status_t module_dm_motor_bus_unregister(module_dm_motor_bus_t *me,
                                                         module_dm_motor_t *motor);
    module_motor_status_t module_dm_motor_bus_handle_feedback(module_dm_motor_bus_t *me,
                                                              const bsp_can_frame_t *frame);
    module_motor_status_t module_dm_motor_bus_update(module_dm_motor_bus_t *me, float delta_time_s);

#ifdef __cplusplus
}
#endif

#endif
