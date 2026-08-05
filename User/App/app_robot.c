#include "app_robot.h"

#include "app_command.h"
#include "app_config.h"
#include "app_vision.h"
#include "board_config.h"
#include "module_board_comm.h"
#include "module_dr16.h"

static module_dr16_t app_robot_dr16;
static uint8_t app_robot_dr16_buffers[2][MODULE_DR16_DMA_BUFFER_SIZE];
static module_board_comm_t app_robot_board_comm;
static bsp_can_t *app_robot_link_can;
static bool app_robot_dr16_is_local;
static bool app_robot_initialized;
static bool app_robot_board_comm_ok;
static bool app_robot_can_started;
static bool app_robot_dr16_ok;
static int app_robot_module_error;
static const char *app_robot_failed_step;

static bool app_robot_local_dr16_selected(void)
{
#if APP_BOARD_ROLE == APP_BOARD_ROLE_SINGLE
    return true;
#elif APP_BOARD_ROLE == APP_BOARD_ROLE_GIMBAL
    return APP_DR16_LOCATION == APP_DEVICE_LOCATION_GIMBAL;
#else
    return APP_DR16_LOCATION == APP_DEVICE_LOCATION_CHASSIS;
#endif
}

bsp_status_t app_robot_init(void)
{
    bsp_status_t status = BSP_STATUS_OK;
    app_command_config_t command_config;

    if (!board_config_is_initialized())
    {
        app_robot_failed_step = "board_config";
        return BSP_STATUS_NOT_INITIALIZED;
    }

    app_robot_link_can = board_config_get_can(BOARD_CONFIG_CAN_2);
    if (app_robot_link_can == NULL)
    {
        app_robot_failed_step = "can_get";
        return BSP_STATUS_IO_ERROR;
    }

    /* ---- 板间通信 ---- */
    if (status == BSP_STATUS_OK)
    {
        const module_board_comm_config_t cfg = {
            .can = app_robot_link_can,
            .base_identifier = APP_BOARD_COMM_BASE_IDENTIFIER,
            .transmit_timeout_ms = 2U,
            .offline_timeout_ms = 100U,
        };
        const int module_rc = (int)module_board_comm_init(&app_robot_board_comm, &cfg);
        if (module_rc != (int)MODULE_BOARD_COMM_STATUS_OK)
        {
            app_robot_module_error = module_rc;
            app_robot_failed_step = "board_comm_init";
            status = BSP_STATUS_IO_ERROR;
        }
        else
        {
            app_robot_board_comm_ok = true;
        }
    }

    /* ---- CAN 启动 ---- */
    if (status == BSP_STATUS_OK)
    {
        status = bsp_can_start(app_robot_link_can);
        if (status != BSP_STATUS_OK)
        {
            app_robot_failed_step = "can_start";
        }
        else
        {
            app_robot_can_started = true;
        }
    }

    /* ---- DR16 遥控器（本地连接时） ---- */
    app_robot_dr16_is_local = app_robot_local_dr16_selected();
    if ((status == BSP_STATUS_OK) && app_robot_dr16_is_local)
    {
        int module_rc;
        bsp_usart_t *const dr16_usart = board_config_get_usart(BOARD_CONFIG_UART_DR16);
        if (dr16_usart == NULL)
        {
            app_robot_failed_step = "dr16_usart_get";
            status = BSP_STATUS_IO_ERROR;
        }
        if (status == BSP_STATUS_OK)
        {
            const module_dr16_config_t cfg = {
                .logical_name = "dr16",
                .registration_key = 1U,
                .usart = dr16_usart,
                .dma_receive_buffer = app_robot_dr16_buffers,
                .channel_deadband = 10,
                .offline_timeout_ms = 100U,
                .frame_callback = NULL,
                .user_context = NULL,
            };
            module_rc = (int)module_dr16_init(&app_robot_dr16, &cfg);
            if (module_rc != (int)MODULE_DR16_STATUS_OK)
            {
                app_robot_module_error = module_rc;
                app_robot_failed_step = "dr16_init";
                status = BSP_STATUS_IO_ERROR;
            }
        }
        if (status == BSP_STATUS_OK)
        {
            module_rc = (int)module_dr16_start(&app_robot_dr16);
            if (module_rc != (int)MODULE_DR16_STATUS_OK)
            {
                app_robot_module_error = module_rc;
                app_robot_failed_step = "dr16_start";
                status = BSP_STATUS_IO_ERROR;
            }
            else
            {
                app_robot_dr16_ok = true;
            }
        }
    }

    /* ---- 遥控命令 ---- */
    if (status == BSP_STATUS_OK)
    {
        command_config = (app_command_config_t){
            .dr16 = app_robot_dr16_is_local ? &app_robot_dr16 : NULL,
            .board_comm = &app_robot_board_comm,
            .dr16_is_local = app_robot_dr16_is_local,
        };
        status = app_command_init(&command_config);
        if (status != BSP_STATUS_OK)
        {
            app_robot_failed_step = "command_init";
        }
    }

    /* ---- 成功 / 回滚 ---- */
    if (status == BSP_STATUS_OK)
    {
        app_robot_initialized = true;
        return BSP_STATUS_OK;
    }

    bsp_error_record(status, app_robot_failed_step, app_robot_module_error);
    /* 逆序回滚 */
    if (app_robot_dr16_ok)
    {
        (void)module_dr16_stop(&app_robot_dr16);
        app_robot_dr16_ok = false;
    }
    if (app_robot_can_started)
    {
        (void)bsp_can_stop(app_robot_link_can);
        app_robot_can_started = false;
    }
    if (app_robot_board_comm_ok)
    {
        (void)module_board_comm_deinit(&app_robot_board_comm);
        app_robot_board_comm_ok = false;
    }
    return status;
}

void app_robot_communication_update(uint32_t elapsed_time_ms)
{
    bsp_can_frame_t frame;
    if (!app_robot_initialized)
    {
        return;
    }
    if (app_robot_dr16_is_local)
    {
        (void)module_dr16_process(&app_robot_dr16);
        module_dr16_update_time(&app_robot_dr16, elapsed_time_ms);
    }
    module_board_comm_update_time(&app_robot_board_comm, elapsed_time_ms);
    while (bsp_can_receive(app_robot_link_can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK)
    {
        (void)module_board_comm_handle_frame(&app_robot_board_comm, &frame);
    }
    while (bsp_can_receive(app_robot_link_can, BSP_CAN_RX_FIFO_1, &frame) == BSP_STATUS_OK)
    {
        (void)module_board_comm_handle_frame(&app_robot_board_comm, &frame);
    }
}
