#include "module_referee_ui.h"

#include <string.h>

#define MODULE_REFEREE_UI_INTERACTION_HEADER_SIZE (6U)
#define MODULE_REFEREE_UI_GRAPHIC_SIZE (15U)
#define MODULE_REFEREE_UI_STRING_SIZE (30U)

static module_device_status_t module_referee_ui_start_device(module_device_t *device);
static module_device_status_t module_referee_ui_stop_device(module_device_t *device);
static module_device_status_t module_referee_ui_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms);

static const module_device_ops_t module_referee_ui_device_ops = {
    .start = module_referee_ui_start_device,
    .stop = module_referee_ui_stop_device,
    .update = module_referee_ui_update_device,
};

static void module_referee_ui_write_uint16(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
}

static void module_referee_ui_write_uint32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)value;
    data[1] = (uint8_t)(value >> 8U);
    data[2] = (uint8_t)(value >> 16U);
    data[3] = (uint8_t)(value >> 24U);
}

static bool module_referee_ui_graphic_is_valid(const module_referee_ui_graphic_t *graphic)
{
    return (graphic != NULL) && (graphic->operation <= MODULE_REFEREE_UI_OPERATION_DELETE) &&
           (graphic->type <= MODULE_REFEREE_UI_GRAPHIC_STRING) && (graphic->layer <= 9U) &&
           (graphic->color <= MODULE_REFEREE_UI_COLOR_WHITE) && (graphic->start_angle <= 511U) &&
           (graphic->end_angle <= 511U) && (graphic->width <= 1023U) &&
           (graphic->start_x <= 2047U) && (graphic->start_y <= 2047U) &&
           (graphic->radius <= 1023U) && (graphic->end_x <= 2047U) && (graphic->end_y <= 2047U);
}

static void module_referee_ui_encode_graphic(const module_referee_ui_graphic_t *graphic,
                                             uint8_t output[MODULE_REFEREE_UI_GRAPHIC_SIZE])
{
    const uint32_t control_word =
        ((uint32_t)graphic->operation & 0x07U) | (((uint32_t)graphic->type & 0x07U) << 3U) |
        (((uint32_t)graphic->layer & 0x0FU) << 6U) | (((uint32_t)graphic->color & 0x0FU) << 10U) |
        (((uint32_t)graphic->start_angle & 0x1FFU) << 14U) |
        (((uint32_t)graphic->end_angle & 0x1FFU) << 23U);
    const uint32_t start_word = ((uint32_t)graphic->width & 0x3FFU) |
                                (((uint32_t)graphic->start_x & 0x7FFU) << 10U) |
                                (((uint32_t)graphic->start_y & 0x7FFU) << 21U);
    const uint32_t end_word = ((uint32_t)graphic->radius & 0x3FFU) |
                              (((uint32_t)graphic->end_x & 0x7FFU) << 10U) |
                              (((uint32_t)graphic->end_y & 0x7FFU) << 21U);
    memcpy(output, graphic->name, MODULE_REFEREE_UI_NAME_SIZE);
    module_referee_ui_write_uint32(&output[3], control_word);
    module_referee_ui_write_uint32(&output[7], start_word);
    module_referee_ui_write_uint32(&output[11], end_word);
}

static uint16_t module_referee_ui_batch_command(size_t graphic_count)
{
    switch (graphic_count)
    {
    case 1U:
        return 0x0101U;
    case 2U:
        return 0x0102U;
    case 5U:
        return 0x0103U;
    case 7U:
        return 0x0104U;
    default:
        return 0U;
    }
}

static size_t module_referee_ui_select_batch(size_t queue_count)
{
    return (queue_count >= 7U) ? 7U : ((queue_count >= 5U) ? 5U : ((queue_count >= 2U) ? 2U : 1U));
}

static void module_referee_ui_encode_interaction_header(module_referee_ui_t *me,
                                                        uint16_t data_command_id)
{
    module_referee_ui_write_uint16(&me->payload_buffer[0], data_command_id);
    module_referee_ui_write_uint16(&me->payload_buffer[2], me->sender_id);
    module_referee_ui_write_uint16(&me->payload_buffer[4], me->receiver_id);
}

module_device_status_t module_referee_ui_init(module_referee_ui_t *me,
                                              const module_referee_ui_config_t *config)
{
    if ((me == NULL) || (config == NULL) || (config->referee == NULL) ||
        !module_device_is_initialized(&config->referee->super) || (config->queue_storage == NULL) ||
        (config->queue_capacity == 0U) || (config->sender_id == 0U) || (config->receiver_id == 0U))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (module_device_init_base(&me->super, &module_referee_ui_device_ops, config->logical_name,
                                config->registration_key) != MODULE_DEVICE_STATUS_OK)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    me->referee = config->referee;
    me->queue = config->queue_storage;
    me->queue_capacity = config->queue_capacity;
    me->read_index = 0U;
    me->write_index = 0U;
    me->queue_count = 0U;
    me->sender_id = config->sender_id;
    me->receiver_id = config->receiver_id;
    me->minimum_transmit_interval_ms = config->minimum_transmit_interval_ms;
    me->transmit_elapsed_time_ms = config->minimum_transmit_interval_ms;
    me->dropped_graphic_count = 0U;
    me->is_started = false;
    return module_device_complete_init(&me->super);
}

module_device_status_t module_referee_ui_enqueue(module_referee_ui_t *me,
                                                 const module_referee_ui_graphic_t *graphic)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super) ||
        !module_referee_ui_graphic_is_valid(graphic) ||
        (graphic->type == MODULE_REFEREE_UI_GRAPHIC_STRING))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    if (me->queue_count >= me->queue_capacity)
    {
        ++me->dropped_graphic_count;
        return MODULE_DEVICE_STATUS_OPERATION_FAILED;
    }
    me->queue[me->write_index] = *graphic;
    me->write_index = (me->write_index + 1U) % me->queue_capacity;
    ++me->queue_count;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_referee_ui_send_delete(module_referee_ui_t *me,
                                                            uint8_t operation, uint8_t layer)
{
    module_referee_status_t status;
    module_referee_ui_encode_interaction_header(me, 0x0100U);
    me->payload_buffer[6] = operation;
    me->payload_buffer[7] = layer;
    status = module_referee_transmit(me->referee, MODULE_REFEREE_UI_COMMAND_INTERACTION,
                                     me->payload_buffer, 8U, BSP_TRANSFER_MODE_INTERRUPT);
    return (status == MODULE_REFEREE_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

module_device_status_t module_referee_ui_delete_layer(module_referee_ui_t *me, uint8_t layer)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super) || (layer > 9U))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    return module_referee_ui_send_delete(me, 1U, layer);
}

module_device_status_t module_referee_ui_delete_all(module_referee_ui_t *me)
{
    if ((me == NULL) || !module_device_is_initialized(&me->super))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    return module_referee_ui_send_delete(me, 2U, 0U);
}

module_device_status_t module_referee_ui_send_string(module_referee_ui_t *me,
                                                     const module_referee_ui_graphic_t *graphic,
                                                     const char *text)
{
    size_t text_length;
    module_referee_status_t status;
    if ((me == NULL) || !module_device_is_initialized(&me->super) || (text == NULL) ||
        !module_referee_ui_graphic_is_valid(graphic) ||
        (graphic->type != MODULE_REFEREE_UI_GRAPHIC_STRING))
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    text_length = strlen(text);
    if (text_length > MODULE_REFEREE_UI_STRING_SIZE)
    {
        return MODULE_DEVICE_STATUS_INVALID_ARGUMENT;
    }
    module_referee_ui_encode_interaction_header(me, 0x0110U);
    module_referee_ui_encode_graphic(graphic, &me->payload_buffer[6]);
    memset(&me->payload_buffer[21], 0, MODULE_REFEREE_UI_STRING_SIZE);
    memcpy(&me->payload_buffer[21], text, text_length);
    status = module_referee_transmit(me->referee, MODULE_REFEREE_UI_COMMAND_INTERACTION,
                                     me->payload_buffer, 51U, BSP_TRANSFER_MODE_INTERRUPT);
    return (status == MODULE_REFEREE_STATUS_OK) ? MODULE_DEVICE_STATUS_OK
                                                : MODULE_DEVICE_STATUS_OPERATION_FAILED;
}

static module_device_status_t module_referee_ui_start_device(module_device_t *device)
{
    module_referee_ui_t *const me = MODULE_CONTAINER_OF(device, module_referee_ui_t, super);
    me->is_started = true;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_referee_ui_stop_device(module_device_t *device)
{
    module_referee_ui_t *const me = MODULE_CONTAINER_OF(device, module_referee_ui_t, super);
    me->is_started = false;
    return MODULE_DEVICE_STATUS_OK;
}

static module_device_status_t module_referee_ui_update_device(module_device_t *device,
                                                              uint32_t elapsed_time_ms)
{
    module_referee_ui_t *const me = MODULE_CONTAINER_OF(device, module_referee_ui_t, super);
    size_t batch_size;
    size_t batch_index;
    module_referee_status_t status;
    me->transmit_elapsed_time_ms = (elapsed_time_ms > UINT32_MAX - me->transmit_elapsed_time_ms)
                                       ? UINT32_MAX
                                       : me->transmit_elapsed_time_ms + elapsed_time_ms;
    if (!me->is_started || (me->queue_count == 0U) ||
        (me->transmit_elapsed_time_ms < me->minimum_transmit_interval_ms))
    {
        return MODULE_DEVICE_STATUS_OK;
    }
    batch_size = module_referee_ui_select_batch(me->queue_count);
    module_referee_ui_encode_interaction_header(me, module_referee_ui_batch_command(batch_size));
    for (batch_index = 0U; batch_index < batch_size; ++batch_index)
    {
        const size_t queue_index = (me->read_index + batch_index) % me->queue_capacity;
        module_referee_ui_encode_graphic(
            &me->queue[queue_index],
            &me->payload_buffer[MODULE_REFEREE_UI_INTERACTION_HEADER_SIZE +
                                batch_index * MODULE_REFEREE_UI_GRAPHIC_SIZE]);
    }
    status = module_referee_transmit(
        me->referee, MODULE_REFEREE_UI_COMMAND_INTERACTION, me->payload_buffer,
        MODULE_REFEREE_UI_INTERACTION_HEADER_SIZE + batch_size * MODULE_REFEREE_UI_GRAPHIC_SIZE,
        BSP_TRANSFER_MODE_INTERRUPT);
    if (status != MODULE_REFEREE_STATUS_OK)
    {
        return MODULE_DEVICE_STATUS_OPERATION_FAILED;
    }
    me->read_index = (me->read_index + batch_size) % me->queue_capacity;
    me->queue_count -= batch_size;
    me->transmit_elapsed_time_ms = 0U;
    return MODULE_DEVICE_STATUS_OK;
}
