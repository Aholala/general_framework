#include "bsp_log.h"

#include <stdarg.h>

void bsp_log_init(void)
{
    SEGGER_RTT_Init();
}

int bsp_log_printf(const char *format, ...)
{
    int character_count;
    va_list arguments;

    if (format == NULL)
    {
        return -1;
    }

    va_start(arguments, format);
    character_count = SEGGER_RTT_vprintf(BSP_LOG_BUFFER_INDEX, format, &arguments);
    va_end(arguments);
    return character_count;
}
