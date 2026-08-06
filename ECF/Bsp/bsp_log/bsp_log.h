#ifndef BSP_LOG_H
#define BSP_LOG_H

#include "SEGGER_RTT.h"

#define BSP_LOG_BUFFER_INDEX (0U)

#ifndef BSP_LOG_DISABLED
#define BSP_LOG_DISABLED (0)
#endif

void bsp_log_init(void);
int bsp_log_printf(const char *format, ...);

#define BSP_LOG_WRITE(type, color, format, ...)                                             \
    SEGGER_RTT_printf(BSP_LOG_BUFFER_INDEX, "  %s%s" format "\r\n%s", color, type,       \
                      ##__VA_ARGS__, RTT_CTRL_RESET)

#define BSP_LOG_CLEAR() SEGGER_RTT_WriteString(BSP_LOG_BUFFER_INDEX, "  " RTT_CTRL_CLEAR)

#if BSP_LOG_DISABLED
#define BSP_LOG_INFO(format, ...) ((void)0)
#define BSP_LOG_WARNING(format, ...) ((void)0)
#define BSP_LOG_ERROR(format, ...) ((void)0)
#else
#define BSP_LOG_INFO(format, ...)                                                            \
    BSP_LOG_WRITE("I:", RTT_CTRL_TEXT_BRIGHT_GREEN, format, ##__VA_ARGS__)
#define BSP_LOG_WARNING(format, ...)                                                         \
    BSP_LOG_WRITE("W:", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define BSP_LOG_ERROR(format, ...)                                                           \
    BSP_LOG_WRITE("E:", RTT_CTRL_TEXT_BRIGHT_RED, format, ##__VA_ARGS__)
#endif

#endif
