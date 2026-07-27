#ifndef BSP_RTC_H
#define BSP_RTC_H

#include "bsp_common.h"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct bsp_rtc bsp_rtc_t;
    typedef struct bsp_rtc_device bsp_rtc_device_t;

    typedef struct
    {
        uint16_t year;
        uint8_t month;
        uint8_t day;
        uint8_t hour;
        uint8_t minute;
        uint8_t second;
        uint16_t millisecond;
    } bsp_rtc_time_t;

    typedef struct
    {
        bsp_device_ops_t super;
        bsp_status_t (*get_time)(bsp_rtc_t *me, bsp_rtc_time_t *time);
        bsp_status_t (*set_time)(bsp_rtc_t *me, const bsp_rtc_time_t *time);
        bsp_status_t (*get_unix_time)(bsp_rtc_t *me, uint64_t *unix_time_s);
    } bsp_rtc_ops_t;

    struct bsp_rtc
    {
        bsp_device_t super;
    };

    typedef struct
    {
        bsp_status_t (*init)(void *handle);
        bsp_status_t (*deinit)(void *handle);
        bsp_status_t (*get_time)(void *handle, bsp_rtc_time_t *time);
        bsp_status_t (*set_time)(void *handle, const bsp_rtc_time_t *time);
        bsp_status_t (*get_unix_time)(void *handle, uint64_t *unix_time_s);
    } bsp_rtc_driver_ops_t;

    struct bsp_rtc_device
    {
        bsp_rtc_t super;
        const bsp_rtc_driver_ops_t *driver_ops;
    };

    typedef struct
    {
        void *device_handle;
        const bsp_rtc_driver_ops_t *driver_ops;
    } bsp_rtc_config_t;

    bsp_status_t bsp_rtc_init(bsp_rtc_device_t *me, const bsp_rtc_config_t *config);
    bsp_rtc_t *bsp_rtc_as_base(bsp_rtc_device_t *me);
    bsp_status_t bsp_rtc_get_time(bsp_rtc_t *me, bsp_rtc_time_t *time);
    bsp_status_t bsp_rtc_set_time(bsp_rtc_t *me, const bsp_rtc_time_t *time);
    bsp_status_t bsp_rtc_get_unix_time(bsp_rtc_t *me, uint64_t *unix_time_s);

#ifdef __cplusplus
}
#endif

#endif
