/**
 * @file bsp_adc.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief ADC 通用抽象层头文件，定义公共类型、操作接口和配置结构
 * @version 1.0
 * @date 2026-07-21
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef BSP_ADC_H
#define BSP_ADC_H

#include "bsp_common.h" /* 包含基础类型、状态码、设备基类等公共定义 */

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_adc bsp_adc_t;               /* ADC 基类（抽象接口） */
    typedef struct bsp_adc_device bsp_adc_device_t; /* ADC 派生设备类 */

    /**
     * @brief ADC 操作虚表（面向应用层）
     * 继承自 bsp_device_ops_t，包含 ADC 特有操作
     */
    typedef struct
    {
        bsp_device_ops_t super;                         /* 父类虚表（含 deinit） */
        bsp_status_t (*start)(bsp_adc_t *const me);     /* 启动转换 */
        bsp_status_t (*stop)(bsp_adc_t *const me);      /* 停止转换 */
        bsp_status_t (*calibrate)(bsp_adc_t *const me); /* 执行校准 */
        bsp_status_t (*read_raw)(bsp_adc_t *const me, uint32_t *raw_value,
                                 uint32_t timeout_ms); /* 阻塞读原始值 */
        bsp_status_t (*start_dma)(bsp_adc_t *const me, uint32_t *sample_buffer,
                                  size_t sample_count); /* 启动 DMA 采样 */
        bsp_status_t (*stop_dma)(bsp_adc_t *const me);  /* 停止 DMA 采样 */
    } bsp_adc_ops_t;

    /**
     * @brief ADC 基类结构体
     * 包含设备基类、回调、用户上下文、参考电压和最大原始值
     */
    struct bsp_adc
    {
        bsp_device_t super;            /* 继承的设备基类 */
        bsp_event_callback_t callback; /* 事件回调函数指针 */
        void *user_context;            /* 回调时的用户上下文 */
        float reference_voltage_v;     /* 参考电压（伏特） */
        uint32_t maximum_raw_value;    /* 最大原始值（由分辨率决定） */
    };

    /**
     * @brief 底层驱动操作表（平台相关）
     * 由具体 MCU 驱动实现，操作对象为设备句柄和通道号
     */
    typedef struct
    {
        bsp_status_t (*init)(void *device_handle, uint32_t channel);   /* 初始化硬件 */
        bsp_status_t (*deinit)(void *device_handle, uint32_t channel); /* 反初始化硬件 */
        bsp_status_t (*start)(void *device_handle, uint32_t channel);  /* 启动转换 */
        bsp_status_t (*stop)(void *device_handle, uint32_t channel);   /* 停止转换 */
        bsp_status_t (*calibrate)(void *device_handle);                /* 校准（无需通道） */
        bsp_status_t (*read_raw)(void *device_handle, uint32_t channel, uint32_t *raw_value,
                                 uint32_t timeout_ms); /* 阻塞读原始值 */
        bsp_status_t (*start_dma)(void *device_handle, uint32_t channel, uint32_t *sample_buffer,
                                  size_t sample_count);                  /* DMA 启动 */
        bsp_status_t (*stop_dma)(void *device_handle, uint32_t channel); /* DMA 停止 */
    } bsp_adc_driver_ops_t;

    /**
     * @brief ADC 设备对象（派生类）
     * 包含基类、底层驱动操作表指针和逻辑通道号
     */
    struct bsp_adc_device
    {
        bsp_adc_t super;                        /* 基类实例 */
        const bsp_adc_driver_ops_t *driver_ops; /* 底层驱动操作表 */
        uint32_t channel;                       /* 逻辑通道号 */
    };

    /**
     * @brief ADC 初始化配置结构
     */
    typedef struct
    {
        void *device_handle;                    /* 平台设备句柄（如 ADC_HandleTypeDef*） */
        const bsp_adc_driver_ops_t *driver_ops; /* 底层驱动操作表 */
        uint32_t channel;                       /* 通道号 */
        uint8_t resolution_bits;                /* 分辨率位数（1~31） */
        float reference_voltage_v;              /* 参考电压（必须 >0 且有限） */
        bsp_event_callback_t callback;          /* 事件回调（可为 NULL） */
        void *user_context;                     /* 用户上下文 */
    } bsp_adc_config_t;

    /* ----- 公共 API 声明 ----- */

    /**
     * @brief 初始化 ADC 设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     */
    bsp_status_t bsp_adc_init(bsp_adc_device_t *const me, const bsp_adc_config_t *const config);

    /**
     * @brief 将设备对象转为基类指针
     * @param me 设备对象指针
     * @return 基类指针
     */
    bsp_adc_t *bsp_adc_as_base(bsp_adc_device_t *const me);

    /**
     * @brief 设置或更改事件回调
     * @param me 基类指针
     * @param callback 回调函数
     * @param user_context 用户上下文
     * @return 执行状态
     */
    bsp_status_t bsp_adc_set_callback(bsp_adc_t *const me, bsp_event_callback_t callback,
                                      void *user_context);

    /**
     * @brief 启动 ADC 转换
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_adc_start(bsp_adc_t *const me);

    /**
     * @brief 停止 ADC 转换
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_adc_stop(bsp_adc_t *const me);

    /**
     * @brief 校准 ADC
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_adc_calibrate(bsp_adc_t *const me);

    /**
     * @brief 阻塞读取原始采样值
     * @param me 基类指针
     * @param raw_value 输出原始码
     * @param timeout_ms 超时时间（ms）
     * @return 执行状态
     */
    bsp_status_t bsp_adc_read_raw(bsp_adc_t *const me, uint32_t *raw_value, uint32_t timeout_ms);

    /**
     * @brief 读取归一化值（0.0 ~ 1.0）
     * @param me 基类指针
     * @param normalized_value 输出归一化值
     * @param timeout_ms 超时时间
     * @return 执行状态
     */
    bsp_status_t bsp_adc_read_normalized(bsp_adc_t *const me, float *normalized_value,
                                         uint32_t timeout_ms);

    /**
     * @brief 读取电压值（参考电压 * 归一化值）
     * @param me 基类指针
     * @param voltage_v 输出电压（伏特）
     * @param timeout_ms 超时时间
     * @return 执行状态
     */
    bsp_status_t bsp_adc_read_voltage(bsp_adc_t *const me, float *voltage_v, uint32_t timeout_ms);

    /**
     * @brief 启动 DMA 方式采样
     * @param me 基类指针
     * @param sample_buffer 用户缓冲区（需保持有效）
     * @param sample_count 采样数量
     * @return 执行状态
     */
    bsp_status_t bsp_adc_start_dma(bsp_adc_t *const me, uint32_t *sample_buffer,
                                   size_t sample_count);

    /**
     * @brief 停止 DMA 采样
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_adc_stop_dma(bsp_adc_t *const me);

    /**
     * @brief 事件通知函数（供底层驱动调用，向上层传递事件）
     * @param me 基类指针
     * @param event 事件类型
     * @param status 状态码
     * @param transferred_size 传输数量（如 DMA 计数）
     */
    void bsp_adc_notify(bsp_adc_t *const me, bsp_event_t event, bsp_status_t status,
                        size_t transferred_size);

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */