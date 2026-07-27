/**
 * @file bsp_storage.h
 * @author Ahola邱泽钦 (aholace0328@gmail.com)
 * @brief 存储设备通用抽象层头文件
 * @version 1.0
 * @date 2026-07-27
 * @copyright Copyright (c) 2026
 *
 * @note 统一的非易失存储基类，覆盖片内 Flash、QSPI/OSPI NOR、EEPROM、
 *       SDMMC 块设备和主机文件模拟器。上层只依赖读、编程、擦除、
 *       同步和几何信息，不依赖具体存储控制器。
 */

#ifndef BSP_STORAGE_H
#define BSP_STORAGE_H

#include "bsp_common.h" // 包含基础类型、状态码、设备基类等

#ifdef __cplusplus
extern "C"
{
#endif

    /* 前向声明，避免循环依赖 */
    typedef struct bsp_storage bsp_storage_t;
    typedef struct bsp_storage_device bsp_storage_device_t;

    /* ---------- 几何信息结构体 ---------- */
    /**
     * @brief 存储设备几何信息
     * @note 描述存储设备的物理特性，用于上层进行对齐和大小规划
     */
    typedef struct
    {
        uint64_t capacity_bytes;          // 总容量（字节）
        uint32_t read_alignment_bytes;    // 读取对齐要求（1 表示无要求）
        uint32_t program_alignment_bytes; // 编程对齐要求（1 表示无要求）
        uint32_t erase_block_bytes;       // 擦除块大小（字节）
        bool erase_is_required;           // 写入前是否需要先擦除
        bool supports_memory_mapping;     // 是否支持内存映射（如 XIP）
    } bsp_storage_geometry_t;

    /* ---------- 高层虚表（面向应用层） ---------- */
    /**
     * @brief 存储设备操作虚表，继承自 bsp_device_ops_t
     * @note 派生类必须保持 super 为第一成员
     *       所有函数均为必须实现（由 bsp_storage_init 校验）
     */
    typedef struct
    {
        bsp_device_ops_t super; // 父类虚表（含 deinit）

        /**
         * @brief 读取数据
         * @param me 基类指针
         * @param address 起始地址（偏移量，从 0 开始）
         * @param data 输出缓冲区
         * @param size 读取大小（字节）
         * @return 执行状态
         */
        bsp_status_t (*read)(bsp_storage_t *me, uint64_t address, void *data, size_t size);

        /**
         * @brief 编程（写入）数据
         * @param me 基类指针
         * @param address 起始地址（偏移量，从 0 开始）
         * @param data 待写入数据
         * @param size 写入大小（字节）
         * @return 执行状态
         */
        bsp_status_t (*program)(bsp_storage_t *me, uint64_t address, const void *data, size_t size);

        /**
         * @brief 擦除存储区域
         * @param me 基类指针
         * @param address 起始地址（偏移量，从 0 开始）
         * @param size 擦除大小（字节）
         * @return 执行状态
         */
        bsp_status_t (*erase)(bsp_storage_t *me, uint64_t address, size_t size);

        /**
         * @brief 同步存储设备
         * @param me 基类指针
         * @return 执行状态
         * @note 确保数据已持久化到物理介质
         */
        bsp_status_t (*sync)(bsp_storage_t *me);

        /**
         * @brief 获取几何信息
         * @param me 基类指针（const）
         * @param geometry 输出几何信息
         * @return 执行状态
         */
        bsp_status_t (*get_geometry)(const bsp_storage_t *me, bsp_storage_geometry_t *geometry);

    } bsp_storage_ops_t;

    /* ---------- 基类 ---------- */
    /**
     * @brief 存储设备基类结构体
     * @note 目前基类仅为 bsp_device_t 的简单包装
     */
    struct bsp_storage
    {
        bsp_device_t super; // 设备基类
    };

    /* ---------- 底层驱动操作表（平台实现） ---------- */
    /**
     * @brief 平台相关存储驱动操作表
     * @note 所有函数均为必须实现（由 bsp_storage_init 校验）
     *       驱动必须进行地址越界和对齐检查
     */
    typedef struct
    {
        bsp_status_t (*init)(void *handle);                                            // 初始化硬件
        bsp_status_t (*deinit)(void *handle);                                          // 反初始化
        bsp_status_t (*read)(void *handle, uint64_t address, void *data, size_t size); // 读取
        bsp_status_t (*program)(void *handle, uint64_t address, const void *data,
                                size_t size);                               // 编程
        bsp_status_t (*erase)(void *handle, uint64_t address, size_t size); // 擦除
        bsp_status_t (*sync)(void *handle);                                 // 同步
        bsp_status_t (*get_geometry)(const void *handle,
                                     bsp_storage_geometry_t *geometry); // 几何信息
    } bsp_storage_driver_ops_t;

    /* ---------- 派生设备对象 ---------- */
    /**
     * @brief 存储设备对象（派生类）
     */
    struct bsp_storage_device
    {
        bsp_storage_t super;                        // 基类实例
        const bsp_storage_driver_ops_t *driver_ops; // 底层驱动操作表
    };

    /* ---------- 配置结构 ---------- */
    /**
     * @brief 存储设备初始化配置
     */
    typedef struct
    {
        void *device_handle;                        // 平台设备句柄
        const bsp_storage_driver_ops_t *driver_ops; // 底层驱动表
    } bsp_storage_config_t;

    /* ---------- 公共 API ---------- */

    /**
     * @brief 初始化存储设备
     * @param me 设备对象指针
     * @param config 配置参数
     * @return 执行状态
     * @note 所有驱动函数指针必须非空
     */
    bsp_status_t bsp_storage_init(bsp_storage_device_t *me, const bsp_storage_config_t *config);

    /**
     * @brief 将派生对象转为基类指针（向上转型）
     * @param me 派生对象指针
     * @return 基类指针
     */
    bsp_storage_t *bsp_storage_as_base(bsp_storage_device_t *me);

    /**
     * @brief 读取数据
     * @param me 基类指针
     * @param address 起始地址（偏移量，从 0 开始）
     * @param data 输出缓冲区
     * @param size 读取大小（字节），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_storage_read(bsp_storage_t *me, uint64_t address, void *data, size_t size);

    /**
     * @brief 编程（写入）数据
     * @param me 基类指针
     * @param address 起始地址（偏移量，从 0 开始）
     * @param data 待写入数据
     * @param size 写入大小（字节），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_storage_program(bsp_storage_t *me, uint64_t address, const void *data,
                                     size_t size);

    /**
     * @brief 擦除存储区域
     * @param me 基类指针
     * @param address 起始地址（偏移量，从 0 开始）
     * @param size 擦除大小（字节），必须大于 0
     * @return 执行状态
     */
    bsp_status_t bsp_storage_erase(bsp_storage_t *me, uint64_t address, size_t size);

    /**
     * @brief 同步存储设备
     * @param me 基类指针
     * @return 执行状态
     */
    bsp_status_t bsp_storage_sync(bsp_storage_t *me);

    /**
     * @brief 获取几何信息
     * @param me 基类指针（const）
     * @param geometry 输出几何信息
     * @return 执行状态
     */
    bsp_status_t bsp_storage_get_geometry(const bsp_storage_t *me,
                                          bsp_storage_geometry_t *geometry);

#ifdef __cplusplus
}
#endif

#endif /* BSP_STORAGE_H */