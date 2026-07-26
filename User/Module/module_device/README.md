# Module 通用设备基类

`module_device_t` 为非电机模块提供统一的 C11 对象基类。派生对象将 `super` 作为第一个成员，
并在各自 `.c` 文件中持有 `static const module_device_ops_t` 虚表。

```c
module_device_t *devices[] = {
    module_bmi088_as_device(&imu),
    module_dr16_as_device(&remote_control),
};

(void)module_device_start(devices[0]);
(void)module_device_start(devices[1]);
```

相同的 `module_device_update` 调用会由 vptr 分派：BMI088 读取一次六轴数据，DR16 则更新失联
计时。对象内存由调用者提供，基类不分配内存，也不依赖 MCU 或厂商 HAL。

## 基类成员

- `vptr`：指向派生类在 `.c` 中定义的 `static const module_device_ops_t`；
- `logical_name`：便于日志和诊断的人类可读名称；
- `registration_key`：由系统分配的稳定数字标识；
- `object_magic`：用于拒绝未构造对象和明显错误的基类指针；
- `is_initialized`：只能由 `module_device_complete_init` 在完整构造成功后提交。

## 派生类实现规则

```c
struct module_example
{
    module_device_t super;
    bsp_spi_t *spi;
};

static module_device_status_t module_example_update_impl(
    module_device_t *const device_base,
    uint32_t elapsed_time_ms)
{
    module_example_t *const me =
        MODULE_CONTAINER_OF(device_base, module_example_t, super);
    (void)elapsed_time_ms;
    return module_example_read(me);
}
```

`super` 必须是首成员。虚操作实现接收基类指针，通过 `MODULE_CONTAINER_OF` 恢复派生对象。
公共代码只能调用 `module_device_start`、`module_device_stop` 和 `module_device_update`，不能直接
访问 `vptr`。

派生类使用两阶段构造：

```c
status = module_device_init_base(&me->super, &s_module_example_ops,
                                 config->logical_name,
                                 config->registration_key);
if (status != MODULE_DEVICE_STATUS_OK)
{
    return MODULE_EXAMPLE_STATUS_INVALID_ARGUMENT;
}

/* 初始化派生类资源。任何失败路径都调用 abort。 */

status = module_device_complete_init(&me->super);
if (status != MODULE_DEVICE_STATUS_OK)
{
    module_device_abort_init(&me->super);
    return MODULE_EXAMPLE_STATUS_INITIALIZATION_FAILED;
}
```

禁止派生模块直接写入 `super.is_initialized`、`super.vptr` 或 `super.object_magic`。
`module_device_get_logical_name` 和 `module_device_get_registration_key` 提供只读访问。

## 状态处理

未构造对象返回 `MODULE_DEVICE_STATUS_NOT_INITIALIZED`，空虚操作返回
`MODULE_DEVICE_STATUS_UNSUPPORTED`，派生设备错误映射为
`MODULE_DEVICE_STATUS_OPERATION_FAILED`。需要具体故障原因时，再调用派生模块自己的接口。
