# 模块设备基类 (module_device) —— 完整使用指南

## 1. 模块概述

`module_device` 是提供给所有非电机模块的统一 C11 对象基类。它定义了设备对象的公共接口（启动、停止、更新）、生命周期管理（两阶段构造）以及运行时状态查询。派生类将 `super` 作为第一个成员，并在各自的 `.c` 文件中持有 `static const module_device_ops_t` 虚表。

本目录只包含 `module_device.c/.h`，负责设备基类、生命周期和虚表分派。

**核心功能**：

- 统一的对象生命周期管理（初始化、启动、停止、更新）
- 虚表分派（通过 `vptr` 实现多态）
- 对象有效性检查（魔数 + 初始化标志）
- 逻辑名称和注册键值管理

**设计哲学**：

- **零动态内存**：对象由调用者静态分配，基类不分配内存。
- **两阶段构造**：允许派生类在基类基础之上安全地初始化自己的资源，失败时可回滚。
- **纯虚接口**：`start`、`stop`、`update` 均可选实现，公共层会检查并返回 `UNSUPPORTED`。

## 2. 对象模型与继承

```text
module_device_t                    (基类：vptr、logical_name、magic、is_initialized)
└── module_xxx_t                   (派生类：super 为第一个成员，附加自有字段)
```

派生类的 `super` 必须是第一个成员，确保 `MODULE_CONTAINER_OF` 可以正确工作。

## 3. 核心 API

| 函数                                 | 说明                           | 返回值                                            |
| :----------------------------------- | :----------------------------- | :------------------------------------------------ |
| `module_device_init_base`            | 第一阶段构造：填充基类字段     | `OK` / `INVALID_ARGUMENT`                         |
| `module_device_complete_init`        | 第二阶段构造：标记对象已初始化 | `OK` / `INVALID_ARGUMENT` / `ALREADY_INITIALIZED` |
| `module_device_abort_init`           | 中止初始化：清除所有字段       | 无                                                |
| `module_device_start`                | 调用虚表 `start`               | `OK` / `NOT_INITIALIZED` / `UNSUPPORTED`          |
| `module_device_stop`                 | 调用虚表 `stop`                | 同上                                              |
| `module_device_update`               | 调用虚表 `update`              | 同上                                              |
| `module_device_is_initialized`       | 检查对象是否有效               | `true` / `false`                                  |
| `module_device_get_logical_name`     | 获取逻辑名称                   | 名称指针 / `NULL`                                 |
| `module_device_get_registration_key` | 获取注册键值                   | 键值 / `0`                                        |

## 4. 两阶段构造

派生类必须严格按照两阶段构造流程初始化对象：

```c
// 1. 调用 init_base，填充基类字段（此时 is_initialized = false）
status = module_device_init_base(&me->super, &s_module_xxx_ops,
                                 config->logical_name, config->registration_key);
if (status != MODULE_DEVICE_STATUS_OK) {
    return MODULE_XXX_STATUS_INVALID_ARGUMENT;
}

// 2. 初始化派生类自己的资源（如硬件初始化、分配缓冲区等）
//    若失败，调用 abort_init 回滚并返回错误
if (derived_init_failed) {
    module_device_abort_init(&me->super);
    return MODULE_XXX_STATUS_INITIALIZATION_FAILED;
}

// 3. 完成构造，提交对象为已初始化
status = module_device_complete_init(&me->super);
if (status != MODULE_DEVICE_STATUS_OK) {
    module_device_abort_init(&me->super);
    return MODULE_XXX_STATUS_INITIALIZATION_FAILED;
}
```

**关键约束**：派生类不得直接写入 `super.is_initialized`、`super.vptr` 或 `super.object_magic`，必须通过上述 API 完成。

## 5. 虚表实现

派生类在 `.c` 文件中定义一个 `static const module_device_ops_t`，实现 `start`、`stop`、`update` 三个虚函数（可部分为 `NULL`）：

```c
static module_device_status_t module_example_start(module_device_t *base) {
    module_example_t *me = MODULE_CONTAINER_OF(base, module_example_t, super);
    // 启动逻辑
}

static const module_device_ops_t s_module_example_ops = {
    .start = module_example_start,
    .stop = NULL,          // 不支持停止
    .update = module_example_update,
};
```

## 6. 使用示例

### 6.1 定义派生类

```c
// module_example.h
typedef struct {
    module_device_t super;
    bsp_spi_t *spi;
    uint32_t counter;
} module_example_t;

// module_example.c
static module_device_status_t module_example_update(module_device_t *base, uint32_t dt) {
    module_example_t *me = MODULE_CONTAINER_OF(base, module_example_t, super);
    me->counter += dt;
    return MODULE_DEVICE_STATUS_OK;
}

static const module_device_ops_t s_ops = {
    .start = NULL,
    .stop = NULL,
    .update = module_example_update,
};

module_device_status_t module_example_init(module_example_t *me, const module_example_config_t *cfg) {
    // 两阶段构造...
}
```

### 6.2 批量管理多个设备

```c
module_device_t *devices[] = {
    module_bmi088_as_device(&imu),
    module_dr16_as_device(&remote_control),
    module_buzzer_as_device(&buzzer),
};

for (size_t i = 0; i < ARRAY_SIZE(devices); i++) {
    module_device_start(devices[i]);
}

// 在循环中统一更新
for (size_t i = 0; i < ARRAY_SIZE(devices); i++) {
    module_device_update(devices[i], elapsed_time_ms);
}
```

## 7. 状态码映射

| 基类状态码                | 含义               | 派生类应如何使用                    |
| :------------------------ | :----------------- | :---------------------------------- |
| `MODULE_DEVICE_STATUS_OK` | 操作成功           | 派生公共 API 返回 `OK`              |
| `INVALID_ARGUMENT`        | 参数非法           | 返回对应的派生类 `INVALID_ARGUMENT` |
| `NOT_INITIALIZED`         | 对象未初始化       | 返回派生类 `NOT_INITIALIZED`        |
| `ALREADY_INITIALIZED`     | 重复初始化         | 用于 `complete_init` 防止重复提交   |
| `UNSUPPORTED`             | 虚操作未实现       | 公共 `start/stop/update` 调用时返回 |
| `OPERATION_FAILED`        | 派生类具体操作失败 | 派生虚函数实现返回此状态            |

派生类的公共接口应将基类错误映射为派生类自己的错误码，而虚函数实现应将派生错误映射为 `MODULE_DEVICE_STATUS_OPERATION_FAILED`。

## 8. 注意事项

- **魔数校验**：所有公共 API 在调用前应通过 `module_device_is_initialized` 检查对象有效性。
- **两阶段构造**：派生类必须在 `complete_init` 之前完成所有资源初始化，否则对象可能处于半成品状态。
- **虚表可选**：`start`、`stop`、`update` 均可为 `NULL`，公共层会安全处理。
- **不可复制**：对象初始化后只通过指针使用，禁止按值复制。

## 9. 建议验证测试项

- [ ] 派生类正确实现两阶段构造
- [ ] 构造成功后 `is_initialized` 为 `true`
- [ ] 构造失败调用 `abort_init` 后对象被清零
- [ ] 未初始化对象调用 `start/stop/update` 返回 `NOT_INITIALIZED`
- [ ] `init_base` 在任一生命周期虚函数为 `NULL` 时拒绝构造
- [ ] 逻辑名称和注册键值可通过 getter 正确获取
- [ ] 多个派生设备可通过统一数组管理

---

**总结**：`module_device` 为所有模块设备提供了轻量、零依赖的基类，统一了生命周期管理和多态接口。派生模块只需遵循两阶段构造和虚表约定，即可接入统一的调度框架，便于系统级设备管理和日志诊断。

## 一页式派生顺序与可读信息

```c
/* 1. 派生对象首成员必须是 module_device_t super。 */
typedef struct {
    module_device_t super;
    /* 派生实例自己的依赖和运行状态。 */
} module_example_t;

/* 2. 在 .c 中检查布局，并定义完整的只读生命周期虚表。 */
MODULE_STATIC_ASSERT_SUPER_FIRST(module_example_t);
static const module_device_ops_t s_example_ops = {
    .start = example_start_virtual,
    .stop = example_stop_virtual,
    .update = example_update_virtual,
};

/* 3. 构造第一阶段只绑定基类；任一虚函数为空都会失败。 */
status = module_device_init_base(&me->super, &s_example_ops,
                                 config->logical_name, config->registration_key);

/* 4. 初始化派生资源。失败时必须 module_device_abort_init。 */

/* 5. 所有派生资源成功后再提交对象。 */
status = module_device_complete_init(&me->super);

/* 6. 上层可通过基类统一 start/update/stop。 */
module_device_start(&me->super);
module_device_update(&me->super, elapsed_time_ms);
module_device_stop(&me->super);
```

| 可读取信息 | API | 说明 |
| --- | --- | --- |
| 初始化状态 | `module_device_is_initialized()` | 魔数、虚表、名称和提交状态均有效才返回 true |
| 逻辑名称 | `module_device_get_logical_name()` | 日志和诊断使用的稳定字符串 |
| 注册键 | `module_device_get_registration_key()` | 由 App/Board 分配的稳定数字标识 |
| `module_device_t` | 仅由基类 API 管理 | `vptr`、魔数和初始化标志禁止派生类直接修改 |

`module_device_t` 不是所有 Module 的强制父类；只有需要统一 `start/stop/update` 调度的设备才使用它。
