# 裁判系统 UI 模块 (module_referee_ui)

## 1. 模块概述

裁判系统客户端图形 UI 模块，通过裁判系统交互数据通道向裁判系统客户端发送图形绘制命令。支持直线、矩形、圆、椭圆、圆弧、数值和字符串七种图形类型，以及添加、修改、删除三种操作。

**核心功能**：

- 图形维护在环形队列中，按批次自动发送
- 自动选择协议支持的批次大小（7/5/2/1 个图形一批）
- 发送限频：避免超过客户端刷新率
- 字符串单独发送（最多 30 字节）
- 删除单个图层或全部图形
- 独立构建 15 字节图形位域编码
- `module_device_t` 基类接口

## 2. 设计边界

| **模块负责** | **模块不负责** |
| :--- | :--- |
| 图形位域编码（15 字节协议格式） | 图形坐标系统的屏幕映射 |
| 发送队列管理与批次选择 | 图形名称唯一性保证 |
| 发送限频调度 | 裁判系统服务器转发机制 |
| 删除图层/全部命令构建 | 客户端渲染性能 |
| 字符串文本发送 | 字体、字号、对齐等渲染属性 |

> UI 是非关键功能：发送失败只保留队列并返回错误，不得阻塞控制任务，也不得影响电机安全状态。

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_referee_ui_t            (UI 对象：裁判系统引用、环形队列、负载缓冲区)
```

## 4. 协议支持

| 批次类型 | 图形数 | 命令 ID |
| :--- | :--- | :--- |
| 单图形 | 1 | `0x0101` |
| 双图形 | 2 | `0x0102` |
| 五图形 | 5 | `0x0103` |
| 七图形 | 7 | `0x0104` |
| 字符串 | 1（文字） | `0x0110` |

## 5. API 参考

| 函数 | 说明 | 返回值 |
| :--- | :--- | :--- |
| `module_referee_ui_init` | 初始化 UI 模块 | `OK` / `INVALID_ARGUMENT` |
| `module_referee_ui_enqueue` | 入队一个图形（字符串类型除外） | `OK` / `OPERATION_FAILED`（队列满） |
| `module_referee_ui_delete_layer` | 删除指定图层的所有图形 | `OK` / `INVALID_ARGUMENT` |
| `module_referee_ui_delete_all` | 删除所有图形 | `OK` / `INVALID_ARGUMENT` |
| `module_referee_ui_send_string` | 发送字符串文本（立即发送，不入队） | `OK` / `INVALID_ARGUMENT` |

## 6. 使用示例

### 6.1 初始化

```c
#define UI_QUEUE_SIZE 20

static module_referee_ui_t s_ui;
static module_referee_ui_graphic_t s_ui_queue[UI_QUEUE_SIZE];

const module_referee_ui_config_t config = {
    .referee = &s_referee,                    // 已初始化的裁判系统对象
    .queue_storage = s_ui_queue,
    .queue_capacity = UI_QUEUE_SIZE,
    .sender_id = 0x0101,                       // 机器人 ID
    .receiver_id = 0x0102,                     // 客户端 ID
    .minimum_transmit_interval_ms = 50U,       // 50ms 限频
    .logical_name = "ui",
    .registration_key = 4U,
};

(void)module_referee_ui_init(&s_ui, &config);
```

### 6.2 添加图形

```c
module_referee_ui_graphic_t graphic = {
    .name = "R1",                              // 3 字节唯一名称
    .operation = MODULE_REFEREE_UI_OPERATION_ADD,
    .type = MODULE_REFEREE_UI_GRAPHIC_RECTANGLE,
    .layer = 0,
    .color = MODULE_REFEREE_UI_COLOR_GREEN,
    .width = 2,                                // 线宽
    .start_x = 100, .start_y = 100,            // 起点
    .end_x = 300, .end_y = 200,                 // 终点
    .start_angle = 0, .end_angle = 0,
    .radius = 0,
};

(void)module_referee_ui_enqueue(&s_ui, &graphic);
// 队列中的图形会在更新循环中自动发送
```

### 6.3 发送字符串

```c
module_referee_ui_graphic_t str_graphic = {
    .name = "TX",
    .operation = MODULE_REFEREE_UI_OPERATION_ADD,
    .type = MODULE_REFEREE_UI_GRAPHIC_STRING,
    .layer = 1,
    .color = MODULE_REFEREE_UI_COLOR_WHITE,
    .start_x = 500, .start_y = 300,
    /* ... */
};

(void)module_referee_ui_send_string(&s_ui, &str_graphic, "Hello RoboMaster!");
```

### 6.4 周期更新

```c
void ui_task(void *param) {
    while (1) {
        // 通过设备基类更新，自动处理队列发送
        module_device_t *ui_dev = &s_ui.super;
        (void)module_device_update(ui_dev, elapsed_time_ms);
        vTaskDelay(10);
    }
}
```

## 7. 图形类型

| 类型 | 说明 | 有效字段 |
| :--- | :--- | :--- |
| `GRAPHIC_LINE` | 直线 | `start_x/y`, `end_x/y`, `width`, `color` |
| `GRAPHIC_RECTANGLE` | 矩形（对角线） | `start_x/y`, `end_x/y`, `width`, `color` |
| `GRAPHIC_CIRCLE` | 圆 | `start_x/y`, `radius`, `width`, `color` |
| `GRAPHIC_ELLIPSE` | 椭圆 | `start_x/y`, `end_x/y`, `width`, `color` |
| `GRAPHIC_ARC` | 圆弧 | `start_x/y`, `radius`, `start_angle`, `end_angle`, `width`, `color` |
| `GRAPHIC_FLOAT` | 浮点数 | `start_x/y`, `end_x/y`, `color`, `layer` |
| `GRAPHIC_INTEGER` | 整数 | `start_x/y`, `end_x/y`, `color`, `layer` |
| `GRAPHIC_STRING` | 字符串 | `start_x/y`, `color`, `layer` |

## 8. 参数范围

| 字段 | 范围 | 说明 |
| :--- | :--- | :--- |
| `layer` | 0~9 | 图层号，大值在上层 |
| `color` | 0~8 | 见 `module_referee_ui_color_t` |
| `start_angle` / `end_angle` | 0~511 | 角度（弧度 × 512/2π） |
| `width` | 0~1023 | 线宽 |
| `start_x/y`, `end_x/y` | 0~2047 | 屏幕坐标 |
| `radius` | 0~1023 | 半径 |

## 9. 建议验证测试项

- [ ] 单图形发送（直线、矩形、圆、椭圆、圆弧）
- [ ] 批量发送（2/5/7 个图形）
- [ ] 字符串发送
- [ ] 图形更新（ADD → CHANGE）
- [ ] 删除单个图层
- [ ] 删除全部图形
- [ ] 发送限频：短时间大量入队不会被全部立即发出
- [ ] 队列满时 `dropped_graphic_count` 递增
- [ ] 图形参数合法性校验（超出范围返回 `INVALID_ARGUMENT`）
- [ ] 字符串长度超过 30 字节被拒绝
- [ ] 图形名称 3 字节唯一性（由调用者保证）
