# DR16 / DT7 遥控器接收机模块 (module_dr16)

## 1. 模块概述

本模块解析 RoboMaster DR16 接收机输出的 18 字节遥控数据帧，提供摇杆通道、三段开关、鼠标、键盘和拨轮的解码。通过通用 `bsp_usart_t` 的 DMA 空闲中断接收，支持流式粘包/半帧处理。

**核心功能**：

- 18 字节 DR16 协议帧解析（摇杆、开关、鼠标、键盘、拨轮）
- 摇杆通道去中心、死区处理和归一化 `[-1.0, 1.0]`
- DMA 空闲中断接收，ISR 仅拷贝数据不解析
- STM32 DMA M0/M1 硬件双缓冲：一个缓冲区接收时，另一个缓冲区可交给 CPU
- 流式滑动窗口处理粘包、半帧和前导噪声
- 帧有效校验（通道范围 + 开关值合法性）
- 离线超时检测与数据安全清零
- 帧回调通知（任务上下文执行）
- `module_device_t` 基类接口

## 2. 设计边界

| **模块负责** | **模块不负责** |
| :--- | :--- |
| DR16 帧格式解析（摇杆、开关、鼠标、键盘、拨轮） | USART 波特率、校验位、停止位配置 |
| 摇杆去中心、死区和归一化 | DBUS 硬件反相电路 |
| DMA 空闲中断接收与双缓冲管理 | EXTI 或定时器触发调度 |
| 流式滑动窗口边界查找 | 遥控器与上位机通信协议 |
| 离线超时检测与数据清零 | 串口 GPIO 的 HAL 初始化 |
| 帧回调通知 | 控制模式切换等应用层逻辑 |

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_dr16_t                  (DR16 设备对象：USART、缓冲区、解码数据)
```

## 4. 串口参数

根据 RoboMaster 接收机手册：

| 参数 | 值 |
| :--- | :--- |
| 波特率 | 100000 bit/s |
| CubeMX 字长 | 9 Bits（包含偶校验位，实际有效数据为 8 Bits） |
| 校验 | 偶校验 (Even) |
| 停止位 | 2 |
| 流控 | 无 |
| 帧周期 | ~14 ms |
| 帧长 | 18 字节 |
| DBUS 电平 | 与普通 UART 相反，需要外部反相电路或 MCU RX 反相功能 |

这些参数属于 `board_config` 和平台串口驱动，不写死在 Module 中。

UART5 RX DMA 配置为 `DMA1_Stream0`、循环模式、存储器地址递增、Very High
优先级。每个 M0/M1 缓冲区为 36 字节；正常 DR16 空闲事件在收到 18 字节后触发，
平台层切换目标缓冲区并重装 DMA 计数。

## 5. API 参考

| 函数 | 说明 | 返回值 |
| :--- | :--- | :--- |
| `module_dr16_init` | 初始化 DR16 设备，注册 USART 回调 | `OK` / `INVALID_ARGUMENT` |
| `module_dr16_start` | 启动 DMA 空闲中断接收 | `OK` / `TRANSPORT_ERROR` |
| `module_dr16_stop` | 停止接收 | `OK` / `TRANSPORT_ERROR` |
| `module_dr16_process` | 任务上下文中处理 ISR 提交的数据块 | `OK` / `INVALID_FRAME` |
| `module_dr16_feed_data` | 手动注入任意长度数据流进行解析 | `OK` / `INVALID_FRAME` |
| `module_dr16_update_time` | 累加无有效帧时间，超时自动离线 | - |
| `module_dr16_get_data` | 获取当前解码后的遥控数据 | 数据指针 / `NULL` |
| `module_dr16_is_key_pressed` | 检查指定键盘按键是否按下 | `true` / `false` |
| `module_dr16_normalize_channel_value` | 将通道值归一化到 `[-1.0, 1.0]` | 归一化值 |
| `module_dr16_as_device` | 获取 `module_device_t` 基类指针 | `module_device_t` 指针 |

## 6. 使用示例

### 6.1 初始化与启动

```c
static module_dr16_t s_remote_control;
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_remote_control_dma_buffer[2][MODULE_DR16_DMA_BUFFER_SIZE];

const module_dr16_config_t config = {
    .logical_name = "operator_remote",
    .registration_key = 2U,
    .usart = board_dr16_usart,            // 已配置好的 USART BSP 对象
    .dma_receive_buffer = s_remote_control_dma_buffer,
    .channel_deadband = 20,               // 摇杆死区
    .offline_timeout_ms = 100U,           // 100ms 无帧视为离线
    .frame_callback = app_remote_control_updated,  // 每帧回调（可为 NULL）
    .user_context = NULL,
};

(void)module_dr16_init(&s_remote_control, &config);
(void)module_dr16_start(&s_remote_control);
```

或通过设备基类统一调度：

```c
module_device_t *remote_dev = module_dr16_as_device(&s_remote_control);
(void)module_device_start(remote_dev);
(void)module_device_update(remote_dev, elapsed_time_ms);
```

### 6.2 数据读取

```c
const module_dr16_process_data_t *remote = module_dr16_get_data(&s_remote_control);

if (remote->is_online) {
    float forward = remote->normalized_channel[3];   // 前进通道
    float strafe  = remote->normalized_channel[2];   // 平移通道

    if (module_dr16_is_key_pressed(&s_remote_control, MODULE_DR16_KEY_SHIFT)) {
        // 高速模式
    }

    if (remote->left_switch == MODULE_DR16_SWITCH_UP) {
        // 开关拨到上
    }
}
```

### 6.3 失联处理

```c
void control_task(void *param) {
    uint32_t last_time = get_time_ms();
    while (1) {
        uint32_t now = get_time_ms();
        uint32_t dt = now - last_time;
        last_time = now;

        // 处理接收 + 更新超时
        module_dr16_process(&s_remote_control);
        module_dr16_update_time(&s_remote_control, dt);

        const module_dr16_process_data_t *remote = module_dr16_get_data(&s_remote_control);
        if (!remote->is_online) {
            motor_disable_all();  // 失联时安全停电机
        }
        vTaskDelay(1);
    }
}
```

## 7. 数据通道说明

| 数据 | 范围 | 说明 |
| :--- | :--- | :--- |
| `channel[0~3]` | `[-660, 660]` | 四路摇杆（去中心后） |
| `normalized_channel[0~3]` | `[-1.0, 1.0]` | 归一化摇杆 |
| `left_switch` / `right_switch` | UP=1 / DOWN=2 / MIDDLE=3 | 三段开关 |
| `mouse_x/y/z` | 有符号 16 位 | 鼠标位移 |
| `mouse_left_pressed` / `mouse_right_pressed` | bool | 鼠标按键 |
| `keyboard` | 16 位位掩码 | 键盘按键（按位与检测） |
| `dial` / `normalized_dial` | `[-660, 660]` / `[-1.0, 1.0]` | 拨轮 |

## 8. 错误码速查

| 状态码 | 触发场景 |
| :--- | :--- |
| `INVALID_ARGUMENT` | 参数为空、死区超出范围、离线超时为 0 |
| `NOT_INITIALIZED` | 对象未初始化 |
| `TRANSPORT_ERROR` | USART 回调注册失败、DMA 启停失败 |
| `INVALID_FRAME` | 数据中未找到有效帧 |

## 9. 集成约束

- 一个 `bsp_usart_t` 对象只能保存一个事件回调，DR16 使用的串口不应被其他模块覆盖回调
- 平台必须提供硬件反相或外部反相电路；仅配置 100000 8E1 而未反相时不会得到稳定帧
- 拨轮字段来自常见 RoboMaster 固件扩展，标准手册将对应字节标为保留，使用前应在目标接收机上实测

## 10. 建议验证测试项

- [ ] 摇杆各通道范围正确，去中心后 `[-660, 660]`
- [ ] 死区功能：死区内通道值为 0
- [ ] 开关三段状态识别正确
- [ ] 鼠标位移和按键正确解码
- [ ] 键盘按键位掩码正确
- [ ] 拨轮解码
- [ ] 流式粘包处理（半帧、两帧粘在一起）
- [ ] 离线超时：停止发送后 `is_online` 变为 false
- [ ] 离线恢复：重新收到帧后 `is_online` 变为 true
- [ ] 接收覆盖计数递增（数据块未及时处理）

## 一页式接入顺序与可读信息

```c
/* 1. 先按 DBUS 参数初始化 UART5/其他 USART BSP，再准备 DMA 双缓冲。 */
static module_dr16_t remote;
static uint8_t dma_buffer[2][MODULE_DR16_DMA_BUFFER_SIZE];

/* 2. 配置中注入 USART、双缓冲、死区、离线超时和可选帧回调。 */
module_dr16_status_t status = module_dr16_init(&remote, &remote_config);

/* 3. 启动 Receive-to-Idle DMA 双缓冲接收。 */
status = module_dr16_start(&remote);

/* 4. 任务循环先处理 ISR 留下的数据，再推进离线计时。 */
status = module_dr16_process(&remote);
module_dr16_update_time(&remote, elapsed_time_ms);

/* 5. 获取只读快照并根据 is_online 决定是否采纳遥控命令。 */
const module_dr16_process_data_t *data = module_dr16_get_data(&remote);
if ((data != NULL) && data->is_online) {
    float forward = data->normalized_channel[1];
}

/* 6. 停机或重新配置 UART 前先 module_dr16_stop。 */
```

`module_dr16_process_data_t` 是主要可读结构体：

| 字段 | 含义 |
| --- | --- |
| `channel[4]` / `normalized_channel[4]` | 摇杆原始去中心值和 `[-1, 1]` 归一化值 |
| `left_switch` / `right_switch` | 左右三段开关 |
| `mouse_x/y/z`、鼠标按键 | 鼠标输入 |
| `keyboard` | 键盘位掩码；也可用 `module_dr16_is_key_pressed()` |
| `dial` / `normalized_dial` | 拨轮原始值和归一化值 |
| 帧计数和错误计数 | 有效帧、无效帧、覆盖和传输错误统计 |
| `is_online` | 是否仍在离线超时窗口内 |

`module_dr16_get_data()` 返回对象内部只读指针，不得释放或写入；下一次处理有效帧后内容会更新。
