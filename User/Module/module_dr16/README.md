# module_dr16 — DR16/DT7 遥控器

双缓冲 DMA 接收 + DBUS 协议解码。通过 `module_dr16_get_data()` 获取已解析的只读数据。

## 关键结构体

| 结构体 | 用途 |
|--------|------|
| `module_dr16_t` | 遥控器对象（`init/start/stop/process`） |
| `module_dr16_config_t` | 配置：`usart`, `dma_receive_buffer`, `channel_deadband`, `offline_timeout_ms` |
| `module_dr16_process_data_t` | 已解析的遥控数据（通过 `get_data()` 获取只读指针） |

## 读取遥控数据

```c
const module_dr16_process_data_t *data = module_dr16_get_data(&dr16);
if (data != NULL && data->is_online) {
    // 摇杆（归一化 [-1, 1]）
    float ch0 = data->normalized_channel[0];  // 右水平
    float ch1 = data->normalized_channel[1];  // 右垂直
    float ch2 = data->normalized_channel[2];  // 左水平
    float ch3 = data->normalized_channel[3];  // 左垂直

    // 开关
    if (data->right_switch == 1) { /* UP */ }

    // 鼠标
    int16_t mx = data->mouse_x;  // 鼠标 X 位移
    bool lbtn = data->mouse_left_pressed;

    // 键盘（位掩码）
    if (data->keyboard & 0x01) { /* W pressed */ }

    // 拨轮 [-1, 1]
    float dial = data->normalized_dial;
}
```

## `process_data_t` 字段一览

| 字段 | 类型 | 说明 |
|------|------|------|
| `channel[4]` | `int16_t` | 四路摇杆原始值 |
| `normalized_channel[4]` | `float` | 四路摇杆归一化 `[-1, 1]` |
| `left_switch` / `right_switch` | `int` | 三段开关（1=UP,2=DOWN,3=MIDDLE） |
| `mouse_x` / `mouse_y` / `mouse_z` | `int16_t` | 鼠标三轴位移 |
| `mouse_left_pressed` / `mouse_right_pressed` | `bool` | 鼠标按键 |
| `keyboard` | `uint16_t` | W/S/A/D/Shift/Ctrl/Q/E/R/F/G/Z/X/C/V/B 位掩码 |
| `dial` | `int16_t` | 拨轮原始值 |
| `normalized_dial` | `float` | 拨轮归一化 `[-1, 1]` |
| `valid_frame_count` | `uint32_t` | 有效帧计数 |
| `is_online` | `bool` | 是否在线 |

## 用法

```c
// 1. 初始化（DMA 缓冲区不能放在 DTCM）
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t dr16_dma_buf[2][MODULE_DR16_DMA_BUFFER_SIZE];

module_dr16_t dr16;
module_dr16_config_t cfg = {
    .logical_name = "dr16", .registration_key = 1,
    .usart = board_config_get_usart(BOARD_CONFIG_UART_DR16),
    .dma_receive_buffer = dr16_dma_buf,
    .channel_deadband = 10, .offline_timeout_ms = 100,
};
module_dr16_init(&dr16, &cfg);
module_dr16_start(&dr16);  // 启动 DMA 接收

// 2. 在任务中周期调用
module_dr16_process(&dr16);               // 解码最新帧
module_dr16_update_time(&dr16, elapsed_ms); // 更新超时

// 3. 读取
const module_dr16_process_data_t *d = module_dr16_get_data(&dr16);

// 4. 停用
module_dr16_stop(&dr16);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_dr16_init(me, cfg)` | 初始化 |
| `module_dr16_start(me)` | 启动 DMA 接收 |
| `module_dr16_stop(me)` | 停止 DMA |
| `module_dr16_process(me)` | 解码最新帧（任务上下文） |
| `module_dr16_update_time(me, ms)` | 更新超时计时 |
| `module_dr16_get_data(me)` | 获取已解析数据（只读指针，离线时 `is_online=false`） |
| `module_dr16_normalize_channel_value(raw)` | 单路摇杆去中心归一化 |
