# DR16 / DT7 接收机模块

## 串口配置

根据 RoboMaster 接收机手册：

- 波特率：100000 bit/s；
- 数据位：8；
- 校验：偶校验；
- 停止位：1；
- 流控：无；
- 帧周期：约 14 ms；
- 帧长：18 字节；
- DBUS 电平与普通 UART 相反，需要外部反相电路或 MCU RX 反相功能。

这些参数属于 `board_config` 和平台串口驱动，不能写死在 Module 中。

## 初始化与 DMA Receive-to-Idle

```c
static module_dr16_t s_remote_control;

const module_dr16_config_t config = {
    .logical_name = "operator_remote",
    .registration_key = 2U,
    .usart = board_dr16_usart,
    .channel_deadband = 20,
    .offline_timeout_ms = 100U,
    .frame_callback = app_remote_control_updated,
    .user_context = NULL,
};

(void)module_dr16_init(&s_remote_control, &config);
(void)module_dr16_start(&s_remote_control);
```

也可以通过基类指针调度；DR16 的虚 `Update` 将参数作为失联计时增量：

```c
module_device_t *remote_device = module_dr16_as_device(&s_remote_control);
(void)module_device_start(remote_device);
(void)module_device_update(remote_device, elapsed_time_ms);
```

模块会注册 USART 回调并立即重新启动 DMA Receive-to-Idle。ISR 只复制已接收的数据块、
记录事件和恢复接收，不执行协议解析，也不调用用户回调。任务上下文必须周期调用：

```c
(void)module_dr16_process(&s_remote_control);
module_dr16_update_time(&s_remote_control, elapsed_time_ms);
```

通过 `module_device_update` 调度时会自动执行这两个步骤。若前一数据块尚未处理，
`receive_overrun_count` 会递增；DMA 恢复失败时 `transport_error_count` 会递增。

也可以不启动内部接收，直接将任意长度数据块传给 `module_dr16_feed_data`。解析器会滑动寻找合法
18 字节边界，因此能够处理半帧、粘包和前导错位字节。

## 数据读取

```c
const module_dr16_data_t *remote = module_dr16_get_data(&s_remote_control);

if (remote->is_online)
{
    float forward = remote->normalized_channel[3];
    float strafe = remote->normalized_channel[2];

    if (module_dr16_is_key_pressed(&s_remote_control, MODULE_DR16_KEY_SHIFT))
    {
        /* high-speed mode */
    }
}
```

四个摇杆通道减去中心值 1024，死区后范围约为 `[-660, 660]`；归一化通道限制在
`[-1.0F, 1.0F]`。开关值为 UP=1、DOWN=2、MIDDLE=3。

字节 16-17 在手册中标为保留字段，但 RoboMaster 常用遥控器固件将其作为左上拨轮。本模块以
`dial`/`normalized_dial` 暴露，并在文档中保留这一兼容性说明。

## 失联处理

在系统时间更新处调用：

```c
module_dr16_update_time(&s_remote_control, elapsed_time_ms);
```

超过 `offline_timeout_ms` 后，模块将所有摇杆、鼠标、键盘、拨轮和开关清零，并设置
`is_online=false`。App 必须以在线状态作为电机使能和运动控制的安全条件。

## 接口速查

- `module_dr16_init`：初始化解析状态并安装 USART 回调；
- `module_dr16_start` / `module_dr16_stop`：控制 DMA Receive-to-Idle；
- `module_dr16_process`：在任务上下文处理 ISR 提交的数据块；
- `module_dr16_feed_data`：输入任意长度数据流；
- `module_dr16_update_time`：累计无有效帧时间；
- `module_dr16_get_data`：获取最近一帧控制数据；
- `module_dr16_is_key_pressed`：检查键盘位；
- `module_dr16_as_device`：取得通用设备基类。

## 集成约束

一个 `bsp_usart_t` 对象只能保存一个事件回调，因此 DR16 使用的串口不应再被其他 Module
直接覆盖回调。平台必须提供硬件反相或外部反相电路；仅配置 100000 8E1 而未反相时，解析器
不会得到稳定有效帧。拨轮字段来自常见 RoboMaster 固件扩展，标准手册将对应字节标为保留，
使用前应在目标接收机上实测。
