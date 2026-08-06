# module_board_comm — 板间 CAN 通信

云台板↔底盘板的 Classic CAN 数据协议。遥控输入/云台/底盘/发射机构数据分包传输，自动组装。

## 关键结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `module_board_comm_t` | 通信对象 | 内部状态，通过 getter 读取 |
| `module_board_comm_config_t` | 配置 | `can`, `base_identifier`(CAN ID 基址), `transmit_timeout_ms`, `offline_timeout_ms` |
| `module_board_comm_remote_process_data_t` | 遥控数据 | `channel[4]`, `left_switch`, `right_switch`, `mouse_*`, `keyboard`, `dial`, `is_online` |
| `module_board_comm_gimbal_process_data_t` | 云台数据 | `yaw_rad`, `pitch_rad`, `yaw_velocity_rad_per_s`, `imu_valid`, `motors_online` |
| `module_board_comm_chassis_process_data_t` | 底盘数据 | `velocity_x_m_per_s`, `velocity_y_m_per_s`, `angular_velocity_rad_per_s`, `self_lock_active` |
| `module_board_comm_shooter_process_data_t` | 发射数据 | `state`, `jam_retry_count`, `friction_ready`, `fire_permission` |

## 读取数据

```c
const module_board_comm_gimbal_process_data_t *g =
    module_board_comm_get_gimbal(&board_comm);
if (g != NULL) {
    float yaw   = g->yaw_rad;
    float pitch = g->pitch_rad;
    bool  imu_ok = g->imu_valid;
}

const module_board_comm_remote_process_data_t *r =
    module_board_comm_get_remote(&board_comm);
if (r != NULL && r->is_online) {
    float ch0 = module_dr16_normalize_channel_value(r->channel[0]);
}
```

## 用法

```c
module_board_comm_t link;
module_board_comm_config_t cfg = {
    .can = board_config_get_can(BOARD_CONFIG_CAN_2),
    .base_identifier = 0x100, .transmit_timeout_ms = 2, .offline_timeout_ms = 100,
};
module_board_comm_init(&link, &cfg);
// CAN 由 App 层启动: bsp_can_start(can);

// 接收：在 CAN 轮询循环中
bsp_can_frame_t frame;
while (bsp_can_receive(can, BSP_CAN_RX_FIFO_0, &frame) == BSP_STATUS_OK) {
    module_board_comm_handle_frame(&link, &frame);
}

// 发送：周期性调用
module_board_comm_remote_process_data_t remote = { ... };
module_board_comm_send_remote(&link, &remote);

// 超时维护
module_board_comm_update_time(&link, elapsed_ms);

// 反初始化
module_board_comm_deinit(&link);
```

## API 速查

| 函数 | 功能 |
|------|------|
| `module_board_comm_init(me, cfg)` | 初始化 |
| `module_board_comm_deinit(me)` | 反初始化（清零全部字段） |
| `module_board_comm_handle_frame(me, frame)` | 处理接收帧 |
| `module_board_comm_update_time(me, ms)` | 更新各通道超时计时 |
| `module_board_comm_send_remote/chassis/gimbal/shooter(me, data)` | 发送各类数据 |
| `module_board_comm_get_remote/gimbal/chassis/shooter(me)` | 获取数据只读指针（离线返回 NULL） |
