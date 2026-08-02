# 板间通信模块 (module_board_comm)

## 1. 模块概述

`module_board_comm` 是云台板与底盘板之间的 Classic CAN 数据协议模块，用于传输 DR16 遥控器数据、云台状态、底盘运动数据、发射机构状态和心跳信息。模块只负责协议编解码与在线状态快照，不决定 DR16 或拨弹盘的实际安装位置。

**核心功能**：

- **发送**：遥控器数据（3 帧分片）、云台数据（2 帧分片）、底盘数据（1 帧）、发射机构数据（1 帧）、心跳（1 帧）
- **接收**：自动路由并组装分片数据，各数据组独立在线超时检测
- **分片组装**：遥控器和云台数据跨多帧传输，接收端按序列号组装，避免不同时刻的帧被错误拼合
- **数据获取**：通过只读指针获取各数据组的最新快照

## 2. 设计边界

| **模块负责**                          | **模块不负责**                           |
| :------------------------------------ | :--------------------------------------- |
| CAN 协议编解码（8 种消息类型）        | 具体遥控器的采集与归一化（由输入模块/App负责） |
| 分片组装（遥控器 3 帧、云台 2 帧）    | 云台/底盘/发射机构的实际控制逻辑         |
| 各数据组独立在线超时检测              | 板卡角色和硬件安装位置的配置             |
| 数据快照管理（staging → committed）   | CAN 硬件初始化和过滤器配置               |
| 浮点数缩放编码（±32.767，精度 0.001） | CAN 总线仲裁和错误恢复                   |

## 3. 消息协议

### 3.1 帧格式

所有消息均为 **8 字节 Classic CAN 数据帧**：

| 字节偏移 | 内容       | 说明                                   |
| :------- | :--------- | :------------------------------------- |
| 0        | `sequence` | 序列号（同一数据组的分片共享相同序号） |
| 1        | `flags`    | 标志位（含义因消息类型而异）           |
| 2-7      | `payload`  | 6 字节负载（编码后的数据）             |

### 3.2 消息类型

从 `base_identifier` 开始连续分配 8 个 CAN ID：

| 消息类型                    | 偏移 | 分片数   | 说明                                          |
| :-------------------------- | :--- | :------- | :-------------------------------------------- |
| `REMOTE_CHANNELS_PRIMARY`   | 0    | 3 帧之一 | 遥控器主通道（channel 0~2）                   |
| `REMOTE_CHANNELS_AUXILIARY` | 1    | 3 帧之一 | 遥控器辅助通道（channel 3 + dial + keyboard） |
| `REMOTE_INPUT`              | 2    | 3 帧之一 | 遥控器输入事件（鼠标 + 开关 + 按键）          |
| `GIMBAL_PRIMARY`            | 3    | 2 帧之一 | 云台主数据（yaw/pitch 角度 + yaw 角速度）     |
| `GIMBAL_AUXILIARY`          | 4    | 2 帧之一 | 云台辅助数据（pitch 角速度）                  |
| `CHASSIS`                   | 5    | 1 帧     | 底盘数据（速度 + 状态）                       |
| `SHOOTER`                   | 6    | 1 帧     | 发射机构数据                                  |
| `HEARTBEAT`                 | 7    | 1 帧     | 心跳（板卡角色 + 运行时间）                   |

### 3.3 数据编码

浮点数使用 **缩放因子 1000** 编码为 int16（范围 ±32.767，精度 0.001）：

```c
encoded = clamp(value * 1000, -32768, 32767)
decoded = encoded / 1000.0
```

## 4. 分片组装机制

### 4.1 为什么要分片

- 遥控器数据（4 通道 + dial + keyboard + 鼠标 + 开关）超过 8 字节
- 云台数据（2 角度 + 2 角速度 + 状态）超过 8 字节
- 必须分多帧传输同一数据组的完整信息

### 4.2 分片策略

**发送端**：

- 同一数据组的所有分片使用**相同的序列号**（`transmit_sequence` 递增）
- 分片按顺序发送

**接收端**：

- 收到分片时，先写入 `staging` 暂存区
- 若序列号与当前组装事务相同，继续累积；否则丢弃旧数据，开始新事务
- 收到全部所需分片后，原子提交到公开数据（`remote_data` / `gimbal_data`）

### 4.3 分片组成

| 数据组   | 所需分片                    | 掩码值             |
| :------- | :-------------------------- | :----------------- |
| 遥控器   | PRIMARY + AUXILIARY + INPUT | 0x07（二进制 111） |
| 云台     | PRIMARY + AUXILIARY         | 0x03（二进制 11）  |
| 底盘     | 单帧                        | 直接提交           |
| 发射机构 | 单帧                        | 直接提交           |

## 5. API 参考

### 5.1 初始化

```c
module_board_comm_status_t module_board_comm_init(
    module_board_comm_t *me,
    const module_board_comm_config_t *config);
```

**配置参数**：

| 参数                  | 说明                                                         |
| :-------------------- | :----------------------------------------------------------- |
| `can`                 | CAN BSP 基类（已初始化）                                     |
| `base_identifier`     | CAN ID 基址（消息从此连续分配，需保证不与其他 CAN 设备冲突） |
| `transmit_timeout_ms` | CAN 发送超时（毫秒）                                         |
| `offline_timeout_ms`  | 各数据组离线超时（毫秒）                                     |

**约束**：`base_identifier + 8 <= 0x7FF`（标准帧范围）。

### 5.2 发送接口

| 函数                               | 说明             | 分片数 |
| :--------------------------------- | :--------------- | :----- |
| `module_board_comm_send_remote`    | 发送遥控器数据   | 3 帧   |
| `module_board_comm_send_gimbal`    | 发送云台数据     | 2 帧   |
| `module_board_comm_send_chassis`   | 发送底盘数据     | 1 帧   |
| `module_board_comm_send_shooter`   | 发送发射机构数据 | 1 帧   |
| `module_board_comm_send_heartbeat` | 发送心跳         | 1 帧   |

所有发送函数同步执行（阻塞直到 CAN 发送完成或超时）。

### 5.3 接收接口

```c
module_board_comm_status_t module_board_comm_handle_frame(
    module_board_comm_t *me,
    const bsp_can_frame_t *frame);
```

- 由 CAN 回调调用（如 `bsp_can_dispatcher`）
- 自动路由到对应消息类型
- 完成分片组装后更新公开数据

### 5.4 数据获取接口

| 函数                            | 返回类型                                   | 说明                   |
| :------------------------------ | :----------------------------------------- | :--------------------- |
| `module_board_comm_get_remote`  | `const module_board_comm_remote_process_data_t *`  | 遥控输入（若在线）     |
| `module_board_comm_get_gimbal`  | `const module_board_comm_gimbal_process_data_t *`  | 云台数据（若在线）     |
| `module_board_comm_get_chassis` | `const module_board_comm_chassis_process_data_t *` | 底盘数据（若在线）     |
| `module_board_comm_get_shooter` | `const module_board_comm_shooter_process_data_t *` | 发射机构数据（若在线） |

### 5.5 在线状态更新

```c
void module_board_comm_update_time(module_board_comm_t *me, uint32_t elapsed_time_ms);
```

- 必须在周期任务中调用（且只能有一个时间所有者）
- 累加各数据组的超时计数，超过 `offline_timeout_ms` 则置离线

## 6. 数据结构

### 6.1 云台数据 (`module_board_comm_gimbal_process_data_t`)

```c
typedef struct {
    float yaw_rad;                  // 偏航角（弧度）
    float pitch_rad;                // 俯仰角（弧度）
    float yaw_velocity_rad_per_s;   // 偏航角速度（rad/s）
    float pitch_velocity_rad_per_s; // 俯仰角速度（rad/s）
    bool imu_valid;                 // IMU 数据是否有效
    bool motors_online;             // 电机是否在线
} module_board_comm_gimbal_process_data_t;
```

### 6.2 底盘数据 (`module_board_comm_chassis_process_data_t`)

```c
typedef struct {
    float velocity_x_m_per_s;        // X 方向速度（m/s）
    float velocity_y_m_per_s;        // Y 方向速度（m/s）
    float angular_velocity_rad_per_s; // 角速度（rad/s）
    bool motors_online;              // 电机是否在线
    bool self_lock_active;           // 自锁是否激活
} module_board_comm_chassis_process_data_t;
```

### 6.3 发射机构数据 (`module_board_comm_shooter_process_data_t`)

```c
typedef struct {
    float friction_velocity_rad_per_s; // 摩擦轮速度（rad/s）
    float feeder_position_rad;         // 拨弹盘位置（弧度）
    uint8_t state;                     // 发射机构状态
    uint8_t jam_retry_count;           // 卡弹重试次数
} module_board_comm_shooter_process_data_t;
```

## 7. 使用示例

### 7.1 初始化（云台板）

```c
static module_board_comm_t s_robot_link;

const module_board_comm_config_t cfg = {
    .can = can_ptr,
    .base_identifier = 0x100,
    .transmit_timeout_ms = 10,
    .offline_timeout_ms = 100,
};

module_board_comm_init(&s_robot_link, &cfg);
```

### 7.2 注册 CAN 接收回调

```c
// 在 CAN 接收回调中处理
void can_rx_callback(const bsp_can_frame_t *frame) {
    if (frame->identifier >= 0x100 && frame->identifier < 0x108) {
        module_board_comm_handle_frame(&s_robot_link, frame);
    }
}
```

### 7.3 发送数据（云台板发送遥控器数据到底盘板）

```c
const module_dr16_process_data_t *dr16_data = module_dr16_get_data(&dr16);
if (dr16_data != NULL && dr16_data->is_online) {
    module_board_comm_remote_process_data_t link_remote = {
        .channel = {
            dr16_data->channel[0], dr16_data->channel[1],
            dr16_data->channel[2], dr16_data->channel[3],
        },
        .left_switch = (module_board_comm_switch_t)dr16_data->left_switch,
        .right_switch = (module_board_comm_switch_t)dr16_data->right_switch,
        .mouse_x = dr16_data->mouse_x,
        .mouse_y = dr16_data->mouse_y,
        .mouse_z = dr16_data->mouse_z,
        .mouse_left_pressed = dr16_data->mouse_left_pressed,
        .mouse_right_pressed = dr16_data->mouse_right_pressed,
        .keyboard = dr16_data->keyboard,
        .dial = dr16_data->dial,
        .is_online = true,
    };
    module_board_comm_send_remote(&s_robot_link, &link_remote);
}

// 发送云台状态
module_board_comm_gimbal_process_data_t gimbal = {
    .yaw_rad = current_yaw,
    .pitch_rad = current_pitch,
    .yaw_velocity_rad_per_s = yaw_vel,
    .pitch_velocity_rad_per_s = pitch_vel,
    .imu_valid = true,
    .motors_online = true,
};
module_board_comm_send_gimbal(&s_robot_link, &gimbal);
```

### 7.4 接收数据（底盘板接收云台板发来的数据）

```c
void control_loop(void) {
    // 1. 更新在线超时
    module_board_comm_update_time(&s_robot_link, dt_ms);

    // 2. 获取遥控器数据
    const module_board_comm_remote_process_data_t *remote =
        module_board_comm_get_remote(&s_robot_link);
    if (remote != NULL) {
        // 板间协议保留原始输入；归一化策略由 App 决定。
        float forward = (float)remote->channel[3] / 660.0F;
        float strafe = (float)remote->channel[2] / 660.0F;
        // ...
    }

    // 3. 获取云台数据
    const module_board_comm_gimbal_process_data_t *gimbal = module_board_comm_get_gimbal(&s_robot_link);
    if (gimbal != NULL) {
        // 使用云台角度数据
    }
}
```

## 8. 标志位定义

### 8.1 遥控器输入帧（`REMOTE_INPUT`）

| 位  | 内容                                   |
| :-- | :------------------------------------- |
| 0-1 | 左开关状态（0=无效，1=上，2=下，3=中） |
| 2-3 | 右开关状态（同上）                     |
| 4   | 鼠标左键按下                           |
| 5   | 鼠标右键按下                           |
| 6   | 保留                                   |
| 7   | 遥控器在线状态                         |

### 8.2 云台主数据帧（`GIMBAL_PRIMARY`）

| 位  | 内容         |
| :-- | :----------- |
| 0   | IMU 数据有效 |
| 1   | 电机在线     |

### 8.3 底盘数据帧（`CHASSIS`）

| 位  | 内容     |
| :-- | :------- |
| 0   | 电机在线 |
| 1   | 自锁激活 |

### 8.4 发射机构数据帧（`SHOOTER`）

无标志位（payload 中包含状态信息）。

## 9. 在线超时机制

- 每个数据组（遥控器、云台、底盘、发射机构）有**独立的**超时计数器
- 每次收到有效帧时，对应计数器的 `elapsed_time_ms` 重置为 0
- `module_board_comm_update_time()` 累加各计数器
- 超过 `offline_timeout_ms` 时，对应数据组置离线
- **数据离线时，调用者应回退到本板本地安全状态，不能继续使用最后控制目标**

## 10. 注意事项

- **序列号回绕**：`transmit_sequence` 为 uint8_t，从 255 回绕到 0 时正常工作
- **分片抢占**：新序列号到达会丢弃未完成的旧事务，避免拼合不同时刻的数据
- **CAN ID 区域**：不同机器人或不同总线的 `base_identifier` 不得重叠
- **发送频率**：关键控制数据优先，限制总线占用率（建议总负载 < 70%）
- **指针生命周期**：`get_*` 返回的指针只在对象生命周期内有效，不可长期持有
- **时间所有者**：`update_time` 只能由一个任务调用，避免重复累加

## 11. 错误码速查

| 状态码             | 触发场景                                |
| :----------------- | :-------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、浮点值非有限、CAN ID 基址越界 |
| `NOT_INITIALIZED`  | 对象未初始化                            |
| `TRANSPORT_ERROR`  | CAN 发送失败                            |
| `INVALID_FRAME`    | 接收帧 ID 越界、类型不匹配              |

## 12. 建议验证测试项

- [ ] 每种消息类型的编解码正确性（原始值 ↔ 编码值 ↔ 解码值）
- [ ] 分片乱序到达时的正确组装
- [ ] 丢帧时旧数据不被错误提交
- [ ] 新序列号抢占丢弃旧事务
- [ ] 序列号回绕（255 → 0）
- [ ] 各数据组独立离线超时（一组离线不影响其他组）
- [ ] CAN ID 基址边界（base_identifier + 8 <= 0x7FF）
- [ ] 发送失败返回 `TRANSPORT_ERROR`
- [ ] 两块板不同发送周期下的数据一致性

---

**总结**：`module_board_comm` 提供云台板和底盘板之间的 Classic CAN 协议，支持多帧分片、独立在线超时和原子数据提交。模块不依赖 DR16；任意遥控器或上位机输入都可由 App 映射到板间遥控结构体。

## 一页式接入顺序与可读信息

```c
/* 1. 两块板先初始化各自 CAN BSP，并约定完全相同的 base_identifier。 */
static module_board_comm_t board_comm;

/* 2. 注入 CAN、ID 基址、发送超时和离线超时。 */
module_board_comm_status_t status =
    module_board_comm_init(&board_comm, &board_comm_config);

/* 3. 将本模块的 ID 范围注册进 bsp_can_dispatcher。 */
/* CAN 路由回调中只调用 module_board_comm_handle_frame(&board_comm, frame)。 */

/* 4. 各板按职责发送 remote/gimbal/chassis/shooter 或 heartbeat。 */
status = module_board_comm_send_remote(&board_comm, remote_data);

/* 5. 周期推进每组数据的独立离线计时。 */
module_board_comm_update_time(&board_comm, elapsed_time_ms);

/* 6. getter 离线时返回 NULL，App 必须切换安全状态。 */
const module_board_comm_chassis_process_data_t *chassis =
    module_board_comm_get_chassis(&board_comm);
```

| 可读取结构体 | Getter | 主要信息 |
| --- | --- | --- |
| `module_board_comm_remote_process_data_t` | `module_board_comm_get_remote()` | 跨板转发的摇杆、开关、鼠标、键盘和拨轮输入 |
| `module_board_comm_gimbal_process_data_t` | `module_board_comm_get_gimbal()` | 云台角度/角速度、IMU 和电机在线状态 |
| `module_board_comm_chassis_process_data_t` | `module_board_comm_get_chassis()` | 车体速度、自锁和电机在线状态 |
| `module_board_comm_shooter_process_data_t` | `module_board_comm_get_shooter()` | 摩擦轮、拨弹盘、发射状态和卡弹次数 |
| `module_board_comm_t` | 调试器只读查看 | 每组在线标志、超时计时、分片掩码和序号 |

所有 getter 都返回内部只读指针；多帧消息只有在全部分片通过校验后才会原子更新正式数据。
