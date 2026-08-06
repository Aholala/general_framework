# App — 业务应用层

装配所有 Module + Algorithm + BSP，编排机器人行为。

## 模块列表

| 模块 | 职责 |
|------|------|
| `app_robot` | 顶层装配：DR16、板间通信、遥控命令 |
| `app_command` | 遥控器→云台/底盘/发射机构命令映射 |
| `app_gimbal` | 云台控制（PID/LQR、IMU/编码器反馈） |
| `app_chassis` | 底盘运动（跟随/旋转/无力模式） |
| `app_shooter` | 发射机构状态机 + 火控 |
| `app_imu` | IMU 读数 + 姿态估计 + 坐标系变换 |
| `app_vision` | USB 视觉通信：mode/ID 协议 |
| `app_safety` | 安全监控（看门狗、遥控失联、电机健康） |
| `app_exchange` | 模块间数据交换（共享内存，零拷贝） |

## 初始化

所有 `app_*_init()` 返回 `bsp_status_t`（不再返回 `bool`）：

```c
if (app_robot_init() != BSP_STATUS_OK) Error_Handler();
```

错误信息通过全局寄存器 `bsp_error_read()` 获取。

## 依赖方向

```
Task（FreeRTOS 入口）→ App（本层）→ Module + Algorithm + BSP
```

App 不依赖 Task，Task 只转发到 `app_*_update()`。
