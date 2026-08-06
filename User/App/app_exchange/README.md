# app_exchange — 模块间数据交换

共享内存发布/订阅。零拷贝，无锁（单生产者单消费者）。

## 用法

```c
app_exchange_init();  // 清零所有通道

// 发布
app_chassis_command_t cmd = { .velocity_x_m_per_s = 1.0f, ... };
app_exchange_publish_chassis_command(&cmd);

// 读取
app_chassis_command_t cmd;
app_exchange_read_chassis_command(&cmd);

// 所有通道
// chassis/gimbal/shooter command/feedback
// imu snapshot, vision target
```
