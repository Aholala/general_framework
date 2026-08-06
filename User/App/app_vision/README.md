# app_vision — 视觉通信

USB CDC 协议：接收 mode/ID，发布视觉目标。控制自瞄开关。

## 用法

```c
app_vision_config_t cfg = {
    .usb_vcp = board_config_get_usb_vcp(),
    .target_timeout_ms = 200,
    .transmit_period_ms = 10,
};
app_vision_init(&cfg);

// 周期更新
app_vision_update(dt);
// 接收 USB 帧 → 解析 mode/ID → 发布到 app_exchange
```
