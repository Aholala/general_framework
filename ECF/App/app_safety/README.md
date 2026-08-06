# app_safety — 安全监控

看门狗喂狗 + 遥控失联检测 + 电机健康汇总。

## 用法

```c
app_safety_init(NULL);  // NULL = 不用看门狗

// 周期更新
app_safety_update(dt);
// 自动检查：遥控在线、电机健康、CAN 通信
// 失联 → 自动无力模式
```
