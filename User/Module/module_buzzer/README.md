# module_buzzer

基于 `bsp_pwm_t` 的有源/无源蜂鸣器控制模块，支持持续音调、音符序列、静音、循环播放和
非阻塞时间推进。

## 配置

`module_buzzer_config_t` 包含 PWM 基类、逻辑名称、注册键和发声占空比。PWM 平台必须支持
运行时频率与占空比修改。`duty_cycle` 应位于 `[0, 1]`，无源蜂鸣器常用 50%。

## 音符序列

`module_buzzer_note_t` 定义频率、发声时间和发声后的静音时间：

```c
static const module_buzzer_note_t startup_notes[] = {
    {.frequency_hz = 1000U, .sound_time_ms = 80U, .silence_time_ms = 40U},
    {.frequency_hz = 1500U, .sound_time_ms = 80U, .silence_time_ms = 0U},
};

module_buzzer_play_sequence(
    &buzzer, startup_notes,
    sizeof(startup_notes) / sizeof(startup_notes[0]),
    false);
```

音符数组不会被复制，必须在播放期间有效，推荐 `static const`。

## 更新

周期调用 `module_buzzer_update(me, elapsed_time_ms)` 推进发声/静音相位。接口使用实际经过
时间，不要求固定周期。非循环序列完成后返回 `MODULE_BUZZER_STATUS_FINISHED`。

## 控制

- `play_tone`：持续播放单一频率；
- `play_sequence`：播放序列；
- `silence`：立即关闭声音并清除播放状态；
- `is_playing`：查询当前是否在播放。

`stop` 会将 PWM 置于安全静音状态。

## 赛场使用

蜂鸣器适合上电自检、校准完成、遥控离线和故障编码。不要在控制任务中阻塞延时播放，也不
应让普通提示音覆盖更高优先级故障告警；优先级调度可在 App 层实现。

## 建议验证

- 单音、零静音间隔和多音序列；
- 循环与非循环；
- 大时间步跨越多个相位；
- 播放中切换序列；
- 静音和停止；
- 非法频率、占空比和空序列；
- PWM 平台错误传播。
