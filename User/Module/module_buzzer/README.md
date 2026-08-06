# module_buzzer — 蜂鸣器

基于 `bsp_pwm_t` 的非阻塞蜂鸣器。支持单音、音符序列和循环播放。

## 关键结构体

| 结构体 | 用途 | 关键字段 |
|--------|------|---------|
| `module_buzzer_t` | 蜂鸣器对象 | `pwm`, `notes[]`, `note_count`, `is_playing`, `is_looping` |
| `module_buzzer_config_t` | 配置 | `pwm`, `duty_cycle`(0~1), `logical_name`, `registration_key` |
| `module_buzzer_note_t` | 音符 | `frequency_hz`, `sound_time_ms`, `silence_time_ms` |

## 用法

```c
module_buzzer_t buzzer;
module_buzzer_config_t cfg = {
    .pwm = board_config_get_pwm(BOARD_CONFIG_PWM_BUZZER),
    .duty_cycle = 0.5f,  // 50% 占空比
};
module_buzzer_init(&buzzer, &cfg);
module_buzzer_start(&buzzer);

// 持续单音
module_buzzer_play_tone(&buzzer, 440);  // A4

// 音符序列（非阻塞）
static const module_buzzer_note_t melody[] = {
    {440, 200, 50},  // A4 200ms + 50ms 静音
    {494, 200, 50},  // B4
    {523, 400, 0},   // C5 400ms 无间隔
};
module_buzzer_play_sequence(&buzzer, melody, 3, true);  // 循环

// 周期更新（在任务中调用）
module_buzzer_status_t rc = module_buzzer_update(&buzzer, elapsed_ms);
if (rc == MODULE_BUZZER_STATUS_FINISHED) { /* 非循环序列播放完毕 */ }

// 停止
module_buzzer_silence(&buzzer);
module_buzzer_stop(&buzzer);
```

## 状态码

| 值 | 含义 |
|----|------|
| `OK` | 正在播放 |
| `FINISHED` | 非循环序列播放完毕 |
| `INVALID_ARGUMENT` | 参数非法 |
| `NOT_INITIALIZED` | 未初始化 |
| `NOT_STARTED` | 未 start |
| `TRANSPORT_ERROR` | PWM 操作失败 |

## API 速查

| 函数 | 功能 |
|------|------|
| `module_buzzer_init(me, cfg)` | 初始化 |
| `module_buzzer_start(me)` | 使能 PWM 输出 |
| `module_buzzer_stop(me)` | 关闭 PWM |
| `module_buzzer_play_tone(me, hz)` | 播放持续单音 |
| `module_buzzer_play_sequence(me, notes, n, loop)` | 播放音符序列 |
| `module_buzzer_silence(me)` | 立即静音 |
| `module_buzzer_update(me, ms)` | 周期推进（必须周期性调用） |
| `module_buzzer_is_playing(me)` | 查询是否在播放 |
