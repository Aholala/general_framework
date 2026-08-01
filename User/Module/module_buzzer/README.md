# 蜂鸣器控制模块 (module_buzzer) —— 完整使用指南

## 1. 模块概述

`module_buzzer` 是基于 `bsp_pwm_t` 的有源/无源蜂鸣器控制模块，支持持续音调、音符序列、静音、循环播放和非阻塞时间推进。适用于设备上电提示、操作反馈、故障告警等场景。

**核心功能**：

- 播放持续单音（无限循环直到停止）
- 播放自定义音符序列（非阻塞，支持循环）
- 立即静音
- 查询播放状态
- 通过 `module_device` 框架统一调度

## 2. 设计边界

| **模块负责**                       | **模块不负责**                     |
| :--------------------------------- | :--------------------------------- |
| PWM 频率和占空比控制（单音、序列） | PWM 硬件初始化和时钟配置           |
| 音符序列的非阻塞时序管理           | 音符数组的存储（由调用者静态分配） |
| 静音和停止                         | 优先级调度（由 App 层管理）        |
| 播放状态查询                       | 音量调节（占空比固定，由配置决定） |

## 3. 对象模型

```text
module_device_t                    (设备基类)
└── module_buzzer_t                (蜂鸣器对象：PWM、序列指针、播放状态)
```

通过 `module_buzzer_as_device`（未显式提供，但可通过 `&me->super` 获取）可接入 `module_device` 框架进行统一调度。

## 4. 核心类型

### 4.1 音符结构 (`module_buzzer_note_t`)

```c
typedef struct {
    uint32_t frequency_hz;   // 频率（Hz），必须 > 0
    uint32_t sound_time_ms;  // 发声持续时间（毫秒），必须 > 0
    uint32_t silence_time_ms;// 发声后的静音持续时间（毫秒），可为 0
} module_buzzer_note_t;
```

### 4.2 配置结构 (`module_buzzer_config_t`)

```c
typedef struct {
    bsp_pwm_t *pwm;               // PWM BSP 基类
    const char *logical_name;     // 设备逻辑名称
    uint32_t registration_key;    // 注册键值
    float duty_cycle;             // PWM 占空比（0.0~1.0），无源蜂鸣器常用 0.5
} module_buzzer_config_t;
```

## 5. API 参考

| 函数                          | 说明                             | 返回值                    |
| :---------------------------- | :------------------------------- | :------------------------ |
| `module_buzzer_init`          | 初始化蜂鸣器模块                 | `OK` / `INVALID_ARGUMENT` |
| `module_buzzer_start`         | 启动蜂鸣器（使能 PWM，默认静音） | `OK` / `TRANSPORT_ERROR`  |
| `module_buzzer_stop`          | 停止蜂鸣器（关闭 PWM）           | `OK` / `TRANSPORT_ERROR`  |
| `module_buzzer_play_tone`     | 播放持续单音                     | `OK` / `INVALID_ARGUMENT` |
| `module_buzzer_play_sequence` | 播放音符序列（非阻塞）           | `OK` / `INVALID_ARGUMENT` |
| `module_buzzer_silence`       | 立即静音                         | `OK` / `NOT_STARTED`      |
| `module_buzzer_update`        | 周期更新（推进播放）             | `OK` / `FINISHED`         |
| `module_buzzer_is_playing`    | 查询是否正在播放                 | `true` / `false`          |

## 6. 使用示例

### 6.1 初始化与启动

```c
static module_buzzer_t s_buzzer;

const module_buzzer_config_t cfg = {
    .pwm = board_buzzer_pwm,      // 已初始化的 PWM 基类
    .logical_name = "buzzer",
    .registration_key = 0,
    .duty_cycle = 0.5F,           // 50% 占空比
};

module_buzzer_init(&s_buzzer, &cfg);
module_buzzer_start(&s_buzzer);
```

### 6.2 播放单音

```c
// 播放 1000Hz 持续音（直到调用 silence）
module_buzzer_play_tone(&s_buzzer, 1000);

// 稍后静音
module_buzzer_silence(&s_buzzer);
```

### 6.3 播放音符序列

```c
static const module_buzzer_note_t startup_notes[] = {
    {.frequency_hz = 1000U, .sound_time_ms = 80U, .silence_time_ms = 40U},
    {.frequency_hz = 1500U, .sound_time_ms = 80U, .silence_time_ms = 40U},
    {.frequency_hz = 2000U, .sound_time_ms = 100U, .silence_time_ms = 0U},
};

// 非循环播放
module_buzzer_play_sequence(&s_buzzer, startup_notes,
                            sizeof(startup_notes)/sizeof(startup_notes[0]),
                            false);
```

### 6.4 周期更新

```c
void main_loop(void) {
    uint32_t dt_ms = get_delta_time_ms();
    module_buzzer_update(&s_buzzer, dt_ms);
    // 其他任务...
}
```

## 7. 播放状态机

```text
         +-------------------+
         |   is_playing=false|
         +---------+---------+
                   | play_tone / play_sequence
                   v
         +-------------------+
         |   is_playing=true |
         |  (应用第一个音符)  |
         +---------+---------+
                   |
                   | module_buzzer_update()
                   v
         +-------------------+
         | 发声阶段 (is_sound_phase=true)
         |  累积时间 < sound_time_ms
         +---------+---------+
                   | 时间到
                   v
         +-------------------+
         | 静音阶段 (is_sound_phase=false)
         |  累积时间 < silence_time_ms
         +---------+---------+
                   | 时间到 或 silence_time_ms=0
                   v
         +-------------------+
         | 切换到下一个音符    |
         | 或循环/结束        |
         +-------------------+
```

## 8. 注意事项

- **音符数组生命周期**：`play_sequence` 不复制音符数组，播放期间必须保持有效（推荐 `static const`）。
- **非阻塞**：所有 API 非阻塞，`update` 使用实际经过时间，不要求固定周期。
- **占空比**：有源蜂鸣器通常 50% 占空比，无源蜂鸣器也常用 50% 以获得较大音量。
- **优先级**：蜂鸣器适合非关键提示，不应阻塞控制任务；更高优先级故障告警应由 App 层管理。

## 9. 错误码速查

| 状态码             | 触发场景                                                   |
| :----------------- | :--------------------------------------------------------- |
| `INVALID_ARGUMENT` | 参数为空、占空比非法、频率为 0、音符发声时间为 0、序列为空 |
| `NOT_INITIALIZED`  | 对象未初始化                                               |
| `NOT_STARTED`      | 未调用 `start`                                             |
| `TRANSPORT_ERROR`  | PWM 设置频率或占空比失败、PWM 启动/停止失败                |
| `FINISHED`         | 非循环序列播放完毕（仅 `update` 返回）                     |

## 10. 建议验证测试项

- [ ] 单音播放和静音
- [ ] 单音序列（多个音符，含静音间隔）
- [ ] 循环播放（`is_looping = true`）
- [ ] 大时间步（一次 `update` 跨越多个相位）
- [ ] 播放中切换序列（先 `silence` 再新序列）
- [ ] 停止后拒绝操作（返回 `NOT_STARTED`）
- [ ] 非法频率（0）、非法占空比
- [ ] 空序列返回错误
- [ ] PWM 平台错误传播

---

**总结**：`module_buzzer` 提供了简洁、非阻塞的蜂鸣器控制，适用于各种提示音和告警场景。通过 `bsp_pwm` 抽象层，与具体 MCU 解耦，可移植到任意平台。配合 `module_device` 框架，可统一接入系统调度，便于管理。

## 一页式接入顺序与可读信息

```c
/* 1. PWM BSP 必须先初始化；音符数组由调用者长期保存。 */
static module_buzzer_t buzzer;
static const module_buzzer_note_t alarm_notes[] = {
    {.frequency_hz = 1000U, .sound_time_ms = 100U, .silence_time_ms = 50U},
};

/* 2. 注入 PWM、占空比、逻辑名称等配置。 */
module_buzzer_status_t status = module_buzzer_init(&buzzer, &buzzer_config);

/* 3. 启动后才能播放。 */
status = module_buzzer_start(&buzzer);

/* 4. 选择单音或序列；序列播放期间 alarm_notes 不能失效。 */
status = module_buzzer_play_sequence(&buzzer, alarm_notes, 1U, false);

/* 5. 在任务中周期推进发声/停顿状态机。 */
status = module_buzzer_update(&buzzer, elapsed_time_ms);

/* 6. 停止播放可用 silence；关闭模块使用 stop。 */
```

| 可读取信息 | 推荐读取方式 | 说明 |
| --- | --- | --- |
| 是否正在播放 | `module_buzzer_is_playing()` | 单音或音符序列是否仍在运行 |
| 音符定义 | `module_buzzer_note_t` | 频率、持续时间和停顿时间 |
| `module_buzzer_t` | 调试器只读查看 | 当前音符索引、阶段累计时间、循环标志和启动状态 |

播放状态由 `module_buzzer_update()` 推进；如果不周期调用，序列不会自动切换音符。
