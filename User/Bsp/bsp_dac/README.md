# bsp_dac

通用 DAC 通道抽象，支持原始码、归一化值、电压输出以及 DMA 波形播放。

## 配置

`bsp_dac_config_t` 指定平台句柄、通道、分辨率、参考电压和事件回调。构造时计算最大原始
码。平台端负责输出缓冲、触发源、引脚模拟模式和实际参考电压。

## 静态输出

```c
bsp_dac_init(&dac_device, &config);
bsp_dac_start(bsp_dac_as_base(&dac_device));
bsp_dac_set_voltage(bsp_dac_as_base(&dac_device), 1.65F);
```

提供三种表示：

- `set_raw` / `get_raw`：硬件码；
- `set_normalized`：`0.0F`～`1.0F`；
- `set_voltage`：`0`～`reference_voltage_v`。

超出范围返回错误，不应静默绕回。

## DMA 波形

`bsp_dac_start_dma` 使用调用者提供的 `const uint32_t` 样本数组。平台端决定单次、循环及
触发频率；这些行为必须在平台配置中明确。异步传输结束前不得修改或释放样本。

完成、停止或错误通过 `bsp_dac_notify` 报告。回调不应在 ISR 中重新生成大量波形数据。

## 安全状态

停止 DAC 后输出可能保持最后电压、高阻或被硬件复位。执行器控制场景必须由板级平台明确
安全电压，并在故障路径主动设置安全值后停止。

## 共享资源

多个 DAC 通道可能共享定时触发器和 DMA。通用对象只描述单通道，平台端必须处理资源
冲突并返回 `BUSY` 或 `NO_RESOURCE`。

## 建议验证

- 零码、半量程和满量程；
- 电压/归一化换算；
- 非法分辨率和参考电压；
- DMA 单次和循环模式；
- 停止后的安全输出；
- 多通道资源冲突；
- 未初始化、超范围和平台错误。
