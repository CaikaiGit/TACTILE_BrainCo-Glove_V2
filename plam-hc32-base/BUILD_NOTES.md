# 强脑手套右手工程说明

硬件权威来源为 `小华项目/强脑手套/O02.eprj2`。

## 工程与芯片

- 主工程：`MDK/usart_uart_dma.uvprojx`
- GCC 工程配置：`GCC/.cproject`
- MCU：`HC32F460JEUA-QFN48TR`
- 链接配置：`HC32F460xC`
- 时钟：内部 `HRC -> PLL`

## 当前采样实现

- 右手默认：`STRONG_GLOVE_HAND_RIGHT=1`
- 五指共用 `AS_S0/AS_S1/AS_S2`；X 对应列、Y 对应行
- 硬件按手指→Y→有效X扫描，PGA 每根手指只切换一次
- 无效交叉点不切换 X、不延时、不采 ADC
- 有效点切换 X 后不再附加软件稳定延时；ADC 采样时间仍保持 `0x20`
- 五指 EN 均为低有效并常态使能
- 有效点来自 PCB 的 X/Y 焊盘网络交叉，不从图片猜测
- 强脑手套没有掌部传感器，实时上传只发送五个手指包

详细引脚见 `PINMAP_2026-04-07.md`，点位和区域见 `RIGHT_HAND_POINTMAP.md`。

## 串口上传

- 外层协议沿用小米手套：`<< + channel/flags/length + payload + CRC16 + >>`
- channel 为 `0x02`
- USART1 波特率为 `2000000`
- 每个采样点为 2 字节小端序，保留 ADC 原始值
- 每个手指独立上传
- payload 采用小米手套的多区域结构
- 拇指当前为 2 个区域，其他四指为 3 个区域
- 每个区域只上传固定点位表中的有效点，不增加逐点坐标或位图
- 右手食指去掉 `(X3,Y3)`，五指合计350点；整手五包共797字节
- 200 Hz 实时通道旁路640格双边滤波，直接上传零点扣除后的PGA/ADC值

## 主要源码

- `source/main.c`
- `application/sampler.c`
- `application/system.c`
- `application/serial.c`
- `application/process.c`
- `application/configuration.c`
- `application/force_lut_calculator.c`
- `application/force_lut_storage.c`

## 本地验证

- 右手实板、PGA 4倍、2 Mbps、350点：J-Link完成发送计数实测 `223.330 Hz`
- 串口吞吐约 `178563 B/s`，与 `223.33 × 797` 字节/秒吻合

右手固件输出在 `build_right_hand_verify/`：

- `strong_glove_right.elf`
- `strong_glove_right.hex`
- `strong_glove_right.map`

使用 ARM GNU GCC 编译链接。`nosys` 的 `_read/_write/_close/_lseek` 提示以及链接脚本的 RWX LOAD 段提示是现有裸机工程警告。
