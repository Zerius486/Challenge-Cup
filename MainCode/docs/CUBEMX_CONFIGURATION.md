# CubeMX 配置

当前配置文件是 [`Config/ChallengeCup_Main.ioc`](../Config/ChallengeCup_Main.ioc)。它记录
本板的 MCU、时钟、引脚、CAN、USART6/DMA、RMII 和 lwIP 选项，便于在装有
STM32CubeMX 的电脑上查看或重新生成初始化代码。现有 CMake 构建不依赖 CubeMX；
本机只有 STM32CubeCLT（没有 CubeMX GUI），因此 `Core/Src/board.c` 和
`Core/Src/main.c` 仍是实际编译入口。

如果需要重新生成，请在独立副本中打开 `.ioc`，选择 CMake 工具链，生成后逐文件
比较 `Core/`、`App/` 和 `Middlewares/`，不要直接覆盖当前可用版本。原 UWV 的不完整
配置仅作为历史参考：`../../Reference/UWV_Example/Extracted/UWV_0926 20241107/SEU_EthToCan.ioc`。

## 时钟

| 参数 | 值 |
| --- | --- |
| MCU | STM32F407ZGTx, LQFP144 |
| HSE | Crystal/Ceramic Resonator, 8 MHz |
| PLLM / PLLN / PLLP / PLLQ | 8 / 336 / 2 / 7 |
| SYSCLK / HCLK | 168 / 168 MHz |
| APB1 / APB2 | 42 / 84 MHz |
| Flash latency | 5 WS |

## CAN1 与 CAN2

两路参数相同：Normal mode，Prescaler 3，SJW 1 TQ，BS1 9 TQ，BS2 4 TQ，Automatic Bus-Off enabled，Automatic Retransmission enabled。所有标准帧和扩展帧进入 FIFO0；CAN1 使用 filter bank 0，CAN2 从 bank 14 开始。

- CAN1：PD0 RX、PD1 TX，RX0 和 SCE 中断优先级 5
- CAN2：PB12 RX、PB13 TX，RX0 和 SCE 中断优先级 5

## USART6 与 DMA

- Asynchronous，460800 baud，8 data bits，1 stop bit，no parity，无流控
- PC6 TX、PC7 RX，AF8
- USART6_RX：DMA2 Stream1 Channel5，Peripheral-to-memory，byte/byte，memory increment，circular，high priority，FIFO disabled
- USART6 global interrupt：抢占优先级 6；用于捕获 ORE/FE/NE 并在 HAL 中止 DMA 后请求主循环恢复
- PB0：GPIO output push-pull，初始低，作为 RS485 DE/RE

## Ethernet

- RMII，LAN8720A，PHY address 0，自动协商
- LAN8720A 使用板上独立 25 MHz 晶振，并向 PA1 提供 50 MHz RMII REF_CLK；它不是 MCU HSE
- PA1 REF_CLK、PA2 MDIO、PA7 CRS_DV
- PC1 MDC、PC4 RXD0、PC5 RXD1
- PG11 TX_EN、PG13 TXD0、PG14 TXD1
- PE2：PHY reset，GPIO output，低有效

## 其他 GPIO 与中间件

- PE3、PE4、PG9：GPIO push-pull output，初始高，低电平点亮
- PA13/PA14：Serial Wire
- lwIP：IPv4、UDP、无 DHCP、`NO_SYS=1`
- 静态地址：`192.168.1.10/24`，网关 `192.168.1.1`
- 不启用 FreeRTOS；主循环调用 `ethernetif_input()` 与 `sys_check_timeouts()`

重新生成工程前应先创建独立分支或副本，并逐文件比较。CubeMX 模板版本、Ethernet HAL 和 lwIP glue code 的变化可能破坏当前已经编译验证的零拷贝接收路径。
