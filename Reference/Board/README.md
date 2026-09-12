# 启明欣欣 STM32F407 高配版 V6.1 资料

本目录只保留与当前机械臂控制器直接相关的板级资料，统一使用英文文件名，方便在工程中引用。

## 目录

| 目录 | 内容 |
| --- | --- |
| `Schematic/` | V6.1 原理图。它是核对引脚、接口和板载收发器的首要依据。 |
| `Documentation/` | V6.1 例程手册、下载教程、工程结构说明和原始注意事项。 |
| `MCU_Reference/` | STM32F407ZGT6 数据手册、STM32F4 参考手册和 Cortex-M4 手册。 |
| `Programmer/` | CMSIS-DAP 使用手册和 LAN8720 替换说明。 |
| `Examples/` | 从厂商例程中筛选出的 LED、USART、RS485、CAN 和 LAN8720/UDP 应用源码。 |

## 当前工程核对结果

原理图已确认当前 `MainCode` 使用的关键资源：

| 功能 | 板级连接 |
| --- | --- |
| CAN1 | PD0 = RX，PD1 = TX，板载 TJA1050 |
| CAN2 | PB12 = RX，PB13 = TX，板载 TJA1050 |
| USART6 | PC6 = TX，PC7 = RX；可用于力传感器 RS485，也可运行独立 UART 回环测试 |
| 板载 RS485 方向控制 | PG6，标号 `485_RE` |
| 外接 RS485 方向控制 | `MainCode` 使用 PB0，同时连接 DE 和 `/RE` |
| Ethernet RMII | PA1/PA2/PA7、PC1/PC4/PC5、PG11/PG13/PG14，PHY 为 LAN8720A |
| LED | PE3 = LED0，PE4 = LED1，PG9 = LED2，低电平点亮 |
| HSE | PH0/PH1，8 MHz |

实际程序以 [`MainCode/Config/ChallengeCup_Main.ioc`](../../MainCode/Config/ChallengeCup_Main.ioc)、[`MainCode/docs/HARDWARE_PORT.md`](../../MainCode/docs/HARDWARE_PORT.md) 和 `MainCode` 源码为准；这里的例程只用于查阅和对照，不能直接覆盖主工程。

## 例程说明

`Examples/` 中只保留应用层 `Main/`、`Common/`、`USER/` 和 Keil 工程文件，未复制重复的 HAL/CMSIS 库、启动文件、lwIP 全套源码及 Keil 编译产物。例程不是当前主工程的构建输入；需要完整编译时使用 `MainCode` 自带的 CMake 工程。

筛选保留的例程如下：

- `HAL/01_LED`：LED 极性和基础 GPIO 操作
- `HAL/12_USART2_RS485`：板载 USART2/PG6 RS485 连接方式（仅作冲突参考）
- `HAL/13_USART6_TTL`：PC6/PC7 的 USART6 复用配置
- `HAL/16_CAN1_CAN2`：两路 CAN 引脚和 HAL 初始化示例
- `HAL/25_232_485_CAN_Bridge`：板载串口、RS485 与 CAN 的桥接思路
- `Standard/29_UDP`：LAN8720、RMII 和 UDP 应用示例

导入包中与本项目无关的扩展屏幕、音频、无线模块、云平台、UCOS/FreeRTOS/EMWIN/NES 教材、软件安装包和编译输出均未纳入本目录。
