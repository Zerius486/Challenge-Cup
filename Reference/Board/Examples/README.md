# 板级例程摘录

这里的文件是从启明欣欣 V6.1 厂商资料中筛选出的应用层源码，仅用于核对板级外设和借鉴写法。它们不是当前 `MainCode` 的构建输入，也没有复制重复的 HAL/CMSIS/lwIP 库。

| 目录 | 用途 |
| --- | --- |
| `HAL/01_LED` | LED GPIO 极性与基础初始化 |
| `HAL/12_USART2_RS485` | 板载 USART2 + PG6 RS485 方向控制；用于理解 PA2/MDIO 冲突 |
| `HAL/13_USART6_TTL` | PC6/PC7 的 USART6 复用和中断示例 |
| `HAL/16_CAN1_CAN2` | CAN1/CAN2 引脚、过滤器和收发示例 |
| `HAL/25_232_485_CAN_Bridge` | 串口、RS485、CAN 桥接思路 |
| `Standard/29_UDP` | LAN8720 RMII 和 UDP 应用示例 |

每个例程保留 `Main/`、`Common/`、`USER/` 和 Keil 工程文件；编译中间文件、重复库和启动文件已排除。实际引脚和通信参数以 `Reference/Board/Schematic/` 及 `MainCode` 为准。
