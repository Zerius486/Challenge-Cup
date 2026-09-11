# 硬件移植与接线

## 目标板

- 开发板：启明欣欣 STM32F407 高配版 V6.1
- MCU：STM32F407ZGTx，LQFP144
- MCU HSE：8 MHz 晶振
- Ethernet PHY：LAN8720A 独立 25 MHz 晶振，输出 50 MHz RMII REF_CLK
- 系统时钟：168 MHz
- 原理图来源：工作区 `../../Reference/Board/`（当前交付包未附入原理图文件；补入商家资料时请保持该目录名）

## 引脚表

| 功能 | MCU 引脚 | 说明 |
| --- | --- | --- |
| ETH RMII REF_CLK | PA1 | LAN8720A 50 MHz 参考时钟 |
| ETH MDIO | PA2 | 与板载 USART2_TX 冲突，启用以太网后不可再使用该板载 485 发送脚 |
| ETH CRS_DV | PA7 | RMII |
| ETH MDC | PC1 | RMII 管理接口 |
| ETH RXD0/RXD1 | PC4/PC5 | RMII |
| ETH TX_EN | PG11 | RMII |
| ETH TXD0/TXD1 | PG13/PG14 | RMII |
| ETH PHY RESET | PE2 | 低有效，启动时拉低 2 ms 后释放并等待 50 ms |
| CAN1 RX/TX | PD0/PD1 | 板载 TJA1050；默认分配给 eRob |
| CAN2 RX/TX | PB12/PB13 | 板载 TJA1050；默认分配给 RobStride |
| Force UART TX/RX | PC6/PC7 | USART6，460800 8N1；同时是板上 DCMI_D0/D1，不能再启用摄像头接口 |
| Force RS485 DE/RE | PB0 | 高电平发送、低电平接收，接外置收发器的 DE 与 `/RE`；同时是 `T_CS`，不要插用触摸接口的屏 |
| LED0/LED1/LED2 | PE3/PE4/PG9 | 低电平点亮：心跳/网络/故障 |
| SWDIO/SWCLK | PA13/PA14 | 下载与调试 |

## 为什么力传感器不用板载 USART2 RS485

高配板板载 RS485 使用 USART2 的 PA2/PA3，方向脚为 PG6；但 PA2 同时是 LAN8720A 的 RMII MDIO。以太网与板载 USART2 RS485 无法同时可靠工作。因此本工程选择 USART6 PC6/PC7，加一个 3.3 V 逻辑兼容的外置半双工 RS485 收发器，方向脚为 PB0。

结论是：**UDP 与力传感器协议本身不冲突，冲突的是板载 USART2_TX 与 Ethernet MDIO 对 PA2 的复用。** 按本工程的 USART6 + PB0 + 外置 RS485 方案接线时，Ethernet 使用 RMII 和 MAC 自带 DMA，力传感器使用 USART6/DMA2 Stream1，GPIO、外设和 DMA 资源均不重叠，可以在整机固件中同时工作。主循环还限制了每轮 Ethernet 收包数和 CAN 镜像发送数，USART6 使用 4096 字节循环 DMA，避免网络突发长期饿死串口解析；这仍需用 1 kHz 连续数据和满速 UDP 在实物板上做压力验收。

推荐接线：

| STM32/收发器 | 连接 |
| --- | --- |
| PC6 USART6_TX | 收发器 DI |
| PC7 USART6_RX | 收发器 RO |
| PB0 | DE 与 `/RE` 并联 |
| 3.3 V/GND | 收发器逻辑电源/公共地 |
| A/B | 传感器 A/B，若无数据先核对厂商 A/B 命名而不是盲目交换带电线路 |
| 传感器电源 | 独立 9–24 V，额定能力不低于手册要求；与控制器共地 |

不要将 5 V TTL 输出直接接 STM32 输入。总线较长或环境干扰大时使用隔离式 RS485，并按拓扑在总线末端配置终端和偏置。

如果整机还要使用 DCMI 摄像头或触摸屏，必须先把 USART6 或 DE/RE 重映射到另一组已引出的空闲 GPIO，并同步修改 BSP；仅修改接线而不修改复用配置不可行。

## CAN 电气要求

- 两条 CAN 总线相互独立，均配置为 1 Mbit/s：APB1=42 MHz，Prescaler=3，BS1=9 TQ，BS2=4 TQ，SJW=1 TQ，总计 14 TQ，采样点约 71.4%。
- 开发板原理图包含 TJA1050 和 120 欧姆终端。接线前用断电电阻测量确认整条总线等效终端，避免板端与电机端同时重复接入过多 120 欧姆电阻。
- CANH、CANL 必须成对布线，并连接控制器与电机的信号地。电机动力电源不得由开发板供给。
- eRob 接 CAN1，RobStride 接 CAN2。更改分配需要同时修改 `app_config.h`、接线和验收记录。

## 与 UWV 样例的主要差异

| 项目 | UWV 样例 | 本移植 |
| --- | --- | --- |
| 板级 GPIO | 样例板 PF6/PF7/PF8 等 | 启明欣欣 PE3/PE4/PG9、PE2 |
| CAN | 原应用主要使用 CAN2 | CAN1=eRob，CAN2=RobStride |
| RS485 | 未形成与 RMII 冲突隔离方案 | USART6 + 外置收发器 |
| 调度 | FreeRTOS 多任务 | lwIP `NO_SYS=1` + 非阻塞主循环/中断/DMA |
| UDP | 原始 ASCII 和裸 CAN 桥接 | CRC32 二进制主协议，保留受限兼容入口 |

## 上板验收记录

逐项记录实际结果，不应只凭编译成功判定移植完成：

- [ ] 3.3 V、MCU HSE 8 MHz、SYSCLK 168 MHz、PHY 25/50 MHz 时钟正常
- [ ] SWD 下载、复位和三个 LED 状态正常
- [ ] LAN8720A 自动协商、ARP、ICMP/UDP 双向正常
- [ ] CAN1/CAN2 空闲电平、1 Mbit/s、ACK、错误计数正常
- [ ] 台架安全构建（`APP_REMOTE_MOTION_ALLOWED=0`）拒绝所有危险运动命令；提交版按已完成验证开启运动
- [ ] eRob 节点 ID、SDO 应答、状态字和独立急停正常
- [ ] RobStride 型号、ID、协议模式、反馈与独立急停正常
- [ ] RS485 方向切换、460800 8N1、单帧与 1 kHz 连续帧正常，并验证人为断线/错帧后 DMA 能恢复
- [ ] 断网/停止发送命令后，各已跟踪电机在设定超时内停机
- [ ] 机械限位、供电限流、温升和急停在带载前验证
