# Reference 参考资料

本目录只保存硬件厂商手册、协议文件和 UWV 借鉴例程，不参与 `MainCode` 编译。这里的文件只作为查阅和参数核对依据，实际引脚、时钟和程序行为以 `MainCode` 为准。

| 目录 | 内容 |
| --- | --- |
| `Motor/eRob` | 零差云控 eRob CANopen/EtherCAT 用户手册 |
| `Motor/RobStride` | 灵足时代 RobStride 产品资料（当前仅保留目录说明） |
| `ForceSensor` | 六维力传感器 RS485 协议 |
| `Chassis` | 履带底盘与 NUC UDP 通讯协议 |
| `UWV_Example` | UWV 例程压缩包及解压后的借鉴代码 |
| `Board` | 启明欣欣 STM32F407 V6.1 原理图、板级手册和筛选后的外设例程 |

参考资料中的代码只用于查阅和移植，实际引脚、时钟和通信参数以 `MainCode/Config/ChallengeCup_Main.ioc` 与 `MainCode/docs/` 为准。
