# CubeMX 配置

`ChallengeCup_Main.ioc` 是主控板的可导入 CubeMX 配置记录，包含：

- STM32F407ZGTx、8 MHz HSE、168 MHz 系统时钟；
- CAN1/2（PD0/PD1、PB12/PB13）；
- USART6 PC6/PC7 和 DMA2 Stream1 Channel5；
- LAN8720A RMII、PHY 复位脚和三个 LED；
- 静态 IPv4 与 `NO_SYS=1` 的 lwIP 选项。

当前环境中的 STM32CubeCLT 不带 CubeMX 图形/生成器，实际编译使用手写的
`Core/Src/board.c` 和 `Core/Src/main.c`。如果在其他电脑上用 CubeMX 重新生成，
请先复制整个 `MainCode`，再逐文件比较生成结果；不要直接覆盖已经验证过的板级代码。
