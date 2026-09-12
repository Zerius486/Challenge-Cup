# CAN 通讯独立测试工程

本工程只验证开发板两路 CAN 控制器、收发器、终端电阻和中断收发，不解析 eRob 或 RobStride 私有协议。默认同时测试 CAN1 和 CAN2，波特率为 1 Mbit/s：CAN1 使用 PD0/PD1，CAN2 使用 PB12/PB13。

## 接线和运行

每一路都需要一个正常工作的 CAN 对端（USB-CAN、CAN 分析仪或另一块控制器），并连接 CANH、CANL 和信号地。总线两端各保留 120 Ω 终端，电机动力电源不必接入；不要把 CANH/CANL 直接接到 MCU GPIO。若只测试一路，在 `Inc/test_config.h` 将 `CAN_TEST_BUS_MASK` 改为 `1`（CAN1）或 `2`（CAN2）后重新构建。

固件每 100 ms 在已选择的总线上发送一帧 8 字节标准帧：CAN1 默认 ID 为 `0x321`，CAN2 默认 ID 为 `0x322`，数据头为 `CA 4E bus sequence`。对端回发任意 CAN 帧即可验证接收中断；LED1 在收到帧时翻转，LED0 是心跳，LED2 在发送失败或 CAN 错误时点亮。

## 调试变量和通过标准

- `g_can_tx_count[0/1]`、`g_can_rx_count[0/1]`：CAN1/CAN2 收发计数；
- `g_can_last_id[0/1]`、`g_can_last_dlc[0/1]`、`g_can_last_data[0/1]`：最近收到的帧；
- `g_can_last_rx_ms[0/1]`、`g_can_rx_valid[0/1]`：最近接收时间和有效标志；
- `g_can_error[0/1]`：对应控制器最近一次 HAL 错误。

连续运行 10 s 后，已选择总线的发送计数应约为 100 次/秒；使用 CAN 分析仪回发后接收计数应持续增加，帧 ID/DLC/数据可见且 LED1 翻转。`g_can_error` 应保持 0。无对端时发送可能因缺少 ACK 进入错误状态，这不代表固件或引脚接线通过。

构建产物为 `output/standalone_tests/can_communication/CAN_Communication_Test.hex`，构建和烧录命令见上级 [模块测试说明](../README.md)。
