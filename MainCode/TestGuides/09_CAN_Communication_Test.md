# CAN 通讯测试

烧录 `output/standalone_tests/can_communication/CAN_Communication_Test.hex`。默认同时测试 CAN1（PD0/PD1）和 CAN2（PB12/PB13），两路均为 1 Mbit/s。每一路连接 USB-CAN 或其他 CAN 节点，并连接 CANH、CANL、信号地；总线两端保留 120 Ω 终端。

通过标准：固件每 100 ms 发送标准帧，CAN1 默认 ID `0x321`、CAN2 默认 ID `0x322`；对端回发任意帧后，`g_can_rx_count[0/1]` 持续增加，LED1 翻转，`g_can_error[0/1]` 保持 0。没有 CAN 对端时可能因缺少 ACK 产生错误，不应据此判定硬件已通过。只测一路时在 `can_communication/Inc/test_config.h` 将 `CAN_TEST_BUS_MASK` 改为 1 或 2。
