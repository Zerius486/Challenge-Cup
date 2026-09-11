# eRob 单电机测试工程

本工程只验证 CAN1 上的零差云控 eRob。默认固件不会使能或移动电机，只以 100 ms 周期读取 CANopen `0x6041` 状态字。

## 接线

- CAN1：PD0 RX、PD1 TX，1 Mbit/s
- 电机 CANH/CANL 与开发板 CAN1 对应连接，并连接信号地
- 断电测量总线两端等效终端；电机动力电源独立供电
- 准备硬件急停，首次测试保持无负载

## 调试变量

烧录后可在 STM32CubeIDE Live Expressions 或 GDB 中观察：

- `g_erob_status_valid`：收到合法状态字后为 1
- `g_erob_statusword`、`g_erob_ds402_state`、`g_erob_last_status_ms`：CiA 402 状态及时间
- `g_erob_tx_count`、`g_erob_rx_count`：收发计数
- `g_erob_last_abort_code`：最近一次 SDO abort code
- `g_erob_can_error`、`g_erob_test_failed`：错误状态
- `g_erob_motion_phase`：一次性动作阶段

LED0 心跳闪烁，LED1 每收到一帧合法 eRob SDO 应答翻转，LED2 表示 CAN/SDO 错误。

## 一次性位置动作

先在默认模式确认状态字、节点 ID、总线波特率和急停均正确。然后编辑 `Inc/test_config.h`：

1. 设置准确的 `EROB_TEST_NODE_ID`。
2. 根据具体 eRob 编码器和机械零点设置绝对位置 `EROB_TEST_TARGET_POSITION_PLUS` 与轮廓速度。
3. 将 `EROB_TEST_ENABLE_MOTION` 改为 `1` 后重新构建。

固件只执行一次 `NMT start -> 6 -> PP 模式/轮廓速度/目标位置 -> 7 -> 15 -> 等待 500 ms 及 OPERATION_ENABLED -> 15 -> 31 -> quick stop`，不会循环动作。触发目标前必须收到 250 ms 内的 `OPERATION_ENABLED` 状态字，否则直接快速停机。位置单位是编码器 plus，不是角度。
