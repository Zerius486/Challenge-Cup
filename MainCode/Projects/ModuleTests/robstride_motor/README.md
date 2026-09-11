# RobStride 单电机测试工程

本工程只验证 CAN2 上的一台灵足时代 RobStride 电机。默认固件不使能电机，每 200 ms 轮询一个参数，依次读取 position、speed、Iq 和 bus voltage。

协议实现来自灵足时代官方产品资料库和官方 STM32 示例，支持 RS00 到 RS06；电机必须预先配置为私有协议和 1 Mbit/s。

## 接线

- CAN2：PB12 RX、PB13 TX，1 Mbit/s
- CANH/CANL 和信号地对应连接，动力电源独立供电
- 断电核对终端电阻；首次动作保持无负载并准备硬件急停

## 调试变量

- `g_robstride_last_parameter_index/value/raw`：最近参数回复
- `g_robstride_position_rad`、`velocity_rad_s`、`torque_nm`、`temperature_c`、`g_robstride_last_feedback_ms`：最近反馈
- `g_robstride_current_limit_a`：所选型号的固件电流上限
- `g_robstride_tx_count`、`g_robstride_rx_count`：收发计数
- `g_robstride_faults/warnings/fault_bits`：故障信息
- `g_robstride_can_error`、`g_robstride_test_failed`：测试错误
- `g_robstride_motion_phase`：一次性动作阶段

LED0 心跳闪烁，LED1 每收到一帧合法回复翻转，LED2 表示 CAN、参数或配置错误。

## 一次性小电流动作

先确认默认诊断能持续收到正确回复，再编辑 `Inc/test_config.h`：

1. 设置正确的电机 ID 与型号编号。
2. 从很小的 `ROBSTRIDE_TEST_CURRENT_A` 开始；单位是 A，不是 N·m。
3. 将 `ROBSTRIDE_TEST_ENABLE_MOTION` 改为 `1` 后重新构建。

固件只执行一次 `clear fault/stop -> current mode -> Iq=0 -> enable -> 小电流 500 ms -> Iq=0 -> stop`。施加电流前必须收到 250 ms 内、无故障且 `mode_state=2` 的反馈。输入还会与所选型号的 `iq_ref` 上限比较；任一条件失败都会保持零电流并停机。
