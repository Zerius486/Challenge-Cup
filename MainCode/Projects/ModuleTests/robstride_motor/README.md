# RobStride 单电机测试工程

本工程只验证 CAN2 上的一台灵足时代 RobStride 电机。默认固件不使能电机，每 200 ms 轮询一个参数，依次读取 position、speed、Iq 和 bus voltage。

协议实现来自灵足时代官方产品资料库和官方 STM32 示例，支持 RS00 到 RS06；电机必须预先配置为私有协议和 1 Mbit/s。

## 接线

- CAN2：PB12 RX、PB13 TX，1 Mbit/s
- CANH/CANL 和信号地对应连接，动力电源独立供电
- 断电核对终端电阻；首次动作保持无负载并准备硬件急停

## 调试变量

- `g_robstride_last_parameter_index/value/raw`：最近参数回复
- `g_robstride_position_rad`、`g_robstride_target_position_rad`、`velocity_rad_s`、`torque_nm`、`temperature_c`、`g_robstride_last_feedback_ms`：最近反馈和一次性动作目标
- `g_robstride_current_limit_a`：所选型号的固件电流上限
- `g_robstride_tx_count`、`g_robstride_rx_count`：收发计数
- `g_robstride_faults/warnings/fault_bits`：故障信息
- `g_robstride_can_error`、`g_robstride_test_failed`：测试错误
- `g_robstride_motion_phase`：一次性动作阶段

LED0 心跳闪烁，LED1 每收到一帧合法回复翻转，LED2 表示 CAN、参数或配置错误。

## 一次性固定角度动作

先确认默认诊断能持续收到正确回复，再编辑 `Inc/test_config.h`：

1. 设置正确的电机 ID 与型号编号。
2. 设置 `ROBSTRIDE_TEST_ROTATION_RAD`（正数为正方向，负数为反方向）和
   `ROBSTRIDE_TEST_PROFILE_SPEED_RAD_S`。默认是相对当前位置转 `0.50 rad`
   （约 28.6 度），速度 `0.50 rad/s`。
3. 将 `ROBSTRIDE_TEST_ENABLE_MOTION` 改为 `1` 后重新构建。

固件只执行一次：读取当前位置，计算目标角度，依次执行
`clear fault/stop -> current mode -> Iq=0 -> enable -> position mode ->
目标角度 -> 到位等待 -> current mode/Iq=0 -> stop`。动作目标是相对启动时
当前位置的固定角度，不是固定的绝对零点。运动前必须收到 250 ms 内、无故障且
`mode_state=2` 的反馈；反馈超时、故障或超出位置范围都会保持零电流并停机。
