# VOFA+ FireWater 回传

主工程通过 USART3 的 PB10 输出 FireWater CSV，参数为 460800 baud、8N1、无流控。PB11 只保留为 USART3_RX 复用脚，不需要接线；USB-TTL 使用 3.3 V 电平，开发板 PB10 接 USB-TTL RX，开发板 GND 接 USB-TTL GND。

输出周期为 100 ms（约 10 Hz），每行 39 列，以 `CRLF` 结束。VOFA+ 选择 FireWater 协议即可直接显示曲线。列顺序如下：

```text
0 time_ms, 1 frame_count, 2 telemetry_flags, 3 force_timestamp_ms,
4 Fx, 5 Fy, 6 Fz, 7 Mx, 8 My, 9 Mz,
10 erob_timestamp_ms, 11 erob_node, 12 erob_ds402_state, 13 erob_statusword,
14 robstride_timestamp_ms, 15 robstride_id, 16 robstride_mode, 17 robstride_fault,
18 robstride_model, 19 robstride_position_rad, 20 robstride_velocity_rad_s,
21 robstride_torque, 22 robstride_temperature_c,
23 arm_state, 24 ik_state, 25 gripper_current_ma,
26 target_x_mm, 27 target_y_mm, 28 target_z_mm,
29 J1_rad, 30 J2_rad, 31 J3_rad, 32 J4_rad, 33 J5_rad, 34 J6_rad,
35 gripper_force_n, 36 ik_position_error_mm, 37 ik_orientation_error_rad,
38 can_error_flags
```

`telemetry_flags` 的 bit0/1/2 表示力传感器、eRob、RobStride 是否有新反馈，bit3 表示 RS485 错误，bit16/17 表示 CAN1/CAN2 错误。FireWater 只用于观察，不参与控制闭环。
