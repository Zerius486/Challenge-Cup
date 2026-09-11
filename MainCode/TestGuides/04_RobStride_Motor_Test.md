# RobStride 电机测试

烧录 `RobStride_Motor_Test.hex`，CAN2 接 RobStride，默认电机 ID 为 15，型号默认 RS01，波特率 1 Mbit/s。默认只读取位置、速度、Iq、温度和母线电压。

通过标准：反馈中的电机 ID 和型号匹配实物，位置/速度/Iq/温度数值连续刷新，故障位为 0。确认后才可打开动作开关，使用无负载小步进和低电流，并全程保留急停。
