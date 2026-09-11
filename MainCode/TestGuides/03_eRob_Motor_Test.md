# eRob 电机测试

烧录 `eRob_Motor_Test.hex`，CAN1 接 eRob，默认节点 ID 为 11，波特率 1 Mbit/s。默认固件只读取 `0x6041` 状态字，不使能、不移动。

通过标准：收到合法 SDO 状态字，`g_erob_status_valid=1`，状态机与电机实际状态一致，LED1 翻转，错误标志保持 0。确认上述结果后，才可在 `test_config.h` 中打开动作开关，断开负载并准备急停。
