# 独立测试 ELF

四个测试固件分别验证网络、RS485 力传感器和两类 CAN 电机。默认均为诊断模式，不使能、不移动电机。烧录前断开电机动力电源，接好硬件急停和限流电源。

| ELF | 默认动作 | 通过标准 |
| --- | --- | --- |
| `output/standalone_tests/udp_communication/UDP_Communication_Test.elf` | 静态 IP `192.168.1.10:20001` UDP 回显 | 主机发送任意 UDP 数据后收到完全相同的数据；LED1 随收包翻转 |
| `output/standalone_tests/force_sensor/Force_Sensor_Test.elf` | USART6 + PB0 驱动外置 RS485，周期请求一帧 | 能持续解析 6 个 float，计数递增；断开/恢复传感器后 DMA 能继续接收 |
| `output/standalone_tests/erob_motor/eRob_Motor_Test.elf` | CAN1 读取 eRob ID 11 的 `0x6041` | 收到合法状态字，状态机显示正确，LED1 翻转；默认不发使能和位置 |
| `output/standalone_tests/robstride_motor/RobStride_Motor_Test.elf` | CAN2 读取 RobStride ID 15 的反馈 | 收到位置、速度、Iq、温度等反馈，模型与 ID 匹配；默认不使能和运动 |

## 构建

在 `MainCode` 目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1
```

产物在 `output/standalone_tests/<name>/`，包含 ELF、HEX、BIN、MAP。单独测试时也可传 `-Project UDP`、`-Project Force`、`-Project eRob` 或 `-Project RobStride`。

## 测试顺序

1. UDP：电脑有线网卡设为 `192.168.1.20/24`，运行 `python .\tools\udp_client.py --host 192.168.1.10 hello` 或 `udp_echo_test.py`，确认回显。
2. 力传感器：PC6 接收发器 DI、PC7 接 RO、PB0 接 DE 与 `/RE`，确认 3.3 V 逻辑和传感器独立供电；观察六维数据和错误恢复。
3. eRob：确认 CAN1 终端、ID 11 和 1 Mbit/s，只读状态字通过后才允许修改 `EROB_TEST_ENABLE_MOTION`。
4. RobStride：确认型号 `ROBSTRIDE_TEST_MODEL` 与实物一致、ID 15 和 1 Mbit/s，只读反馈通过后才允许改动作开关。

任何动作测试必须无负载、小步进、低电流，并保留人工急停。测试通过只表示接口和协议链路正常，不替代整机机械限位验收。
