# 模块测试固件说明

这里放置四个可以单独烧录的 STM32 测试工程。每个工程只验证一个硬件模块，便于在接入主工程前定位接线、供电、总线和协议问题。测试固件默认处于诊断模式：不会使能或移动电机，也不会执行机械臂动作。

烧录前统一要求：断开电机动力电源，确认控制器和外设共地，准备硬件急停；只有对应模块的接线和只读反馈确认正常后，才允许打开动作测试开关。

## 固件与预期效果

构建产物位于 `../../output/standalone_tests/`。每个目录包含该测试工程对应的 `.elf`、`.hex`、`.bin` 和 `.map` 文件。`.hex` 或 `.bin` 用于烧录，`.elf` 只在调试器中加载符号和源码，`.map` 用于排查链接地址；因此“独立测试”指可以单独烧录运行的固件工程，不是直接把 ELF 当作烧录文件。

| 模块 | 固件 | 默认运行效果 | 通过标准 |
| --- | --- | --- | --- |
| UDP 网络 | `udp_communication/UDP_Communication_Test.hex` | 板端使用 `192.168.1.10:20001`，原样回显 1--512 字节 UDP 负载 | 发送 100 个测试包后 `lost=0`、`corrupt=0`，LED1 表示链路状态 |
| 六维力传感器 | `force_sensor/Force_Sensor_Test.hex` | USART6 + 外置 RS485，每 100 ms 请求一帧六维数据 | 六个浮点值持续更新；断开再恢复传感器后接收仍能恢复 |
| eRob 电机 | `erob_motor/eRob_Motor_Test.hex` | CAN1 每 100 ms 读取节点状态字 `0x6041` | 收到合法状态字，CiA 402 状态正确，LED1 翻转；默认不使能、不移动 |
| RobStride 电机 | `robstride_motor/RobStride_Motor_Test.hex` | CAN2 每 200 ms 轮询位置、速度、Iq 和母线电压 | 收到与型号匹配的参数回复和反馈；默认不使能、不移动 |

## 构建

在 `MainCode` 目录执行：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1
```

也可以只构建一个模块：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1 -Project UDP
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1 -Project Force
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1 -Project eRob
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1 -Project RobStride
```

默认生成 Release 版本；需要调试符号时增加 `-Configuration Debug`。

## 烧录

用 ST-LINK 烧录指定测试固件（命令从 `MainCode` 目录执行）：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\flash_firmware.ps1 `
  -Artifact .\output\standalone_tests\udp_communication\UDP_Communication_Test.hex
```

将路径替换为其他模块的 `.hex` 文件即可。烧录主固件时不带 `-Artifact`，脚本默认使用 `output/firmware/ChallengeCup_Main.hex`。

## 推荐测试顺序

1. **UDP**：先验证开发板启动、PHY 链路和网络配置。
2. **力传感器**：确认 RS485 方向控制、波特率、帧解析和错误恢复。
3. **eRob**：确认 CAN1、节点 ID、终端电阻和状态字，再考虑一次性小范围位置动作。
4. **RobStride**：确认 CAN2、准确型号、节点 ID 和反馈模式，再考虑一次性小电流动作。

详细接线、观测变量和动作开关见各子目录 README：

- [UDP 通讯测试](udp_communication/README.md)
- [六维力传感器测试](force_sensor/README.md)
- [eRob 电机测试](erob_motor/README.md)
- [RobStride 电机测试](robstride_motor/README.md)

## 动作测试规则

动作测试只用于验证单个驱动器，不代表整机可以直接运行。修改对应 `Inc/test_config.h` 后重新构建：

- eRob：将 `EROB_TEST_ENABLE_MOTION` 改为 `1`，设置正确的节点 ID、编码器位置和轮廓速度。
- RobStride：将 `ROBSTRIDE_TEST_ENABLE_MOTION` 改为 `1`，设置正确的型号、节点 ID 和小电流值。

首次动作必须无负载、小步进、低电流，并由人员持续控制急停。任何 CAN 错误、反馈超时、型号不符或状态不正确都会保持停机。
