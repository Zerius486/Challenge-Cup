# Challenge Cup STM32F407 主控工程

本目录是面向启明欣欣 STM32F407 高配版 V6.1（STM32F407ZGTx）的主工程。它借鉴 UWV 例程的以太网通信方式，并按本开发板原理图完成时钟、RMII、双 CAN、LED 和力传感器串口配置。

当前状态：源码已通过 Debug/Release 交叉编译，主机协议测试 `protocol_tests` 和 `app_bridge_tests` 已用 `D:\mingw64` 编译并通过；尚未在实物板、电机和力传感器上完成硬件在环验收。首次上板必须按本文的无负载流程进行。

## 目录结构

| 目录 | 用途 |
| --- | --- |
| `App/` | 应用层协议、驱动封装、逆运动学和机械臂控制 |
| `Core/` | STM32 启动、板级初始化和主循环 |
| `Config/` | CubeMX IOC 配置记录 |
| `Drivers/`、`Middlewares/` | STM32 HAL/CMSIS、LAN8720A 和 lwIP |
| `Projects/ModuleTests/` | UART、CAN、UDP、eRob、RobStride、力传感器六个独立测试工程 |
| `Tests/` | MinGW 主机协议和应用桥接测试 |
| `docs/`、`TestGuides/` | 实现说明、接线说明和上板验收步骤 |
| `tools/` | 构建、烧录和 UDP 调试脚本 |
| `output/` | 最近生成的 ELF、HEX、BIN、MAP 文件；不参与编译 |
| `build/` | CMake 中间文件；可删除后重新配置生成 |

## 已实现功能

| 模块 | 实现 | 默认连接 |
| --- | --- | --- |
| 零差云控 eRob | CANopen NMT、快速 SDO、CiA 402 状态解析、位置/速度/转矩命令、状态字轮询、超时停机 | CAN1，PD0/PD1，1 Mbit/s |
| 灵足时代 RobStride | 官方私有协议类型 0/1/2/3/4/6/7/17/18/21/22/23/24/25，RS00-RS06 运控/电流限幅和反馈解析 | CAN2，PB12/PB13，1 Mbit/s |
| UDP | CRC32、序号去重、ACK/ERROR、50 Hz 遥测、安全停机、运动看门狗 | `192.168.1.10:20001` |
| 旧 UWV 兼容 | 19999 端口 15 字节 CAN 桥接、20000 端口 CAN 镜像、`ST:P/` 和 `ZT/` 停机 | 任意 CAN 注入默认关闭 |
| 中科米点六维力传感器 | RS485 命令、循环 DMA 接收、串口错误自动恢复、分帧/重同步、六路 float 解码 | USART6 PC6/PC7 + PB0 DE，460800 8N1 |

机械臂逆运动学已加入 `App/Inc/arm_kinematics.h` 和
`App/Src/arm_kinematics.c`：以底盘中心的基座平面为坐标原点，使用
`l1=419.9 mm`、`l2=122.95 mm`、`l3=124.0 mm`、`l4=208.12 mm`，
采用阻尼最小二乘和零空间姿态项求解 `J1=Yaw`、`J2..J5=Pitch` 的位置逆解。
末端 `l4` 通过 `q2+q3+q4+q5=0` 约束保持水平；J6 夹爪不参加位置逆解，仍由位置目标和力传感器反馈单独控制。
当前默认软件限位为 J1 `±170°`、J2 `±90°`、J3/J4 `±150°`、J5 `±180°`，仅用于初始调试，实物上电前必须按机械干涉和线缆范围重新确认。

详细资料：

- [硬件移植与接线](docs/HARDWARE_PORT.md)
- [UDP 线协议](docs/UDP_PROTOCOL.md)
- [驱动协议与单位](docs/DRIVERS.md)
- [CubeMX 配置说明](docs/CUBEMX_CONFIGURATION.md)
- [可导入的 CubeMX 配置](Config/ChallengeCup_Main.ioc)
- [VOFA+ FireWater 回传](docs/VOFA_FIREWATER.md)
- [模块测试固件说明](Projects/ModuleTests/README.md)

## 安全默认值

提交版固件默认开启远程运动，`APP_REMOTE_MOTION_ALLOWED=1`。每个危险 UDP
命令仍必须在 `options` 中设置 `0x8000` ARM 位，命令行工具中对应 `--arm`；
停机、失能、故障复位和传感器命令始终可用。若制作台架安全版本，将
`App/Inc/app_config.h` 中该宏改为 `0` 后重新编译。

旧 19999 端口的任意 CAN 注入由 `APP_LEGACY_RAW_CAN_ALLOWED` 控制，默认是 `0`。旧履带速度到新电机单位和节点的映射并不明确，因此本固件只解析旧履带命令，不提供执行开关，也不猜测执行。

## 构建

要求 Windows PowerShell 和 STM32CubeCLT。脚本会优先使用默认安装目录 `C:\ST\STM32CubeCLT_1.22.0`，也可使用 `PATH` 中的 CMake、Ninja 和 Arm GNU Toolchain。

```powershell
cd "D:\Challenge Cup\MainCode"
powershell -ExecutionPolicy Bypass -File .\tools\build_firmware.ps1 -Configuration Release
```

产物写入 `output/firmware/`：

- `ChallengeCup_Main.elf`
- `ChallengeCup_Main.hex`
- `ChallengeCup_Main.bin`
- `ChallengeCup_Main.map`

## 六个模块测试工程

`Projects/ModuleTests/` 提供六个可分别构建和烧录的最小测试固件：UART、CAN、UDP、eRob 单电机、RobStride 单电机和六维力传感器。两个电机工程默认只读反馈，不会使能或移动电机。

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\build_standalone_tests.ps1
```

各工程的接线、调试变量、动作保护和单独构建命令见 [Projects/ModuleTests/README.md](Projects/ModuleTests/README.md)。

烧录（ST-LINK/SWD）：

```powershell
powershell -ExecutionPolicy Bypass -File .\tools\flash_firmware.ps1
```

## 首次上板

1. 先不接电机动力电源，检查 MCU 的 8 MHz HSE 晶振、LAN8720A 的 25 MHz PHY 晶振、3.3 V、电源地和 SWD。
2. 烧录默认安全固件。LED0（PE3）约 1 Hz 翻转；网线连接后 LED1（PE4）点亮；致命初始化错误时 LED2（PG9）常亮。三个 LED 均低电平点亮。
3. 将电脑有线网卡设为例如 `192.168.1.20/24`，再执行：

```powershell
python .\tools\udp_client.py --host 192.168.1.10 hello
python .\tools\udp_client.py --host 192.168.1.10 listen --seconds 5
```

4. 接入外置 3.3 V RS485 收发器和力传感器，先读单帧：

```powershell
python .\tools\udp_client.py --host 192.168.1.10 force once
python .\tools\udp_client.py --host 192.168.1.10 listen --seconds 5
```

5. 断电后接 CANH/CANL/公共地，确认总线两端终端电阻和电机端 1 Mbit/s。保持机械悬空或拆除负载，先只观察 CAN 报文与 ACK。
6. 提交版已开启运动编译开关。eRob 使能后固件强制等待 500 ms，并要求收到 250 ms 内的 `OPERATION_ENABLED` 状态；RobStride 运动要求先由本固件成功提交 ENABLE，并收到 250 ms 内无故障的 Motor mode 反馈。灵足电机必须选择准确型号，型号编号为 RS00=0 到 RS06=6。

示例命令见 `python .\tools\udp_client.py motor --help`。任何运动测试都应先准备独立硬件急停和限流电源。

## 验证

本工程使用 `-Wall -Wextra -Wpedantic` 构建。协议测试覆盖 eRob SDO/CiA 402、灵足扩展帧和型号限幅、力传感器分帧、UDP CRC/序号字段、旧 UDP/CAN 格式以及远程帧拒绝；应用桥接测试覆盖安全锁、eRob 重复使能稳定期与位置触发沿、RobStride 电流限幅、序号重放、停机重试和力传感器 DMA 故障恢复到遥测的数据链路。

开发机若有 C11 编译器，可用 CMake 构建两套测试；在本机 MinGW 安装于 `D:\mingw64` 时，命令如下：

```powershell
$env:PATH = "D:\mingw64\bin;" + $env:PATH
cmake -S . -B build\host-tests -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=D:/mingw64/bin/gcc.exe
cmake --build build\host-tests --parallel
ctest --test-dir build\host-tests --output-on-failure
```

解析器还包含固定种子的随机畸形输入回归；在具备 Clang/GCC sanitizer 的环境中，建议再用 AddressSanitizer 和 UndefinedBehaviorSanitizer 运行同一测试。UDP 帧中的 CRC32 是协议字段，除此之外不在固件或构建脚本中增加 hash 校验。

## 工程边界

`Config/ChallengeCup_Main.ioc` 是当前引脚、时钟和外设分配的单一配置记录；`../Reference/UWV_Example/Extracted/UWV_0926 20241107/SEU_EthToCan.ioc` 仅用于追溯原 UWV 配置，不能覆盖本工程。由于当前环境没有随 CubeCLT 安装 CubeMX GUI，实际编译仍以手写的 [Core/Src/board.c](Core/Src/board.c) 与 [Core/Src/main.c](Core/Src/main.c) 为准；在 CubeMX 中重新生成前请先备份这两个文件。权威构建入口是 [CMakeLists.txt](CMakeLists.txt)。
