# Challenge Cup 机械臂工程

这是五轴机械臂加末端夹爪（共六个电机）的完整交付目录。工程按“参考资料”和“可编译代码”分开：

```text
Challenge Cup/
├─ Reference/                 厂商手册、通信协议和 UWV 借鉴例程（只读）
├─ MainCode/                  STM32F407 主工程和四个独立模块测试固件
├─ ARM_SYSTEM_DESIGN.md       系统设计、坐标系、逆运动学和行为逻辑
└─ README.md                  本目录导航
```

## 交付内容

- 主固件：`MainCode/output/firmware/ChallengeCup_Main.{elf,hex,bin,map}`
- 独立模块测试固件：`MainCode/output/standalone_tests/` 下的 eRob、RobStride、力传感器和 UDP 工程
- 主工程源码：`MainCode/App`、`MainCode/Core`、`MainCode/Config`、`MainCode/Drivers`、`MainCode/Middlewares`
- 测试与说明：`MainCode/Projects/ModuleTests`、`MainCode/TestGuides`、`MainCode/docs`

## 快速入口

1. 先阅读 [系统设计说明](ARM_SYSTEM_DESIGN.md)，确认六个关节、坐标系和 NUC 指令格式。
2. 再阅读 [MainCode README](MainCode/README.md) 和 [硬件接线说明](MainCode/docs/HARDWARE_PORT.md)。
3. 按 [模块测试说明](MainCode/Projects/ModuleTests/README.md) 依次验证 UDP、力传感器和两类电机。
4. 硬件确认无误后，使用 `MainCode/tools/build_firmware.ps1` 构建和 `flash_firmware.ps1` 烧录主固件。

板级原理图和筛选后的启明欣欣资料位于 [`Reference/Board/`](Reference/Board/README.md)。

## 当前验证状态

主工程 Debug/Release 交叉编译、四个独立模块测试固件和 MinGW 主机测试均已通过。实物板、电机、RS485 收发器和机械限位仍需按测试说明完成上板验收；编译通过不等同于带载安全验收。
