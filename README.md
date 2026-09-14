# Challenge Cup 机械臂工程

这是五轴机械臂加末端夹爪（共六个电机）的工程目录。工程按“参考资料”“主控源码”和“NUCControl 上位机模块”分开：

```text
Challenge Cup/
├─ Reference/                 厂商手册、通信协议和 UWV 借鉴例程（只读）
├─ MainCode/                  STM32F407 主工程和六个独立模块测试固件
├─ NUCControl/                Python 平面二连杆逆解和关节角度 UDP 报文生成
├─ ARM_SYSTEM_DESIGN.md       系统设计、坐标系、逆运动学和行为逻辑
└─ README.md                  本目录导航
```

## 交付内容

- 主固件源码：`MainCode/App`、`MainCode/Core`、`MainCode/Config`、`MainCode/Drivers`、`MainCode/Middlewares`
- NUC 端控制类：`NUCControl/`，输入末端三维坐标并生成 J1~J6 关节角度 UDP 报文
- 独立模块测试源码：`MainCode/Projects/ModuleTests` 和 `MainCode/Tests`
- 主工程源码：`MainCode/App`、`MainCode/Core`、`MainCode/Config`、`MainCode/Drivers`、`MainCode/Middlewares`
- 测试与说明：`MainCode/Projects/ModuleTests`、`MainCode/TestGuides`、`MainCode/docs`

## 快速入口

1. 先阅读 [系统设计说明](ARM_SYSTEM_DESIGN.md)，确认六个关节、坐标系和角度报文格式。
2. 上位机使用方式和参数配置见 [NUCControl README](NUCControl/README.md)。
3. 主控源码、协议和测试说明见 [MainCode README](MainCode/README.md)。
4. 需要重新生成固件时，使用 `MainCode/tools/build_firmware.ps1`；编译产物默认保存在本地构建目录，不纳入工程源码。

板级原理图和筛选后的启明欣欣资料位于 [`Reference/Board/`](Reference/Board/README.md)。

## 当前验证状态

源代码和主机测试可独立构建；实物板、电机、RS485 收发器和机械限位仍需按测试说明完成上板验收。编译通过不等同于带载安全验收。
