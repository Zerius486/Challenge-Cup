# UDP 线协议 v1

## 传输端点

- 设备静态地址：`192.168.1.10/24`
- 主协议：UDP `20001`
- 多字节整数和 float：小端
- float：IEEE-754 binary32
- 单个数据报最大长度：272 字节（12 字节头 + 256 字节负载 + 4 字节 CRC）

UDP 不保证到达、顺序或唯一性。控制端必须检查 ACK/ERROR、保持递增序号，并以短周期持续刷新运动命令。安全系统不能只依赖网络停机。

## 通用帧

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 2 | magic | 固定 `55 57`，ASCII `UW` |
| 2 | 1 | version | 固定 `01` |
| 3 | 1 | type | 消息类型 |
| 4 | 2 | flags | v1 固定为 0 |
| 6 | 2 | payload_length | 负载字节数，0..256 |
| 8 | 4 | sequence | 无符号 32 位序号 |
| 12 | N | payload | 消息负载 |
| 12+N | 4 | crc32 | 对偏移 0 到 11+N 计算的标准 CRC-32/ISO-HDLC，线序小端 |

CRC 参数与 Python `zlib.crc32()` 一致：反射多项式 `0xEDB88320`，初值和最终异或均为 `0xFFFFFFFF`。

## 消息类型

| type | 名称 | 方向 | 负载 |
| ---: | --- | --- | --- |
| `0x01` | HELLO | 主机到设备 | 空；登记遥测端并重建序号会话 |
| `0x10` | MOTOR_COMMAND | 主机到设备 | 28 字节电机命令 |
| `0x11` | SAFE_STOP | 主机到设备 | 空；停止当前固件已跟踪的全部电机 |
| `0x20` | FORCE_COMMAND | 主机到设备 | 1 或 2 字节传感器命令 |
| `0x80` | TELEMETRY | 设备到主机 | 68 字节遥测 |
| `0x81` | ACK | 设备到主机 | 4 字节结果 |
| `0x82` | ERROR | 设备到主机 | 4 字节结果 |
| `0x83` | ARM_STATUS | 设备到主机 | 64 字节机械臂状态 |

除上述二进制帧外，主端口也接受机械臂任务使用的 ASCII 数据报：

```text
PXYZ:+030000,-001250,+012500,/
JZH_+500_1000/
ST:P/
```

`PXYZ` 的坐标单位为 `0.01 mm`，`JZH` 的第一个字段为 `0.001 rad`、第二个字段为最大电流 mA。ASCII 命令的即时应答为 `ARM:OK/`、`ARM:WAIT/`、`ARM:UNREACHABLE/` 或 `ARM:LOCKED/`；详细执行状态通过 `0x83` 周期回传。
首次收到运动或夹爪指令时，如果电机仍在使能稳定期，固件返回 `ARM:WAIT/` 并保留最新目标，由主循环在反馈就绪后自动继续；停止、超时或掉线会清除待执行目标。

`ARM_STATUS (0x83)` 的 64 字节负载为：`board_ms(u32)`、`arm_state(u8)`、`ik_status(u8)`、`gripper_current_limit_ma(u16)`、目标 `x/y/z(3xf32, mm)`、J1..J5 命令角（5xf32, rad）、夹爪目标（f32, rad）、力大小（f32, N）、最后命令时间（u32）、位置误差（f32, mm）、姿态误差（f32, rad）和力阈值（f32, N）。

除 HELLO 和 SAFE_STOP 外，序号必须在模 2^32 意义下晚于上一条已接收命令；重复或倒序帧返回 `REPLAY`。设备当前只维护一个主协议遥测对端，最后一个通过完整格式与 CRC 校验的数据报会接管遥测目的地址。

## ACK/ERROR 负载

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 1 | 原请求 type |
| 1 | 1 | status |
| 2 | 2 | detail，小端 |

status：

| 值 | 名称 | 含义 |
| ---: | --- | --- |
| 0 | OK | 请求已被固件接受 |
| 1 | BAD_FRAME | 帧头、长度或 CRC 错误；detail 是解码错误码 |
| 2 | BAD_COMMAND | 负载、节点、保留位或参数错误 |
| 3 | SAFETY_LOCKED | 危险运动被编译安全锁拒绝，或未带 ARM 位 |
| 4 | UNSUPPORTED | v1 不支持该消息类型 |
| 5 | IO_ERROR | CAN/UART 发送失败，或该驱动不支持此操作/数值 |
| 6 | REPLAY | 序号重复或倒序 |
| 7 | NOT_READY | eRob 未满足稳定期/状态门控，或 RobStride 缺少 250 ms 内无故障的 Motor mode 反馈；固件同时执行停机 |

ACK 仅表示板端接收并提交请求，不等价于电机或传感器已经执行成功。必须结合反馈、状态字和独立安全监测判断。

## MOTOR_COMMAND 负载

| 偏移 | 长度 | 字段 | 说明 |
| ---: | ---: | --- | --- |
| 0 | 1 | can_bus | eRob 固定 1，RobStride 固定 2 |
| 1 | 1 | driver | 1=eRob，2=RobStride |
| 2 | 1 | node_id | 1..127 |
| 3 | 1 | operation | 见下表 |
| 4 | 4 | position | 驱动/操作相关 |
| 8 | 4 | velocity | 驱动/操作相关 |
| 12 | 4 | torque | 驱动/操作相关 |
| 16 | 4 | kp | RobStride 运控模式 |
| 20 | 4 | kd | RobStride 运控模式 |
| 24 | 2 | timeout_ms | 20..5000；运动刷新超时后自动停机 |
| 26 | 2 | options | bit15=ARM；bits2..0=RobStride 型号 0..6；其余为 0 |

operation：

| 值 | 操作 | 危险命令 | eRob | RobStride |
| ---: | --- | --- | --- | --- |
| 0 | DISABLE | 否 | `0x6040=0` | 类型 4 停止 |
| 1 | ENABLE | 是 | `6 -> PV 模式 -> 目标速度 0 -> 7 -> 15` | 类型 4 停止 -> current 模式 -> `iq_ref=0` -> 类型 3 |
| 2 | STOP | 否 | 快速停机 `0x6040=2` | 类型 4 |
| 3 | SET_ZERO | 是 | 不支持 | 类型 6 |
| 4 | POSITION | 是 | position=编码器 plus，velocity=plus/s | position=rad，velocity=PP 轮廓速度 rad/s |
| 5 | VELOCITY | 是 | velocity=plus/s | velocity=rad/s |
| 6 | TORQUE | 是 | torque=额定电流千分比，范围 -1000..1000 | torque 字段写入 `iq_ref`，单位 A |
| 7 | IMPEDANCE | 是 | 不支持 | position rad、velocity rad/s、torque N·m、kp、kd |
| 8 | FAULT_RESET | 否 | `0x6040=0x80` | 类型 4，data[0]=1 |

RobStride 型号编码：0=RS00、1=RS01、2=RS02、3=RS03、4=RS04、5=RS05、6=RS06。eRob 的 bits2..0 必须为 0。

eRob ENABLE 的看门狗从 500 ms 稳定期结束后开始；重复 ENABLE 会重新开始稳定期并使之前缓存的状态字失效。运动命令还要求同一节点在最近 250 ms 内报告 `OPERATION_ENABLED`。RobStride 后续位置、速度、电流或运控命令必须先由本固件成功提交 ENABLE；ENABLE 同样使旧反馈失效，之后还要求同一节点在最近 250 ms 内报告无故障的 Motor mode。其他危险命令在收到时刷新 `timeout_ms`。条件未满足时固件拒绝并停机，之后必须重新使能。同一已跟踪 RobStride 节点在 STOP/DISABLE 前必须保持相同型号编号。

## FORCE_COMMAND 负载

第 0 字节是传感器命令：`01` 停止、`02` 1 kHz 连续发送、`03` 单次读取、`05` SN、`06` 下次上电自动发送、`07` 固件版本、`30` 清零、`31/32` 退出/进入 debug、`33` kg、`34` 原始电压、`35` N。

命令 `04` 修改波特率时必须有第 1 字节：`01`=460800、`02`=691200、`03`=921600。修改只在传感器停止发送时进行，并需重新上电；固件 UART 的 `APP_FORCE_UART_BAUD` 也必须同步重编译。

## TELEMETRY 负载

| 偏移 | 长度 | 字段 |
| ---: | ---: | --- |
| 0 | 4 | board_ms |
| 4 | 4 | flags |
| 8 | 4 | force_timestamp_ms |
| 12..35 | 24 | Fx, Fy, Fz, Mx, My, Mz，6 个 float |
| 36 | 4 | erob_timestamp_ms |
| 40 | 1 | erob_node_id |
| 41 | 1 | erob_ds402_state |
| 42 | 2 | erob_statusword |
| 44 | 4 | robstride_timestamp_ms |
| 48 | 1 | robstride_motor_id |
| 49 | 1 | robstride_mode_state |
| 50 | 1 | robstride_fault_bits |
| 51 | 1 | robstride_model |
| 52..67 | 16 | position rad、velocity rad/s、torque N·m、temperature degC |

flags：bit0=最近 1 s 内收到恢复后的有效力数据，bit1=至少收到 eRob 状态字，bit2=至少收到 RobStride 反馈，bit3=力传感器 UART 启动失败或接收曾发生错误，bit8=固件运动锁定，bit16/17=CAN1/CAN2 曾报告错误。串口 DMA 恢复或力数据超过 1 s 未更新时 bit0 会清零，收到下一帧合法数据后再置位；电机有效位表示存在最近值，不表示它仍新鲜。控制端必须根据对应时间戳计算数据龄期。

## 旧 UWV 兼容端口

- `20001` 同时识别以 `/` 结尾的旧 ASCII。兼容层只把明确的 `ST:P/` 或 `ZT/` 视为全局停机；NUC 协议中的 `ZT_<pwm>`（钻头）和 `JC_<pwm>,<force>` 不会误触发停机。`L_<left>_<right>`（也兼容资料示例中的 `L_<left>,<right>`）与 `D_<0|1>`（同时兼容旧写法 `D:<0|1>`）只解析不执行。
- `19999` 接收旧 15 字节裸 CAN 帧：IDE、RTR、4 字节大端 ID、DLC、8 字节数据。默认只放行结构严格匹配的 eRob/RobStride 停止帧；任意注入需显式打开危险编译开关。
- `20000` 向最近一个被 19999 端口接受的主机 IP 镜像 CAN2 接收帧，目标 UDP 端口固定为 20000。

兼容入口没有 CRC、认证或会话隔离，只用于迁移诊断，不应用作最终运动控制接口。
