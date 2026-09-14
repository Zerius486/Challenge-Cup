# NUCControl

NUCControl 用于计算一个简化平面机械臂的关节角度。输入车体坐标系中的末端目标坐标，程序计算姿态，并生成只包含 J1~J6 角度的 UDP 数据包。

机械臂始终位于一个与底盘垂直的平面内，末端总成始终水平指向车体前方。因此，位置逆解只需要求解 `l1`、`l2` 两根连杆的两个天顶角。

## 1. 机械臂模型

机械臂包含以下关节：

| 关节 | 作用 | 输出角度单位 |
|---|---|---|
| J1 | 底座 yaw，确定垂直平面的方向 | rad |
| J2 | `l1` 的俯仰 | rad |
| J3 | `l2` 相对 `l1` 的俯仰 | rad |
| J4 | 调整上部总成，使 `l3` 保持水平 | rad |
| J5 | `l4` 绕自身轴滚转 | rad |
| J6 | 夹爪开合 | rad |

物理结构虽然有四段长度，但 `l3` 和 `l4` 在俯仰方向组成固定的水平总成；`l4` 仍可绕自身轴 360° 滚转。逆运动学使用三段有效长度：

```text
(l1, l2, l3 + l4)
```

坐标系约定：`x` 指向车体前方，`y` 指向横向，`z` 竖直向上，单位为 mm。目标点必须位于机械臂所在的垂直平面内，允许的横向误差由 `PLANE_TOLERANCE_MM` 设置。

## 2. 上位机调用入口

上位机要让机械臂从当前位置移动到目标坐标，直接调用 `send_trajectory()`，目标坐标填写在 `target` 参数中：

```python
from NUCControl import NUCControl

arm = NUCControl()
arm.send_trajectory(
    target=(700.0, 0.0, 300.0),  # 在这里填入 x、y、z，单位 mm
    send_packets=lambda packet: udp_socket.sendto(packet, board_address),
    wait_complete=receive_complete_signal,
    sequence=0,
    steps=20,
)
```

`send_trajectory()` 会按轨迹逐帧发送角度报文，并在每帧后调用 `wait_complete()` 等待单片机完成信号。

如果上位机只需要计算角度或自行发送 UDP，可使用：

| 需求 | 函数 | 坐标参数 |
|---|---|---|
| 只计算 J1~J6 角度 | `joint_angle_values(coordinate)` | `coordinate=(x,y,z)` |
| 生成单帧 UDP 报文 | `angle_packet(coordinate, sequence)` | `coordinate=(x,y,z)` |
| 生成完整轨迹报文 | `trajectory_packets(target, sequence, steps=...)` | `target=(x,y,z)` |
| 发送并等待完成 | `send_trajectory(target, ...)` | `target=(x,y,z)` |

## 3. 安装与基本调用

在项目根目录导入：

```python
from NUCControl import NUCControl

arm = NUCControl()

# 输入末端坐标，得到 J1~J6 的角度（弧度）
angles = arm.joint_angle_values(
    (700.0, 0.0, 300.0),
    wrist_roll_rad=1.57,
    gripper_rad=0.5,
)
print(angles)  # (J1, J2, J3, J4, J5, J6)

# 直接生成一条只包含六个角度的 UDP 报文
packet = arm.angle_packet((700.0, 0.0, 300.0), sequence=1, gripper_rad=0.5)
```

也可以只获取逆解结果：

```python
result = arm.inverse_kinematics((700.0, 0.0, 300.0))
print(result.zenith_angles_rad)  # l1、l2 的天顶角
print(result.joint_angles_rad)   # J1~J5 角度
```

角度单位均为弧度。天顶角以竖直向上为 0，向车体前方转动为正。

## 4. 配置文件

所有默认参数位于 [config.py](config.py)。直接修改文件中的常量即可改变默认配置；也可以创建 `ArmConfig` 传给 `NUCControl`：

```python
from NUCControl import ArmConfig, NUCControl

config = ArmConfig(
    link_lengths_mm=(419.9, 122.95, 124.0, 208.12),
    initial_zenith_angles_rad=(0.0, 0.0),
    initial_wrist_roll_rad=0.0,
    base_yaw_rad=0.0,
    top_link_horizontal_angle_rad=0.0,
    joint_node_ids=(11, 12, 13, 14, 15, 16),
)
arm = NUCControl(config)
```

### 4.1 连杆和初始姿态

| 参数 | 类型/单位 | 含义 |
|---|---|---|
| `LINK_LENGTHS_MM` / `link_lengths_mm` | 3 或 4 个浮点数，mm | 可写 `(l1,l2,l3+l4)`，也可写物理长度 `(l1,l2,l3,l4)`。使用四段时程序自动将 `l3+l4` 合并。默认值为 `(419.9,122.95,124.0,208.12)`。 |
| `INITIAL_LINK_ANGLES_RAD` / `initial_link_angles_rad` | 4 个弧度值 | 四段连杆的初始绝对方向，依次对应 `l1~l4`。`l3`、`l4` 必须与水平固定方向一致。 |
| `INITIAL_ZENITH_ANGLES_RAD` / `initial_zenith_angles_rad` | `(theta1,theta2)`，rad | `l1`、`l2` 的初始天顶角。未指定 `initial_link_angles_rad` 时使用此参数。 |
| `INITIAL_WRIST_ROLL_RAD` / `initial_wrist_roll_rad` | rad | J5 初始滚转角。 |
| `BASE_YAW_RAD` / `base_yaw_rad` | rad | J1 初始 yaw；0 表示平面朝向车体前方。 |
| `TOP_LINK_HORIZONTAL_ANGLE_RAD` / `top_link_horizontal_angle_rad` | rad | `l3+l4` 的固定方向；0 表示严格水平向前。 |

### 4.2 运动范围和求解参数

| 参数 | 含义 |
|---|---|
| `ZENITH_MIN_RAD`、`ZENITH_MAX_RAD` | `l1`、`l2` 天顶角的最小/最大值。 |
| `JOINT_MIN_RAD`、`JOINT_MAX_RAD` | J1~J5 的软件角度限制。 |
| `PLANE_TOLERANCE_MM` | 目标点偏离固定平面的最大横向误差。 |
| `POSITION_TOLERANCE_MM` | 逆解后末端位置允许误差。 |
| `REACH_TOLERANCE_MM` | 判断两连杆是否可达时使用的数值容差。 |
| `ARM_DIRECTION` | 电机正方向，通常为 `1.0` 或 `-1.0`。 |

### 4.3 夹爪参数

| 参数 | 含义 |
|---|---|
| `GRIPPER_MIN_RAD`、`GRIPPER_MAX_RAD` | J6 夹爪角度范围。 |
| `GRIPPER_MAX_CURRENT_MA` | 允许设置的最大夹爪电流，单位 mA。 |

## 5. 关节角度对应关系

设逆解得到 `theta1`、`theta2`，则发送给电机的 J1~J5 角度为：

```text
J1 = BASE_YAW_RAD
J2 = π/2 - theta1
J3 = theta1 - theta2
J4 = TOP_LINK_HORIZONTAL_ANGLE_RAD - (π/2 - theta2)
J5 = wrist_roll_rad
```

同一个目标通常有两组肘部解。未指定 `elbow_branch` 时，程序会以当前姿态（或传入的 `seed_zenith_angles_rad`）为参考，计算 J1~J5 的关节角总变化量，并自动选择变化量较小的一组。需要固定某一组解时，可传入 `elbow_branch=0` 或 `elbow_branch=1`。

末端位置计算为：

```text
r = l1 * sin(theta1) + l2 * sin(theta2) + (l3+l4) * cos(top_angle)
z = l1 * cos(theta1) + l2 * cos(theta2) + (l3+l4) * sin(top_angle)
x = r * cos(BASE_YAW_RAD)
y = r * sin(BASE_YAW_RAD)
```

## 6. UDP 关节角度报文

NUCControl 的轨迹接口只发送关节角度，不发送 CAN、驱动器、速度、力矩等底层电机控制字段。每个轨迹帧对应一个 UDP 报文，接收端根据角度自行完成电机控制。

### 6.1 完整报文结构

```text
12 字节头部 + 24 字节 payload + 4 字节 CRC32 = 40 字节
```

头部采用小端序：

```python
struct.pack("<2sBBHHI", b"UW", 1, 0x12, 0, 24, sequence)
```

| 字段 | 类型 | 值/含义 |
|---|---|---|
| `magic` | 2 字节 | `b"UW"` |
| `version` | uint8 | `1` |
| `message_type` | uint8 | `0x12`，关节角度 |
| `flags` | uint16 | `0` |
| `payload_length` | uint16 | `24` |
| `sequence` | uint32 | UDP 序号，每帧递增 |

报文末尾为前 36 字节的 CRC32：

```python
crc = zlib.crc32(header + payload) & 0xFFFFFFFF
```

### 6.2 payload 格式

```python
struct.pack("<6f", j1, j2, j3, j4, j5, j6)
```

| 顺序 | 字段 | 类型 | 单位 |
|---:|---|---|---|
| 0 | `J1` | float32 | rad |
| 1 | `J2` | float32 | rad |
| 2 | `J3` | float32 | rad |
| 3 | `J4` | float32 | rad |
| 4 | `J5` | float32 | rad |
| 5 | `J6` | float32 | rad |

不传 `gripper_rad` 时，J6 使用 `0.0 rad`。

### 6.3 生成报文

```python
angles = arm.joint_angle_values(
    (700.0, 0.0, 300.0),
    wrist_roll_rad=0.0,
    gripper_rad=0.5,
)
packet = arm.joint_angle_packet(angles, sequence=100)
```

也可以直接由坐标生成：

```python
packet = arm.angle_packet(
    (700.0, 0.0, 300.0),
    sequence=100,
    gripper_rad=0.5,
)
```

## 7. 轨迹分段发送

轨迹在关节空间中按五次最小 jerk 曲线分成多个位置帧，每一帧只有一个包含 J1~J6 角度的 UDP 报文。曲线函数为 `s(t)=10t³-15t⁴+6t⁵`，因此起点和终点的速度、加速度均为 0：

```python
packets = arm.trajectory_packets(
    target=(700.0, 0.0, 300.0),
    sequence=100,
    steps=20,
    gripper_rad=0.5,
)
# packets[0]、packets[1] ... 分别对应各个轨迹帧
```

如果需要边发送边等待单片机返回完成信号，可以使用 `send_trajectory`：

```python
def send_packets(packets):
    udp_socket.sendto(packets, board_address)

def wait_complete():
    # 由调用方解析单片机返回的 UCP/UDP 完成信号
    return receive_complete_signal()

sent_frames = arm.send_trajectory(
    target=(700.0, 0.0, 300.0),
    send_packets=send_packets,
    wait_complete=wait_complete,
    sequence=100,
    steps=20,
)
```

函数行为是：发送当前帧的角度报文；如果不是最后一帧，则等待 `wait_complete()` 返回真值后再发送下一帧。返回值为实际发送的帧数。

## 8. 代码划分

核心代码和辅助程序分开使用：

| 文件 | 用途 |
|---|---|
| `config.py` | 机械臂长度、初始姿态、限位等配置。 |
| `arm_control.py` | 逆运动学、关节角计算、角度 UDP 报文和轨迹发送接口。上位机只需要调用本文件中的 `NUCControl` 方法。 |
| `__init__.py` | 对外导出 `NUCControl`、`ArmConfig` 等核心接口。 |
| `simulator.py` | 文本离线仿真，不属于控制核心。 |
| `mujoco_simulator.py` | MuJoCo 物理仿真，不属于控制核心。 |
| `test_arm_control.py` | 自动化测试，不参与上位机运行。 |

上位机程序只需导入 `NUCControl`，不需要导入测试或仿真文件。

## 9. 仿真（独立运行）

仿真程序只用于观察轨迹，不会改变核心接口，也不会自动连接真实 UDP：

```powershell
# 文本仿真
python -m NUCControl.simulator

# MuJoCo 物理仿真
conda run -n tianshou_env python -m NUCControl.mujoco_simulator --target 700 0 300
```

仿真中的坐标输入同样是 `x y z`，单位为 mm；它们调用的是核心 `NUCControl` 逆运动学，但不负责真实报文发送。

## 10. 测试（独立运行）

测试文件位于 `test_arm_control.py`，只验证配置、逆运动学、角度报文和轨迹接口。运行：

```powershell
python -m unittest discover -s NUCControl -v
```

测试不会打开 UDP 端口，也不会启动仿真窗口。
