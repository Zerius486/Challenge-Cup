"""NUCControl 的机械臂参数配置。

机械臂被限制在一个竖直平面内。位置计算使用三段有效连杆：
l1、l2，以及由 l3+l4 组成的末端总成。l4 可以绕自身轴 360° 滚转，
该滚转角单独映射到 J5；夹爪映射到 J6。
修改机械臂尺寸或安装参数时，优先编辑本文件。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import ClassVar, Optional, Tuple


# ==================== 基本机械参数 ====================
# 四段连杆长度，单位：mm，顺序为 l1、l2、l3、l4。
LINK_LENGTHS_MM = (419.9, 122.95, 124.0, 208.12)

# 四段连杆的初始绝对方向，单位：rad。
# 角度从车体前方的水平轴开始逆时针测量，向上为正。
# 这组值是“初始姿态”参考；逆解仍只把 l1、l2 的两个天顶角作为变量。
INITIAL_LINK_ANGLES_RAD = (
    math.pi / 2.0,
    math.pi / 2.0,
    0.0,
    0.0,
)

# l1、l2 的初始天顶角，单位：rad。
# 天顶角以竖直向上为 0，朝车体前方转动为正。
# 由四段连杆的默认绝对方向推导，避免两组默认参数出现不一致。
INITIAL_ZENITH_ANGLES_RAD = tuple(
    math.pi / 2.0 - angle for angle in INITIAL_LINK_ANGLES_RAD[:2]
)

# J1 固定朝向车体前方时为 0；如果机械臂安装方向不同，在这里修改。
BASE_YAW_RAD = 0.0

# l4 绕自身轴的初始滚转角，单位：rad，对应 J5。
INITIAL_WRIST_ROLL_RAD = 0.0

# l3、l4 的固定俯仰偏置，单位：rad；0 表示严格水平朝车体前方。
# 按当前简化模型应保持为 0。保留该参数是为了补偿安装时很小的固定倾角。
TOP_LINK_HORIZONTAL_ANGLE_RAD = 0.0


# ==================== 运动范围和解析求解参数 ====================
# 两个可动连杆的天顶角限位，单位：rad。
ZENITH_MIN_RAD = (math.radians(-90.0), math.radians(-150.0))
ZENITH_MAX_RAD = (math.radians(90.0), math.radians(150.0))

# 输入点允许偏离固定竖直平面的最大距离，单位：mm。
PLANE_TOLERANCE_MM = 1.0

# 位置逆解的数值误差容限，单位：mm。
POSITION_TOLERANCE_MM = 1.0e-5

# 判断两杆边界可达性时使用的距离容限，单位：mm。
REACH_TOLERANCE_MM = 1.0e-7

# 由天顶角换算出的 J1~J5 软件限位，单位：rad。
JOINT_MIN_RAD = (
    math.radians(-170.0),
    math.radians(-90.0),
    math.radians(-150.0),
    math.radians(-150.0),
    math.radians(-180.0),
)
JOINT_MAX_RAD = (
    math.radians(170.0),
    math.radians(90.0),
    math.radians(150.0),
    math.radians(150.0),
    math.radians(180.0),
)


# ==================== 电机和通信参数 ====================
ARM_DIRECTION = 1.0
JOINT_NODE_IDS = (11, 12, 13, 14, 15, 16)
EROB_ENCODER_COUNTS_PER_REV = 524288.0
EROB_ENCODER_MIDPOINT = 262144.0
EROB_PROFILE_VELOCITY_COUNTS_S = 10000.0
ROBSTRIDE_PROFILE_SPEED_RAD_S = 1.0
ROBSTRIDE_J5_MODEL = 1
ROBSTRIDE_J6_MODEL = 1
COMMAND_TIMEOUT_MS = 3000
ARM_OPTION = 0x8000


# ==================== 夹爪参数 ====================
GRIPPER_MIN_RAD = -1.5
GRIPPER_MAX_RAD = 1.5
GRIPPER_MAX_CURRENT_MA = 20000


@dataclass(frozen=True)
class ArmConfig:
    """一套完整的平面二连杆机械臂配置。"""

    # 可填写三段有效长度 (l1,l2,l3+l4)，也可填写四个物理长度 (l1,l2,l3,l4)。
    link_lengths_mm: Tuple[float, ...] = LINK_LENGTHS_MM
    initial_zenith_angles_rad: Tuple[float, float] = INITIAL_ZENITH_ANGLES_RAD
    base_yaw_rad: float = BASE_YAW_RAD
    top_link_horizontal_angle_rad: float = TOP_LINK_HORIZONTAL_ANGLE_RAD
    zenith_min_rad: Tuple[float, float] = ZENITH_MIN_RAD
    zenith_max_rad: Tuple[float, float] = ZENITH_MAX_RAD
    plane_tolerance_mm: float = PLANE_TOLERANCE_MM
    joint_min_rad: Tuple[float, float, float, float, float] = JOINT_MIN_RAD
    joint_max_rad: Tuple[float, float, float, float, float] = JOINT_MAX_RAD
    arm_direction: float = ARM_DIRECTION
    erob_encoder_counts_per_rev: float = EROB_ENCODER_COUNTS_PER_REV
    erob_encoder_midpoint: float = EROB_ENCODER_MIDPOINT
    erob_profile_velocity_counts_s: float = EROB_PROFILE_VELOCITY_COUNTS_S
    robstride_profile_speed_rad_s: float = ROBSTRIDE_PROFILE_SPEED_RAD_S
    robstride_j5_model: int = ROBSTRIDE_J5_MODEL
    robstride_j6_model: int = ROBSTRIDE_J6_MODEL
    joint_node_ids: Tuple[int, int, int, int, int, int] = JOINT_NODE_IDS
    command_timeout_ms: int = COMMAND_TIMEOUT_MS
    arm_option: int = ARM_OPTION
    gripper_min_rad: float = GRIPPER_MIN_RAD
    gripper_max_rad: float = GRIPPER_MAX_RAD
    gripper_max_current_ma: int = GRIPPER_MAX_CURRENT_MA
    # l4 绕自身轴的初始滚转角，单位：rad，对应 J5。
    initial_wrist_roll_rad: float = INITIAL_WRIST_ROLL_RAD
    # 若需要直接按四段连杆绝对方向设置上电姿态，可在这里填写四个角度。
    # 留空时由 initial_zenith_angles_rad 和固定的 l3/l4 方向自动推导。
    initial_link_angles_rad: Optional[Tuple[float, float, float, float]] = None
    position_tolerance_mm: float = POSITION_TOLERANCE_MM
    reach_tolerance_mm: float = REACH_TOLERANCE_MM

    CONFIG_NAME: ClassVar[str] = "平面二连杆机械臂默认配置"

    def __post_init__(self) -> None:
        """冻结由列表或生成器传入的序列参数。"""

        sequence_fields = (
            "link_lengths_mm",
            "initial_zenith_angles_rad",
            "zenith_min_rad",
            "zenith_max_rad",
            "joint_min_rad",
            "joint_max_rad",
            "joint_node_ids",
            "initial_link_angles_rad",
        )
        for field_name in sequence_fields:
            value = getattr(self, field_name)
            if value is None or isinstance(value, (str, bytes)):
                continue
            try:
                normalized = tuple(value)
            except TypeError:
                # 具体的类型错误交给 validate() 统一转换为中文配置错误。
                continue
            object.__setattr__(self, field_name, normalized)

    @property
    def initial_joint_angles_rad(self) -> Tuple[float, float, float, float, float]:
        """根据初始姿态计算 J1~J5 初始关节角。"""

        first_absolute, second_absolute, third_absolute, fourth_absolute = (
            float(value) for value in self.initial_link_orientations_rad
        )

        def relative_angle(first: float, second: float) -> float:
            return math.atan2(math.sin(first - second), math.cos(first - second))

        return (
            float(self.base_yaw_rad),
            first_absolute,
            relative_angle(second_absolute, first_absolute),
            relative_angle(third_absolute, second_absolute),
            float(self.initial_wrist_roll_rad),
        )

    @property
    def effective_link_lengths_mm(self) -> Tuple[float, float, float]:
        """返回逆解使用的三段有效长度。"""

        lengths = tuple(float(value) for value in self.link_lengths_mm)
        if len(lengths) == 3:
            return lengths
        return lengths[0], lengths[1], lengths[2] + lengths[3]

    @property
    def initial_link_orientations_rad(self) -> Tuple[float, float, float, float]:
        """返回四段连杆的初始绝对方向。"""

        if self.initial_link_angles_rad is not None:
            return tuple(float(value) for value in self.initial_link_angles_rad)
        first, second = (float(value) for value in self.initial_zenith_angles_rad)
        first_absolute = math.pi / 2.0 - first
        second_absolute = math.pi / 2.0 - second
        return (
            first_absolute,
            second_absolute,
            float(self.top_link_horizontal_angle_rad),
            float(self.top_link_horizontal_angle_rad),
        )

    @property
    def resolved_initial_zenith_angles_rad(self) -> Tuple[float, float]:
        """返回当前初始姿态对应的两个有效天顶角。"""

        first, second = self.initial_link_orientations_rad[:2]
        return (
            math.atan2(math.sin(math.pi / 2.0 - first), math.cos(math.pi / 2.0 - first)),
            math.atan2(math.sin(math.pi / 2.0 - second), math.cos(math.pi / 2.0 - second)),
        )

    @property
    def initial_pitch_joint_angles_rad(self) -> Tuple[float, float, float, float]:
        """返回四个共面俯仰关节的初始相对角度。"""

        return self.initial_joint_angles_rad[1:]

    @property
    def initial_angles_rad(self) -> Tuple[float, float, float, float]:
        """兼容旧配置名，返回四段连杆的初始绝对方向。"""

        return self.initial_link_orientations_rad

    def validate(self) -> None:
        """检查配置长度、角度、限位和通信参数。"""

        try:
            lengths = tuple(self.link_lengths_mm)
            initial_zenith = tuple(self.initial_zenith_angles_rad)
            zenith_min = tuple(self.zenith_min_rad)
            zenith_max = tuple(self.zenith_max_rad)
            joint_min = tuple(self.joint_min_rad)
            joint_max = tuple(self.joint_max_rad)
            node_ids = tuple(self.joint_node_ids)
        except TypeError as error:
            raise ValueError("配置中的数组参数必须是可迭代对象") from error

        def finite(value: object) -> bool:
            try:
                if isinstance(value, (bool, str, bytes)):
                    return False
                return math.isfinite(float(value))
            except (OverflowError, TypeError, ValueError):
                return False

        if len(lengths) not in (3, 4):
            raise ValueError("link_lengths_mm 必须包含三段有效长度或四个物理长度")
        if len(initial_zenith) != 2:
            raise ValueError("initial_zenith_angles_rad 必须包含两个天顶角")
        if len(zenith_min) != 2 or len(zenith_max) != 2:
            raise ValueError("天顶角限位必须包含两组数值")
        if len(joint_min) != 5 or len(joint_max) != 5:
            raise ValueError("关节限位必须包含 J1~J5 五组数值")
        if len(node_ids) != 6:
            raise ValueError("joint_node_ids 必须包含六个节点 ID")

        numeric_arrays = (
            ("link_lengths_mm", lengths),
            ("initial_zenith_angles_rad", initial_zenith),
            ("zenith_min_rad", zenith_min),
            ("zenith_max_rad", zenith_max),
            ("joint_min_rad", joint_min),
            ("joint_max_rad", joint_max),
        )
        for name, values in numeric_arrays:
            if any(not finite(value) for value in values):
                raise ValueError("{} 必须只包含有限数字".format(name))
        lengths = tuple(float(value) for value in lengths)
        initial_zenith = tuple(float(value) for value in initial_zenith)
        zenith_min = tuple(float(value) for value in zenith_min)
        zenith_max = tuple(float(value) for value in zenith_max)
        joint_min = tuple(float(value) for value in joint_min)
        joint_max = tuple(float(value) for value in joint_max)
        if any(value <= 0.0 for value in lengths):
            raise ValueError("link_lengths_mm 必须包含正数")

        scalar_values = (
            self.base_yaw_rad,
            self.top_link_horizontal_angle_rad,
            self.plane_tolerance_mm,
            self.arm_direction,
            self.erob_encoder_counts_per_rev,
            self.erob_encoder_midpoint,
            self.erob_profile_velocity_counts_s,
            self.robstride_profile_speed_rad_s,
            self.gripper_min_rad,
            self.gripper_max_rad,
            self.position_tolerance_mm,
            self.reach_tolerance_mm,
            self.initial_wrist_roll_rad,
        )
        if any(not finite(value) for value in scalar_values):
            raise ValueError("配置参数必须是有限数")
        top_pitch = float(self.top_link_horizontal_angle_rad)

        if self.initial_link_angles_rad is not None:
            try:
                initial_links = tuple(self.initial_link_angles_rad)
            except TypeError as error:
                raise ValueError("initial_link_angles_rad 必须是四个角度") from error
            if len(initial_links) != 4 or any(not finite(value) for value in initial_links):
                raise ValueError("initial_link_angles_rad 必须包含四个有限角度")
            initial_links = tuple(float(value) for value in initial_links)

            def angle_close(first: float, second: float) -> bool:
                difference = math.atan2(
                    math.sin(first - second), math.cos(first - second)
                )
                return abs(difference) <= 1.0e-6

            if any(not angle_close(initial_links[index], top_pitch) for index in (2, 3)):
                raise ValueError("l3 和 l4 的初始方向必须与固定水平方向一致")
        if any(not isinstance(node_id, int) or isinstance(node_id, bool) for node_id in node_ids):
            raise ValueError("节点 ID 必须是整数")

        for lower, upper in zip(zenith_min, zenith_max):
            if lower > upper:
                raise ValueError("天顶角限位无效")
        for lower, upper in zip(joint_min, joint_max):
            if lower > upper:
                raise ValueError("关节限位无效")
        for angle, lower, upper in zip(
            self.resolved_initial_zenith_angles_rad, zenith_min, zenith_max
        ):
            if float(angle) < lower or float(angle) > upper:
                raise ValueError("初始天顶角必须位于天顶角限位内")
        for angle, lower, upper in zip(self.initial_joint_angles_rad, joint_min, joint_max):
            if float(angle) < lower or float(angle) > upper:
                raise ValueError("初始关节角必须位于关节限位内")
        if float(self.plane_tolerance_mm) < 0.0 or float(self.arm_direction) == 0.0:
            raise ValueError("平面容差不能为负数，arm_direction 不能为 0")
        if float(self.position_tolerance_mm) <= 0.0 or float(self.reach_tolerance_mm) < 0.0:
            raise ValueError("逆解数值容差必须为正数或零")
        if float(self.erob_encoder_counts_per_rev) <= 0.0:
            raise ValueError("eRob 编码器每转计数必须为正数")
        if not 0.0 <= float(self.erob_encoder_midpoint) < float(self.erob_encoder_counts_per_rev):
            raise ValueError("eRob 编码器中位超出计数范围")
        if (
            float(self.erob_profile_velocity_counts_s) <= 0.0
            or float(self.robstride_profile_speed_rad_s) <= 0.0
        ):
            raise ValueError("电机位置速度必须为正数")
        if (
            not isinstance(self.command_timeout_ms, int)
            or isinstance(self.command_timeout_ms, bool)
            or not 20 <= self.command_timeout_ms <= 5000
        ):
            raise ValueError("command_timeout_ms 必须在 20~5000 ms 之间")
        if (
            not isinstance(self.arm_option, int)
            or isinstance(self.arm_option, bool)
            or not 0 <= self.arm_option <= 0xFFFF
            or (self.arm_option & 0x7FFF) != 0
        ):
            raise ValueError("arm_option 必须是 uint16")
        if float(self.gripper_min_rad) > float(self.gripper_max_rad):
            raise ValueError("夹爪角度限位无效")
        if (
            not isinstance(self.gripper_max_current_ma, int)
            or isinstance(self.gripper_max_current_ma, bool)
            or not 1 <= self.gripper_max_current_ma <= 20000
        ):
            raise ValueError("夹爪最大电流必须在 1~20000 mA 之间")
        if any(not 1 <= node_id <= 127 for node_id in node_ids):
            raise ValueError("节点 ID 必须在 1~127 之间")
        if (
            not isinstance(self.robstride_j5_model, int)
            or isinstance(self.robstride_j5_model, bool)
            or not isinstance(self.robstride_j6_model, int)
            or isinstance(self.robstride_j6_model, bool)
            or not 0 <= self.robstride_j5_model <= 6
            or not 0 <= self.robstride_j6_model <= 6
        ):
            raise ValueError("RobStride 型号必须在 RS00~RS06 之间")


# 默认实例显式采用四段连杆初始绝对方向；修改配置文件中的
# INITIAL_LINK_ANGLES_RAD 后，NUCControl() 会立即使用新的上电姿态。
DEFAULT_CONFIG = ArmConfig(initial_link_angles_rad=INITIAL_LINK_ANGLES_RAD)


__all__ = [
    "ArmConfig",
    "DEFAULT_CONFIG",
    "LINK_LENGTHS_MM",
    "INITIAL_LINK_ANGLES_RAD",
    "INITIAL_ZENITH_ANGLES_RAD",
    "BASE_YAW_RAD",
    "TOP_LINK_HORIZONTAL_ANGLE_RAD",
    "ZENITH_MIN_RAD",
    "ZENITH_MAX_RAD",
    "POSITION_TOLERANCE_MM",
    "REACH_TOLERANCE_MM",
    "PLANE_TOLERANCE_MM",
    "JOINT_MIN_RAD",
    "JOINT_MAX_RAD",
    "ARM_DIRECTION",
    "JOINT_NODE_IDS",
    "EROB_ENCODER_COUNTS_PER_REV",
    "EROB_ENCODER_MIDPOINT",
    "EROB_PROFILE_VELOCITY_COUNTS_S",
    "ROBSTRIDE_PROFILE_SPEED_RAD_S",
    "ROBSTRIDE_J5_MODEL",
    "ROBSTRIDE_J6_MODEL",
    "COMMAND_TIMEOUT_MS",
    "ARM_OPTION",
    "GRIPPER_MIN_RAD",
    "GRIPPER_MAX_RAD",
    "GRIPPER_MAX_CURRENT_MA",
    "INITIAL_WRIST_ROLL_RAD",
]
