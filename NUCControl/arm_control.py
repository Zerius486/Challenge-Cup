"""平面二连杆机械臂的解析逆运动学和电机指令生成。"""

from __future__ import annotations

import math
import struct
import zlib
from dataclasses import dataclass, replace
from typing import Callable, ClassVar, Iterator, List, Optional, Sequence, Tuple, Union, cast

from .config import ArmConfig, DEFAULT_CONFIG


Number = Union[int, float]
Coordinate = Union[Sequence[Number], "Point"]
PI = math.pi
TWO_PI = 2.0 * PI
JOINT_COUNT = 5
MOTOR_COUNT = 6
JOINT_ANGLE_MESSAGE_TYPE = 0x12
JOINT_ANGLE_COUNT = 6


class ArmControlError(ValueError):
    """机械臂输入、配置或电机指令无效时抛出的异常。"""


class UnreachableTargetError(ArmControlError):
    """目标点不满足平面二连杆几何约束时抛出的异常。"""

    def __init__(self, result: "IKResult") -> None:
        self.result = result
        super().__init__(
            "目标不可达：平面误差 {:.3f} mm，位置误差 {:.3f} mm".format(
                result.plane_error_mm, result.position_error_mm
            )
        )


@dataclass(frozen=True)
class Point:
    """车体坐标系中的末端坐标，单位为毫米。"""

    x_mm: float
    y_mm: float
    z_mm: float

    def as_tuple(self) -> Tuple[float, float, float]:
        """以 ``(x, y, z)`` 元组返回坐标。"""

        return (self.x_mm, self.y_mm, self.z_mm)

    def __iter__(self) -> Iterator[float]:
        return iter(self.as_tuple())


@dataclass(frozen=True)
class IKResult:
    """一次二连杆解析逆解的结果。"""

    status: str
    zenith_angles_rad: Tuple[float, float]
    joint_angles_rad: Tuple[float, float, float, float, float]
    position_error_mm: float
    plane_error_mm: float
    branch: int = 0

    @property
    def ok(self) -> bool:
        """判断逆解是否成功。"""

        return self.status == "ok"

    @property
    def joints_rad(self) -> Tuple[float, float, float, float, float]:
        """兼容旧接口的 J1~J5 角度属性。"""

        return self.joint_angles_rad

    @property
    def angles_rad(self) -> Tuple[float, float, float, float, float]:
        """返回 J1~J5 角度。"""

        return self.joint_angles_rad


@dataclass(frozen=True)
class MotorCommand:
    """一条 MainCode ``MOTOR_COMMAND`` 电机指令。"""

    joint_index: int
    joint_name: str
    can_bus: int
    driver: str
    node_id: int
    operation: str
    position: float
    velocity: float
    torque: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    timeout_ms: int = 3000
    options: int = 0x8000
    model: Optional[int] = None
    current_ma: Optional[int] = None

    OPERATION_CODES: ClassVar[dict] = {
        "disable": 0,
        "enable": 1,
        "stop": 2,
        "zero": 3,
        "position": 4,
        "velocity": 5,
        "torque": 6,
        "impedance": 7,
        "fault-reset": 8,
    }

    def __post_init__(self) -> None:
        if (
            not isinstance(self.joint_index, int)
            or isinstance(self.joint_index, bool)
            or not 0 <= self.joint_index < MOTOR_COUNT
        ):
            raise ArmControlError("joint_index 必须在 0~5 之间")
        if not isinstance(self.joint_name, str) or not self.joint_name:
            raise ArmControlError("joint_name 必须是非空字符串")
        if not isinstance(self.driver, str) or self.driver not in ("erob", "robstride"):
            raise ArmControlError("driver 必须是 erob 或 robstride")
        expected_bus = 1 if self.driver == "erob" else 2
        if (
            not isinstance(self.can_bus, int)
            or isinstance(self.can_bus, bool)
            or self.can_bus != expected_bus
        ):
            raise ArmControlError("驱动器和 CAN 总线不匹配")
        if not isinstance(self.operation, str) or self.operation not in self.OPERATION_CODES:
            raise ArmControlError("不支持的电机操作")
        if (
            not isinstance(self.node_id, int)
            or isinstance(self.node_id, bool)
            or not 1 <= self.node_id <= 127
        ):
            raise ArmControlError("node_id 必须在 1~127 之间")
        if (
            not isinstance(self.timeout_ms, int)
            or isinstance(self.timeout_ms, bool)
            or not 20 <= self.timeout_ms <= 5000
        ):
            raise ArmControlError("timeout_ms 必须在 20~5000 ms 之间")
        if (
            not isinstance(self.options, int)
            or isinstance(self.options, bool)
            or not 0 <= self.options <= 0xFFFF
        ):
            raise ArmControlError("options 必须是 uint16")
        if self.driver == "erob" and (self.options & 0x7FFF) != 0:
            raise ArmControlError("eRob 的 options 只能保留 ARM 位")
        if self.driver == "robstride" and (self.options & 0x7FF8) != 0:
            raise ArmControlError("RobStride 的 options 含有未定义保留位")
        try:
            finite_values = all(
                math.isfinite(float(value))
                for value in (self.position, self.velocity, self.torque, self.kp, self.kd)
            )
        except (OverflowError, TypeError, ValueError):
            finite_values = False
        if not finite_values:
            raise ArmControlError("电机指令中的浮点数必须是有限数")
        if self.model is not None and (
            not isinstance(self.model, int)
            or isinstance(self.model, bool)
            or not 0 <= self.model <= 6
        ):
            raise ArmControlError("RobStride 型号必须在 RS00~RS06 之间")
        if (
            self.driver == "robstride"
            and self.model is not None
            and self.model != (self.options & 0x0007)
        ):
            raise ArmControlError("RobStride 型号与 options 不一致")
        if self.current_ma is not None and (
            not isinstance(self.current_ma, int)
            or isinstance(self.current_ma, bool)
            or not 1 <= self.current_ma <= 20000
        ):
            raise ArmControlError("current_ma 必须在 1~20000 mA 之间")

    @property
    def operation_code(self) -> int:
        """返回 MainCode 使用的操作编号。"""

        return self.OPERATION_CODES[self.operation]

    def as_dict(self) -> dict:
        """返回适合记录或 JSON 序列化的字典。"""

        return {
            "joint": self.joint_name,
            "joint_index": self.joint_index,
            "can_bus": self.can_bus,
            "driver": self.driver,
            "node_id": self.node_id,
            "operation": self.operation,
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "kp": self.kp,
            "kd": self.kd,
            "timeout_ms": self.timeout_ms,
            "options": self.options,
            "model": self.model,
            "current_ma": self.current_ma,
        }

    def udp_payload(self) -> bytes:
        """打包为 MainCode 的 28 字节小端电机负载。"""

        return struct.pack(
            "<BBBBfffffHH",
            self.can_bus,
            1 if self.driver == "erob" else 2,
            self.node_id,
            self.operation_code,
            float(self.position),
            float(self.velocity),
            float(self.torque),
            float(self.kp),
            float(self.kd),
            self.timeout_ms,
            self.options,
        )

    def udp_packet(self, sequence: int) -> bytes:
        """构造完整的 v1 MOTOR_COMMAND UDP 数据报。"""

        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or not 0 <= sequence <= 0xFFFFFFFF
        ):
            raise ArmControlError("sequence 必须是 uint32")
        payload = self.udp_payload()
        header = struct.pack("<2sBBHHI", b"UW", 1, 0x10, 0, len(payload), sequence)
        body = header + payload
        return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


def pack_joint_angle_packet(
    joint_angles_rad: Sequence[Number], sequence: int
) -> bytes:
    """将 J1~J6 角度打包为一个角度专用 UDP 数据报。"""

    if (
        not isinstance(sequence, int)
        or isinstance(sequence, bool)
        or not 0 <= sequence <= 0xFFFFFFFF
    ):
        raise ArmControlError("sequence 必须是 uint32")
    try:
        angles = tuple(float(value) for value in joint_angles_rad)
    except (TypeError, ValueError, OverflowError) as error:
        raise ArmControlError("joint_angles_rad 必须包含 6 个角度") from error
    if len(angles) != JOINT_ANGLE_COUNT or not all(math.isfinite(value) for value in angles):
        raise ArmControlError("joint_angles_rad 必须包含 6 个有限角度")
    payload = struct.pack("<6f", *angles)
    header = struct.pack(
        "<2sBBHHI", b"UW", 1, JOINT_ANGLE_MESSAGE_TYPE, 0, len(payload), sequence
    )
    body = header + payload
    return body + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF)


@dataclass(frozen=True)
class TrajectoryFrame:
    """一个笛卡尔轨迹采样点和对应的电机指令。"""

    point: Point
    ik: IKResult
    commands: Tuple[MotorCommand, ...]
    gripper_rad: float = 0.0

    @property
    def motor_commands(self) -> Tuple[MotorCommand, ...]:
        return self.commands


def _number(value: Number) -> float:
    """把输入转换成有限浮点数。"""

    try:
        result = float(value)
    except (OverflowError, TypeError, ValueError) as error:
        raise ArmControlError("坐标和角度必须是数字") from error
    if not math.isfinite(result):
        raise ArmControlError("坐标和角度必须是有限数")
    return result


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def _wrap_angle(value: float) -> float:
    """把角度归一化到 [-pi, pi]，并避免超大角度的长循环。"""

    wrapped = math.fmod(value + PI, TWO_PI)
    if wrapped < 0.0:
        wrapped += TWO_PI
    wrapped -= PI
    # 让正 pi 保持为正值，便于在对称限位中稳定选支。
    if wrapped == -PI and value > 0.0:
        return PI
    return wrapped


def _angle_distance(first: float, second: float) -> float:
    return abs(_wrap_angle(first - second))


def _minimum_jerk_progress(ratio: float) -> float:
    """五次最小 jerk 时间缩放，输入和输出范围均为 0~1。"""

    return ratio * ratio * ratio * (10.0 - 15.0 * ratio + 6.0 * ratio * ratio)


def _within(value: float, lower: float, upper: float, tolerance: float = 1.0e-6) -> bool:
    return lower - tolerance <= value <= upper + tolerance


class NUCControl:
    """固定竖直平面、固定末端方向的平面二连杆控制器。

    l3 和 l4 始终保持水平，因此位置逆解只计算 l1、l2 的两个天顶角。
    天顶角以竖直向上为 0，朝车体前方转动为正。求解完成后，控制器会把
    两个绝对杆角转换为 MainCode 所需的 J1~J5 相对关节角。
    """

    def __init__(
        self,
        config: Optional[ArmConfig] = None,
        current_zenith_angles_rad: Optional[Sequence[Number]] = None,
    ) -> None:
        self.config = DEFAULT_CONFIG if config is None else config
        try:
            self.config.validate()
        except (TypeError, ValueError) as error:
            raise ArmControlError(str(error)) from error
        if current_zenith_angles_rad is None:
            current_zenith_angles_rad = self.config.resolved_initial_zenith_angles_rad
        self.current_zenith_angles_rad = self._validate_zenith(
            current_zenith_angles_rad, check_limits=True
        )
        self.current_wrist_roll_rad = _wrap_angle(self.config.initial_wrist_roll_rad)
        self.current_gripper_rad = _clamp(
            0.0, self.config.gripper_min_rad, self.config.gripper_max_rad
        )

    def _validate_zenith(
        self, angles: Sequence[Number], *, check_limits: bool = False
    ) -> Tuple[float, float]:
        """校验并规范两个天顶角。"""

        try:
            values = tuple(angles)
        except TypeError as error:
            raise ArmControlError("天顶角必须包含两个数值") from error
        if len(values) != 2:
            raise ArmControlError("天顶角必须包含 l1、l2 两个数值")
        result = (_number(values[0]), _number(values[1]))
        if check_limits and not all(
            _within(value, lower, upper)
            for value, lower, upper in zip(
                result, self.config.zenith_min_rad, self.config.zenith_max_rad
            )
        ):
            raise ArmControlError("天顶角超出配置限位")
        return result

    @staticmethod
    def _point(
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
    ) -> Point:
        if y is None and z is None:
            if isinstance(coordinate, Point):
                values = coordinate.as_tuple()
            else:
                try:
                    values = tuple(cast(Sequence[Number], coordinate))
                except TypeError as error:
                    raise ArmControlError("坐标必须包含 x、y、z 三个数值") from error
            if len(values) != 3:
                raise ArmControlError("坐标必须包含 x、y、z 三个数值")
            return Point(*(_number(value) for value in values))
        if y is None or z is None:
            raise ArmControlError("x、y、z 三个坐标必须同时提供")
        return Point(
            _number(cast(Number, coordinate)),
            _number(y),
            _number(z),
        )

    def _plane_coordinates(self, point: Point) -> Tuple[float, float]:
        """把车体坐标转换为固定平面内的前向和横向坐标。"""

        yaw = self.config.base_yaw_rad
        forward = point.x_mm * math.cos(yaw) + point.y_mm * math.sin(yaw)
        lateral = -point.x_mm * math.sin(yaw) + point.y_mm * math.cos(yaw)
        return forward, lateral

    def project_to_plane(self, coordinate: Coordinate, y: Optional[Number] = None,
                         z: Optional[Number] = None) -> Tuple[float, float, float]:
        """把车体坐标转换为固定平面的前向、横向和高度坐标。"""

        point = self._point(coordinate, y, z)
        forward, lateral = self._plane_coordinates(point)
        return forward, lateral, point.z_mm

    def _fixed_top_offset(self) -> Tuple[float, float]:
        """计算固定的 l3+l4 偏移（前向距离、竖直高度）。"""

        total = self.config.effective_link_lengths_mm[2]
        # top_link_horizontal_angle_rad 是固定俯仰角，0 表示严格水平朝前。
        pitch = self.config.top_link_horizontal_angle_rad
        return total * math.cos(pitch), total * math.sin(pitch)

    # 兼容早期代码中的私有方法名。
    _horizontal_offset = _fixed_top_offset

    def link_absolute_angles(
        self, zenith_angles_rad: Optional[Sequence[Number]] = None
    ) -> Tuple[float, float, float, float]:
        """返回 l1~l4 相对于车体前方水平轴的绝对方向。"""

        zenith = (
            self.current_zenith_angles_rad
            if zenith_angles_rad is None
            else self._validate_zenith(zenith_angles_rad)
        )
        first, second = zenith
        return (
            PI / 2.0 - first,
            PI / 2.0 - second,
            self.config.top_link_horizontal_angle_rad,
            self.config.top_link_horizontal_angle_rad,
        )

    def _joint_angles_from_zenith(
        self, zenith_first: float, zenith_second: float,
        wrist_roll_rad: Optional[Number] = None,
    ) -> Tuple[float, float, float, float, float]:
        first_absolute, second_absolute, third_absolute, _ = (
            self.link_absolute_angles((zenith_first, zenith_second))
        )
        wrist_roll = self.current_wrist_roll_rad if wrist_roll_rad is None else _number(wrist_roll_rad)
        return (
            self.config.base_yaw_rad,
            first_absolute,
            second_absolute - first_absolute,
            third_absolute - second_absolute,
            _wrap_angle(wrist_roll),
        )

    def _joint_limits_ok(self, joints: Sequence[float]) -> bool:
        return all(
            _within(value, lower, upper)
            for value, lower, upper in zip(
                joints, self.config.joint_min_rad, self.config.joint_max_rad
            )
        )

    def forward_kinematics(
        self, zenith_angles_rad: Optional[Sequence[Number]] = None
    ) -> Point:
        """根据两个天顶角计算末端坐标。"""

        return self.forward_link_points(zenith_angles_rad)[-1]

    def forward_link_points(
        self, zenith_angles_rad: Optional[Sequence[Number]] = None
    ) -> Tuple[Point, Point, Point, Point]:
        """返回四段连杆末端的位置，用于检查固定的上部姿态。"""

        configured_lengths = tuple(self.config.link_lengths_mm)
        lengths = configured_lengths if len(configured_lengths) == 4 else self.config.effective_link_lengths_mm
        angles = self.link_absolute_angles(zenith_angles_rad)
        radial = 0.0
        height = 0.0
        points: List[Point] = []
        for length, angle in zip(lengths, angles):
            radial += length * math.cos(angle)
            height += length * math.sin(angle)
            points.append(
                Point(
                    radial * math.cos(self.config.base_yaw_rad),
                    radial * math.sin(self.config.base_yaw_rad),
                    height,
                )
            )
        # 对三长度配置，第三段已经是 l3+l4 总成；为兼容旧接口补一个重合点。
        if len(points) == 3:
            points.append(points[-1])
        return tuple(points)  # type: ignore[return-value]

    def inverse_kinematics(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
    ) -> IKResult:
        """用余弦定理求解目标坐标对应的两个天顶角。"""

        point = self._point(coordinate, y, z)
        forward, lateral = self._plane_coordinates(point)
        offset_radial, offset_height = self._fixed_top_offset()
        target_radial = forward - offset_radial
        target_height = point.z_mm - offset_height
        first_length, second_length = self.config.effective_link_lengths_mm[:2]
        distance = math.hypot(target_radial, target_height)
        seed = (
            self.current_zenith_angles_rad
            if seed_zenith_angles_rad is None
            else self._validate_zenith(seed_zenith_angles_rad, check_limits=True)
        )
        if elbow_branch is not None and (
            not isinstance(elbow_branch, int)
            or isinstance(elbow_branch, bool)
            or elbow_branch not in (0, 1)
        ):
            raise ArmControlError("elbow_branch 必须是 None、0 或 1")

        def failed(status: str = "unreachable") -> IKResult:
            current_point = self.forward_kinematics(seed)
            position_error = math.sqrt(
                (current_point.x_mm - point.x_mm) ** 2
                + (current_point.y_mm - point.y_mm) ** 2
                + (current_point.z_mm - point.z_mm) ** 2
            )
            return IKResult(
                status,
                seed,
                self._joint_angles_from_zenith(*seed, wrist_roll_rad),
                position_error,
                abs(lateral),
            )

        if abs(lateral) > self.config.plane_tolerance_mm:
            return failed()
        # 两杆的可达范围是 |l1-l2| <= d <= l1+l2。
        reach_tolerance = self.config.reach_tolerance_mm
        if distance < abs(first_length - second_length) - reach_tolerance:
            return failed()
        if distance > first_length + second_length + reach_tolerance:
            return failed()

        # 默认选择与当前姿态整体变化量最小的解。
        # 这里比较 J1~J5 的全部关节角，而不是只比较两个天顶角；
        # 这样会同时考虑 J2、J3、J4 的联动变化。
        seed_joints = self._joint_angles_from_zenith(*seed)
        candidates = []
        if distance <= reach_tolerance:
            # 只有两杆等长时，d=0 才有解；此时选取最接近当前姿态的对向姿态。
            if abs(first_length - second_length) > reach_tolerance:
                return failed()
            raw_candidates = (
                (seed[0], _wrap_angle(seed[0] + PI)),
                (seed[0], _wrap_angle(seed[0] - PI)),
            )
        else:
            direction = math.atan2(target_radial, target_height)
            first_cosine = (
                first_length * first_length
                + distance * distance
                - second_length * second_length
            ) / (2.0 * first_length * distance)
            first_offset = math.acos(_clamp(first_cosine, -1.0, 1.0))
            raw_candidates = []
            for sign in (1.0, -1.0):
                first_zenith = _wrap_angle(direction + sign * first_offset)
                remaining_radial = target_radial - first_length * math.sin(first_zenith)
                remaining_height = target_height - first_length * math.cos(first_zenith)
                second_zenith = _wrap_angle(
                    math.atan2(remaining_radial, remaining_height)
                )
                raw_candidates.append((first_zenith, second_zenith))

        for branch, (first_zenith, second_zenith) in enumerate(raw_candidates):
            if elbow_branch is not None and branch != elbow_branch:
                continue
            joints = self._joint_angles_from_zenith(
                first_zenith, second_zenith, wrist_roll_rad
            )
            if not _within(
                first_zenith,
                self.config.zenith_min_rad[0],
                self.config.zenith_max_rad[0],
            ):
                continue
            if not _within(
                second_zenith,
                self.config.zenith_min_rad[1],
                self.config.zenith_max_rad[1],
            ):
                continue
            if not self._joint_limits_ok(joints):
                continue
            total_joint_change = sum(
                _angle_distance(target_angle, current_angle) ** 2
                for target_angle, current_angle in zip(joints, seed_joints)
            )
            candidates.append((total_joint_change, branch, first_zenith, second_zenith, joints))

        if not candidates:
            return failed()
        _, branch, first_zenith, second_zenith, joints = min(
            candidates, key=lambda item: (item[0], item[1])
        )
        solved = (first_zenith, second_zenith)
        reached = self.forward_kinematics(solved)
        position_error = math.sqrt(
            (reached.x_mm - point.x_mm) ** 2
            + (reached.y_mm - point.y_mm) ** 2
            + (reached.z_mm - point.z_mm) ** 2
        )
        reached_forward, _ = self._plane_coordinates(reached)
        planar_position_error = math.hypot(
            reached_forward - forward, reached.z_mm - point.z_mm
        )
        if (
            not math.isfinite(position_error)
            or not math.isfinite(planar_position_error)
            or planar_position_error > self.config.position_tolerance_mm
        ):
            return failed("numeric_error")
        return IKResult("ok", solved, joints, position_error, abs(lateral), branch)

    def zenith_angles(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
    ) -> Tuple[float, float]:
        """返回目标点对应的两个天顶角。"""

        result = self.inverse_kinematics(
            coordinate, y, z, seed_zenith_angles_rad, elbow_branch, wrist_roll_rad
        )
        if not result.ok:
            raise UnreachableTargetError(result)
        return result.zenith_angles_rad

    def joint_angles(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
    ) -> Tuple[float, ...]:
        """返回目标点对应的 J1~J5 相对关节角。"""

        result = self.inverse_kinematics(
            coordinate, y, z, seed_zenith_angles_rad, elbow_branch, wrist_roll_rad
        )
        if not result.ok:
            raise UnreachableTargetError(result)
        return result.joint_angles_rad

    def _joint_command(self, index: int, position_rad: Number) -> MotorCommand:
        """把关节角转换为 MainCode 电机指令。"""

        angle = _number(position_rad)
        if not isinstance(index, int) or isinstance(index, bool) or not 0 <= index < MOTOR_COUNT:
            raise ArmControlError("关节索引必须在 0~5 之间")
        if index < JOINT_COUNT and not _within(
            angle, self.config.joint_min_rad[index], self.config.joint_max_rad[index]
        ):
            raise ArmControlError("关节角超出配置限位")
        if index < 4:
            counts = self.config.erob_encoder_midpoint + self.config.arm_direction * angle * (
                self.config.erob_encoder_counts_per_rev / TWO_PI
            )
            if not 0.0 <= counts < self.config.erob_encoder_counts_per_rev:
                raise ArmControlError("eRob 位置超出编码器计数范围")
            return MotorCommand(
                index,
                "J{}".format(index + 1),
                1,
                "erob",
                self.config.joint_node_ids[index],
                "position",
                int(math.floor(counts + 0.5)),
                self.config.erob_profile_velocity_counts_s,
                timeout_ms=self.config.command_timeout_ms,
                options=self.config.arm_option,
            )
        model = self.config.robstride_j5_model if index == 4 else self.config.robstride_j6_model
        if abs(angle) > 12.57:
            raise ArmControlError("RobStride 位置超出 MainCode 限制")
        return MotorCommand(
            index,
            "J{}".format(index + 1),
            2,
            "robstride",
            self.config.joint_node_ids[index],
            "position",
            self.config.arm_direction * angle,
            self.config.robstride_profile_speed_rad_s,
            timeout_ms=self.config.command_timeout_ms,
            options=self.config.arm_option | model,
            model=model,
        )

    def motor_commands(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        *,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
        update_state: bool = True,
    ) -> Tuple[MotorCommand, ...]:
        """输入三维坐标，返回 J1~J5（以及可选 J6）的电机指令。"""

        result = self.inverse_kinematics(
            coordinate, y, z, seed_zenith_angles_rad, elbow_branch, wrist_roll_rad
        )
        if not result.ok:
            raise UnreachableTargetError(result)
        commands = tuple(
            self._joint_command(index, angle)
            for index, angle in enumerate(result.joint_angles_rad)
        )
        if gripper_rad is not None:
            commands += (self.gripper_command(gripper_rad, gripper_current_ma),)
        if update_state:
            self.current_zenith_angles_rad = result.zenith_angles_rad
            if wrist_roll_rad is not None:
                self.current_wrist_roll_rad = _wrap_angle(_number(wrist_roll_rad))
            if gripper_rad is not None:
                self.current_gripper_rad = _number(gripper_rad)
        return commands

    @staticmethod
    def _validate_sequence(sequence: object) -> int:
        """校验并返回 UDP 协议使用的 32 位序号。"""

        if (
            not isinstance(sequence, int)
            or isinstance(sequence, bool)
            or not 0 <= sequence <= 0xFFFFFFFF
        ):
            raise ArmControlError("sequence 必须是 uint32")
        return sequence

    @staticmethod
    def packets_for_commands(
        commands: Sequence[MotorCommand], sequence: int
    ) -> Tuple[bytes, ...]:
        """把一组电机命令按递增序号打包成 UDP 数据报。"""

        sequence = NUCControl._validate_sequence(sequence)
        packets = []
        for offset, command in enumerate(commands):
            if not isinstance(command, MotorCommand):
                raise ArmControlError("commands 必须只包含 MotorCommand")
            packet_sequence = sequence + offset
            if packet_sequence > 0xFFFFFFFF:
                raise ArmControlError("命令数量导致 sequence 溢出")
            packets.append(command.udp_packet(packet_sequence))
        return tuple(packets)

    def motor_packets(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        *,
        sequence: Optional[int] = None,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
        update_state: bool = True,
    ) -> Tuple[bytes, ...]:
        """输入坐标并直接返回可发送给 MainCode 的 UDP 数据报。"""

        sequence = self._validate_sequence(sequence)
        commands = self.motor_commands(
            coordinate,
            y,
            z,
            seed_zenith_angles_rad=seed_zenith_angles_rad,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            gripper_current_ma=gripper_current_ma,
            update_state=update_state,
        )
        return self.packets_for_commands(commands, sequence)

    def joint_angle_values(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        *,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Number = 0.0,
        update_state: bool = True,
    ) -> Tuple[float, float, float, float, float, float]:
        """输入末端坐标，返回按 J1~J6 排列的六个关节角（rad）。"""

        result = self.inverse_kinematics(
            coordinate, y, z, seed_zenith_angles_rad, elbow_branch, wrist_roll_rad
        )
        if not result.ok:
            raise UnreachableTargetError(result)
        gripper = _number(gripper_rad)
        if not self.config.gripper_min_rad <= gripper <= self.config.gripper_max_rad:
            raise ArmControlError("夹爪角度超出配置限位")
        values = result.joint_angles_rad + (gripper,)
        if update_state:
            self.current_zenith_angles_rad = result.zenith_angles_rad
            if wrist_roll_rad is not None:
                self.current_wrist_roll_rad = _wrap_angle(_number(wrist_roll_rad))
            self.current_gripper_rad = gripper
        return values

    def joint_angle_packet(
        self,
        joint_angles_rad: Sequence[Number],
        sequence: int,
    ) -> bytes:
        """将 J1~J6 角度打包为 0x12 角度 UDP 数据报。"""

        return pack_joint_angle_packet(joint_angles_rad, sequence)

    def angle_packet(
        self,
        coordinate: Coordinate,
        sequence: int,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
        *,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Number = 0.0,
        update_state: bool = True,
    ) -> bytes:
        """输入末端坐标，返回只包含 J1~J6 角度的 UDP 数据报。"""

        angles = self.joint_angle_values(
            coordinate,
            y,
            z,
            seed_zenith_angles_rad=seed_zenith_angles_rad,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            update_state=update_state,
        )
        return self.joint_angle_packet(angles, sequence)

    commands_for_coordinate = motor_commands
    commands_for_point = motor_commands
    get_motor_commands = motor_commands
    packets_for_coordinate = motor_packets

    def gripper_command(self, position_rad: Number, current_ma: int = 1000) -> MotorCommand:
        """生成 J6 夹爪位置指令。"""

        position = _number(position_rad)
        if not self.config.gripper_min_rad <= position <= self.config.gripper_max_rad:
            raise ArmControlError("夹爪角度超出配置限位")
        if (
            not isinstance(current_ma, int)
            or isinstance(current_ma, bool)
            or not 1 <= current_ma <= self.config.gripper_max_current_ma
        ):
            raise ArmControlError("夹爪电流必须是有效的整数 mA")
        return replace(self._joint_command(5, position), current_ma=current_ma)

    @staticmethod
    def _protocol_integer(value: Number, scale: float, name: str) -> int:
        scaled = _number(value) * scale
        result = int(math.floor(scaled + 0.5) if scaled >= 0.0 else math.ceil(scaled - 0.5))
        if not -2147483648 <= result <= 2147483647:
            raise ArmControlError("{} 超出 int32 协议范围".format(name))
        return result

    def format_position_command(
        self,
        coordinate: Coordinate,
        y: Optional[Number] = None,
        z: Optional[Number] = None,
    ) -> bytes:
        """格式化为固件接收的 ``PXYZ`` ASCII 指令。"""

        point = self._point(coordinate, y, z)
        values = tuple(
            self._protocol_integer(value, 100.0, "PXYZ 坐标")
            for value in point.as_tuple()
        )
        return "PXYZ:{:+07d},{:+07d},{:+07d},/".format(*values).encode("ascii")

    position_command = format_position_command

    def format_gripper_command(self, position_rad: Number, current_ma: int = 1000) -> bytes:
        """格式化为固件接收的 ``JZH`` ASCII 指令。"""

        position = _number(position_rad)
        if not self.config.gripper_min_rad <= position <= self.config.gripper_max_rad:
            raise ArmControlError("夹爪角度超出配置限位")
        if (
            not isinstance(current_ma, int)
            or isinstance(current_ma, bool)
            or not 1 <= current_ma <= self.config.gripper_max_current_ma
        ):
            raise ArmControlError("夹爪电流必须是有效的整数 mA")
        millirad = self._protocol_integer(position, 1000.0, "夹爪角度")
        return "JZH_{:+d}_{}/".format(millirad, current_ma).encode("ascii")

    def trajectory(
        self,
        target: Coordinate,
        *,
        start: Optional[Coordinate] = None,
        steps: int = 20,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
        update_state: bool = True,
        _include_commands: bool = True,
    ) -> Tuple[TrajectoryFrame, ...]:
        """在关节空间生成五次最小 jerk 轨迹。"""

        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 1:
            raise ArmControlError("steps 必须是正整数")
        target_point = self._point(target)
        seed = (
            self.current_zenith_angles_rad
            if seed_zenith_angles_rad is None
            else self._validate_zenith(seed_zenith_angles_rad, check_limits=True)
        )
        if start is None:
            start_zenith = seed
            start_point = self.forward_kinematics(start_zenith)
        else:
            start_point = self._point(start)
            start_result = self.inverse_kinematics(
                start_point,
                seed_zenith_angles_rad=seed,
                elbow_branch=elbow_branch,
            )
            if not start_result.ok:
                raise UnreachableTargetError(start_result)
            start_zenith = start_result.zenith_angles_rad

        _, start_lateral = self._plane_coordinates(start_point)
        _, target_lateral = self._plane_coordinates(target_point)
        if (
            abs(start_lateral) > self.config.plane_tolerance_mm
            or abs(target_lateral) > self.config.plane_tolerance_mm
        ):
            raise UnreachableTargetError(
                IKResult(
                    "unreachable",
                    start_zenith,
                    self._joint_angles_from_zenith(*start_zenith, wrist_roll_rad),
                    math.inf,
                    max(abs(start_lateral), abs(target_lateral)),
                )
            )

        # 先求一次终点逆解，之后整条轨迹沿关节空间插值，避免逐帧切换肘部解。
        target_result = self.inverse_kinematics(
            target_point,
            seed_zenith_angles_rad=start_zenith,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
        )
        if not target_result.ok:
            raise UnreachableTargetError(target_result)
        target_zenith = target_result.zenith_angles_rad
        start_wrist = self.current_wrist_roll_rad
        target_wrist = start_wrist if wrist_roll_rad is None else _wrap_angle(_number(wrist_roll_rad))
        start_gripper = self.current_gripper_rad
        target_gripper = start_gripper if gripper_rad is None else _number(gripper_rad)
        if not self.config.gripper_min_rad <= target_gripper <= self.config.gripper_max_rad:
            raise ArmControlError("夹爪角度超出配置限位")

        frames = []
        for sample in range(1, steps + 1):
            progress = _minimum_jerk_progress(sample / float(steps))
            zenith = tuple(
                start_angle + (target_angle - start_angle) * progress
                for start_angle, target_angle in zip(start_zenith, target_zenith)
            )
            wrist = _wrap_angle(start_wrist + _wrap_angle(target_wrist - start_wrist) * progress)
            joints = self._joint_angles_from_zenith(*zenith, wrist)
            point = self.forward_kinematics(zenith)
            result = IKResult(
                "ok",
                zenith,
                joints,
                0.0,
                0.0,
                target_result.branch,
            )
            gripper = start_gripper + (target_gripper - start_gripper) * progress
            if _include_commands:
                commands = tuple(
                    self._joint_command(index, angle)
                    for index, angle in enumerate(joints)
                )
                if gripper_rad is not None:
                    commands += (self.gripper_command(gripper, gripper_current_ma),)
            else:
                commands = ()
            frames.append(TrajectoryFrame(point, result, commands, gripper))

        if update_state:
            self.current_zenith_angles_rad = target_zenith
            if wrist_roll_rad is not None:
                self.current_wrist_roll_rad = target_wrist
            if gripper_rad is not None:
                self.current_gripper_rad = target_gripper
        return tuple(frames)

    plan_trajectory = trajectory
    calculate_trajectory = trajectory

    def trajectory_packets(
        self,
        target: Coordinate,
        sequence: int,
        *,
        start: Optional[Coordinate] = None,
        steps: int = 20,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
        update_state: bool = True,
    ) -> Tuple[bytes, ...]:
        """生成轨迹，并为每一帧生成一个只包含 J1~J6 角度的 UDP 报文。"""

        sequence = self._validate_sequence(sequence)
        if not isinstance(steps, int) or isinstance(steps, bool) or steps < 1:
            raise ArmControlError("steps 必须是正整数")
        if steps > 0xFFFFFFFF - sequence + 1:
            raise ArmControlError("轨迹命令数量导致 sequence 溢出")

        frames = self.trajectory(
            target,
            start=start,
            steps=steps,
            seed_zenith_angles_rad=seed_zenith_angles_rad,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            gripper_current_ma=gripper_current_ma,
            update_state=update_state,
            _include_commands=False,
        )
        packet_frames = []
        for offset, frame in enumerate(frames):
            angles = frame.ik.joint_angles_rad + (frame.gripper_rad,)
            packet_frames.append(self.joint_angle_packet(angles, sequence + offset))
        return tuple(packet_frames)

    def trajectory_segments(
        self,
        target: Coordinate,
        sequence: int = 0,
        *,
        start: Optional[Coordinate] = None,
        steps: int = 20,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
    ) -> Iterator[bytes]:
        """逐段生成轨迹报文；每次迭代就是一个位置的六关节角度报文。"""

        for packet in self.trajectory_packets(
            target,
            sequence,
            start=start,
            steps=steps,
            seed_zenith_angles_rad=seed_zenith_angles_rad,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            gripper_current_ma=gripper_current_ma,
            update_state=True,
        ):
            yield packet

    def send_trajectory(
        self,
        target: Coordinate,
        send_packets: Callable[[bytes], object],
        wait_complete: Callable[[], object],
        sequence: int = 0,
        *,
        start: Optional[Coordinate] = None,
        steps: int = 20,
        seed_zenith_angles_rad: Optional[Sequence[Number]] = None,
        elbow_branch: Optional[int] = None,
        wrist_roll_rad: Optional[Number] = None,
        gripper_rad: Optional[Number] = None,
        gripper_current_ma: int = 1000,
    ) -> int:
        """按“发送一段、等待完成、再发送下一段”的顺序发送角度轨迹。"""

        sent = 0
        segments = self.trajectory_segments(
            target,
            sequence,
            start=start,
            steps=steps,
            seed_zenith_angles_rad=seed_zenith_angles_rad,
            elbow_branch=elbow_branch,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            gripper_current_ma=gripper_current_ma,
        )
        for index, packet in enumerate(segments):
            send_packets(packet)
            sent += 1
            if index + 1 < steps and not wait_complete():
                break
        return sent

    calculate_motor_commands = motor_commands
    commands_for_target = motor_commands


# 兼容旧脚本中的常用类名。
ArmController = NUCControl
ArmKinematics = NUCControl
ArmKinematicsConfig = ArmConfig
ArmCommand = MotorCommand
ArmIKResult = IKResult


__all__ = [
    "ArmCommand",
    "ArmControlError",
    "ArmController",
    "ArmConfig",
    "ArmIKResult",
    "ArmKinematics",
    "ArmKinematicsConfig",
    "DEFAULT_CONFIG",
    "IKResult",
    "JOINT_ANGLE_COUNT",
    "JOINT_ANGLE_MESSAGE_TYPE",
    "MotorCommand",
    "NUCControl",
    "Point",
    "pack_joint_angle_packet",
    "TrajectoryFrame",
    "UnreachableTargetError",
]
