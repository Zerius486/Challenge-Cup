"""机械臂离线仿真。

运行：
    python -m NUCControl.simulator

输入目标坐标 ``x y z``（单位 mm）后，程序会把轨迹分段打印出来，
每一段都模拟“发送关节角度报文 -> 收到完成信号 -> 发送下一段”。
该文件不会打开网口，也不会驱动真实电机。
"""

from __future__ import annotations

import argparse
import time
from typing import Optional, Sequence, Tuple

from .arm_control import ArmControlError, NUCControl, Point, TrajectoryFrame


class ArmSimulator:
    """使用 NUCControl 运动学的虚拟机械臂。"""

    def __init__(self, arm: Optional[NUCControl] = None) -> None:
        self.arm = NUCControl() if arm is None else arm
        self.position = self.arm.forward_kinematics()
        self.last_frame: Optional[TrajectoryFrame] = None

    def send_segment(self, frame: TrajectoryFrame, index: int) -> None:
        """模拟发送一段关节角度，并把虚拟机械臂更新到该位置。"""

        self.position = frame.point
        self.last_frame = frame
        print(
            "第 {:02d} 段: x={:8.2f}, y={:8.2f}, z={:8.2f} mm | "
            "天顶角=({:+.3f}, {:+.3f}) rad".format(
                index,
                frame.point.x_mm,
                frame.point.y_mm,
                frame.point.z_mm,
                frame.ik.zenith_angles_rad[0],
                frame.ik.zenith_angles_rad[1],
            )
        )
        angles = frame.ik.joint_angles_rad + (frame.gripper_rad,)
        print("    角度 J1~J6(rad): " + ", ".join("{:+.5f}".format(value) for value in angles))

    def move_to(
        self,
        target: Sequence[float],
        *,
        steps: int = 20,
        wrist_roll_rad: Optional[float] = None,
        gripper_rad: Optional[float] = None,
        interval_s: float = 0.05,
    ) -> Point:
        """分段模拟移动到目标坐标，并返回最终位置。"""

        frames = self.arm.trajectory(
            target,
            start=self.position,
            steps=steps,
            wrist_roll_rad=wrist_roll_rad,
            gripper_rad=gripper_rad,
            update_state=True,
        )
        for index, frame in enumerate(frames, 1):
            self.send_segment(frame, index)
            if index < len(frames):
                # 这里立即返回 True，表示模拟收到了单片机完成信号。
                print("    [仿真] 收到 UCP 完成信号，继续下一段")
            if interval_s > 0:
                time.sleep(interval_s)
        return self.position


def _parse_target(text: str) -> Tuple[float, float, float]:
    values = tuple(float(value) for value in text.replace(",", " ").split())
    if len(values) != 3:
        raise ValueError("请输入三个坐标值，例如：700 0 300")
    return values


def interactive(steps: int = 20, interval_s: float = 0.05) -> None:
    """启动交互式仿真。输入 q 退出。"""

    simulator = ArmSimulator()
    print("机械臂仿真已启动，坐标单位为 mm。输入 x y z，输入 q 退出。")
    print("当前末端位置：", simulator.position.as_tuple())
    while True:
        try:
            text = input("目标坐标 x y z > ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            return
        if text.lower() in {"q", "quit", "exit"}:
            return
        try:
            target = _parse_target(text)
            final_point = simulator.move_to(
                target, steps=steps, interval_s=interval_s
            )
            print("已到达：", final_point.as_tuple())
        except (ArmControlError, ValueError) as error:
            print("目标无效：", error)


def main() -> None:
    parser = argparse.ArgumentParser(description="NUCControl 机械臂离线仿真")
    parser.add_argument("--steps", type=int, default=20, help="每次移动的轨迹段数")
    parser.add_argument("--interval", type=float, default=0.05, help="段间显示间隔（秒）")
    args = parser.parse_args()
    interactive(args.steps, args.interval)


if __name__ == "__main__":
    main()
