"""基于 MuJoCo 的机械臂三维物理仿真。

推荐运行环境：

    C:\\Users\\Zerius\\miniconda3\\envs\\tianshou_env\\python.exe \\
        -m NUCControl.mujoco_simulator

MuJoCo 负责刚体、关节和执行器，NUCControl 负责目标坐标的逆运动学。
输入目标坐标后，程序会通过位置执行器让虚拟机械臂逐段运动到目标位置。
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Optional, Sequence, Tuple

from .arm_control import ArmControlError, NUCControl, Point


def _parse_target(text: str) -> Tuple[float, float, float]:
    values = tuple(float(value) for value in text.replace(",", " ").split())
    if len(values) != 3:
        raise ValueError("请输入三个坐标值，例如：700 0 300")
    return values


def _xml_for_arm(arm: NUCControl) -> str:
    """生成三连杆加腕部滚转的 MJCF 模型。长度转换为米。"""

    l1, l2, l34 = (value / 1000.0 for value in arm.config.effective_link_lengths_mm)
    l3 = l34 * 0.6
    l4 = l34 - l3
    return f"""
<mujoco model="nuccontrol_arm">
  <option timestep="0.001" gravity="0 0 -9.81" integrator="Euler"/>
  <visual><headlight ambient="0.45 0.45 0.45" diffuse="0.8 0.8 0.8"/></visual>
  <asset>
    <texture name="floor_tex" type="2d" builtin="checker" width="512" height="512"
             rgb1="0.18 0.18 0.18" rgb2="0.25 0.25 0.25"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="8 8"/>
  </asset>
  <worldbody>
    <light pos="0 0 2.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" type="plane" pos="0 0 -0.08" size="3 3 0.1" material="floor_mat"/>
    <body name="base" pos="0 0 0">
      <geom type="cylinder" size="0.13 0.04" contype="0" conaffinity="0" rgba="0.15 0.15 0.18 1"/>
      <body name="link1">
        <joint name="joint1" type="hinge" axis="0 1 0" damping="5" armature="0.03"/>
        <geom type="capsule" fromto="0 0 0 0 0 {l1:.8f}" size="0.035" density="500" contype="0" conaffinity="0" rgba="0.15 0.45 0.85 1"/>
        <body name="link2" pos="0 0 {l1:.8f}">
          <joint name="joint2" type="hinge" axis="0 1 0" damping="5" armature="0.03"/>
          <geom type="capsule" fromto="0 0 0 0 0 {l2:.8f}" size="0.032" density="500" contype="0" conaffinity="0" rgba="0.15 0.65 0.35 1"/>
          <body name="link3" pos="0 0 {l2:.8f}">
            <joint name="joint3" type="hinge" axis="0 1 0" damping="5" armature="0.02"/>
            <geom type="capsule" fromto="0 0 0 0 0 {l3:.8f}" size="0.03" density="500" contype="0" conaffinity="0" rgba="0.95 0.55 0.12 1"/>
            <body name="link4" pos="0 0 {l3:.8f}">
              <joint name="wrist" type="hinge" axis="0 0 1" damping="2" armature="0.01"/>
              <geom type="capsule" fromto="0 0 0 0 0 {l4:.8f}" size="0.028" density="400" contype="0" conaffinity="0" rgba="0.85 0.2 0.2 1"/>
              <body name="gripper" pos="0 0 {l4:.8f}">
                <body name="finger_left" pos="0.05 0 0.055">
                  <joint name="finger_left_joint" type="slide" axis="1 0 0" range="-0.04 0" damping="2"/>
                  <geom type="box" size="0.012 0.025 0.055" contype="0" conaffinity="0" rgba="0.75 0.75 0.78 1"/>
                </body>
                <body name="finger_right" pos="-0.05 0 0.055">
                  <joint name="finger_right_joint" type="slide" axis="1 0 0" range="0 0.04" damping="2"/>
                  <geom type="box" size="0.012 0.025 0.055" contype="0" conaffinity="0" rgba="0.75 0.75 0.78 1"/>
                </body>
              </body>
            </body>
          </body>
        </body>
      </body>
    </body>
    <body name="ball" mocap="true" pos="0 0 0">
      <geom type="sphere" size="0.045" contype="0" conaffinity="0" rgba="0.08 0.28 0.95 1"/>
    </body>
  </worldbody>
  <actuator>
    <position name="motor1" joint="joint1" kp="25" kv="6" forcerange="-20 20" ctrlrange="-3.14 3.14"/>
    <position name="motor2" joint="joint2" kp="22" kv="5" forcerange="-20 20" ctrlrange="-6.28 6.28"/>
    <position name="motor3" joint="joint3" kp="18" kv="4" forcerange="-15 15" ctrlrange="-6.28 6.28"/>
    <position name="motor4" joint="wrist" kp="10" kv="2" forcerange="-8 8" ctrlrange="-6.28 6.28"/>
    <position name="finger_left_motor" joint="finger_left_joint" kp="20" kv="3" forcerange="-5 5" ctrlrange="-0.04 0"/>
    <position name="finger_right_motor" joint="finger_right_joint" kp="20" kv="3" forcerange="-5 5" ctrlrange="0 0.04"/>
  </actuator>
</mujoco>
"""


def _joint_targets(zenith_angles: Sequence[float], wrist_roll_rad: float = 0.0) -> Tuple[float, ...]:
    theta1, theta2 = zenith_angles
    # MuJoCo 的局部 z 轴初始朝上，因此这里直接使用天顶角。
    return theta1, theta2 - theta1, math.pi / 2.0 - theta2, wrist_roll_rad


def _set_pose(data, controls: Sequence[float], finger_left: float = 0.0,
              finger_right: float = 0.0) -> None:
    """直接设置虚拟机械臂姿态，避免演示过程中因重力慢慢下坠。"""

    for joint_name, value in zip(("joint1", "joint2", "joint3", "wrist"), controls):
        data.joint(joint_name).qpos[0] = float(value)
    data.joint("finger_left_joint").qpos[0] = float(finger_left)
    data.joint("finger_right_joint").qpos[0] = float(finger_right)
    data.qvel[:] = 0.0


def run(target: Sequence[float], *, steps: int = 60, interval_s: float = 0.02,
        wrist_roll_rad: float = 0.0) -> Point:
    """打开 MuJoCo 查看器并把机械臂移动到目标点。"""

    try:
        import mujoco
        import mujoco.viewer
    except ImportError as error:
        raise RuntimeError(
            "没有找到 mujoco，请使用 tianshou_env 环境运行此程序"
        ) from error

    arm = NUCControl()
    target_point = arm._point(target)
    frames = arm.trajectory(target_point, steps=steps, wrist_roll_rad=wrist_roll_rad)
    model = mujoco.MjModel.from_xml_string(_xml_for_arm(arm))
    data = mujoco.MjData(model)
    data.mocap_pos[0] = (target_point.x_mm / 1000.0, target_point.y_mm / 1000.0, target_point.z_mm / 1000.0)
    # 先把物理模型放到 NUCControl 的初始姿态，再开始施加执行器目标，
    # 避免从全零姿态瞬间拉到目标姿态而产生巨大加速度。
    initial_controls = _joint_targets(
        arm.current_zenith_angles_rad,
        arm.current_wrist_roll_rad,
    )
    _set_pose(data, initial_controls)
    data.ctrl[:4] = initial_controls
    data.ctrl[4:] = (0.0, 0.0)
    mujoco.mj_forward(model, data)

    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.distance = 1.7
        for index, frame in enumerate(frames, 1):
            controls = _joint_targets(
                frame.ik.zenith_angles_rad, frame.ik.joint_angles_rad[4]
            )
            data.ctrl[:4] = controls
            _set_pose(data, controls)
            mujoco.mj_forward(model, data)
            viewer.sync()
            print(
                "第 {:02d}/{:02d} 段：末端 ({:.1f}, {:.1f}, {:.1f}) mm".format(
                    index, len(frames), frame.point.x_mm, frame.point.y_mm, frame.point.z_mm
                )
            )
            if not viewer.is_running():
                break
            time.sleep(interval_s)
        if viewer.is_running():
            print("到达球的位置，夹爪开始闭合。")
            close_steps = 20
            for index in range(close_steps + 1):
                ratio = index / float(close_steps)
                left = -0.032 * ratio
                right = 0.032 * ratio
                data.ctrl[4:] = (left, right)
                _set_pose(
                    data,
                    _joint_targets(
                        frames[-1].ik.zenith_angles_rad,
                        frames[-1].ik.joint_angles_rad[4],
                    ),
                    left,
                    right,
                )
                mujoco.mj_forward(model, data)
                viewer.sync()
                time.sleep(0.03)
            print("夹爪已夹住蓝色小球。")
        # 保持最终姿态，直到用户关闭 MuJoCo 窗口。
        while viewer.is_running():
            _set_pose(data, _joint_targets(frames[-1].ik.zenith_angles_rad, wrist_roll_rad), -0.032, 0.032)
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(0.01)
    return target_point


def main() -> None:
    parser = argparse.ArgumentParser(description="NUCControl MuJoCo 机械臂仿真")
    parser.add_argument("--target", nargs=3, type=float, metavar=("X", "Y", "Z"))
    parser.add_argument("--steps", type=int, default=60)
    parser.add_argument("--interval", type=float, default=0.02)
    parser.add_argument("--wrist-roll", type=float, default=0.0)
    args = parser.parse_args()
    text = " ".join(str(value) for value in args.target) if args.target else input("目标坐标 x y z > ")
    try:
        target = _parse_target(text)
        print("MuJoCo 仿真启动，关闭窗口后程序结束。")
        run(target, steps=args.steps, interval_s=args.interval, wrist_roll_rad=args.wrist_roll)
    except (ArmControlError, ValueError, RuntimeError) as error:
        print("仿真启动失败：", error)


if __name__ == "__main__":
    main()
