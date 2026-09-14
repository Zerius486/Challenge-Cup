"""平面二连杆模型的基础测试。"""

import struct
import unittest
import zlib

from NUCControl import (
    ArmConfig,
    ArmControlError,
    DEFAULT_CONFIG,
    INITIAL_LINK_ANGLES_RAD,
    NUCControl,
    UnreachableTargetError,
)


class ArmControlTests(unittest.TestCase):
    def test_config_has_four_links_and_two_independent_angles(self):
        config = ArmConfig(
            link_lengths_mm=(100.0, 80.0, 60.0, 40.0),
            initial_zenith_angles_rad=(0.2, -0.1),
            base_yaw_rad=0.0,
        )
        self.assertEqual(len(config.link_lengths_mm), 4)
        self.assertEqual(len(config.initial_zenith_angles_rad), 2)
        self.assertEqual(len(config.initial_joint_angles_rad), 5)
        self.assertEqual(len(config.initial_link_orientations_rad), 4)

    def test_config_can_describe_four_initial_link_angles(self):
        config = ArmConfig(
            initial_link_angles_rad=INITIAL_LINK_ANGLES_RAD,
        )
        config.validate()
        expected = (1.5707963267948966, 0.0, -1.5707963267948966, 0.0)
        for actual, wanted in zip(config.initial_pitch_joint_angles_rad, expected):
            self.assertAlmostEqual(actual, wanted)

    def test_default_uses_configured_four_link_pose(self):
        self.assertEqual(
            DEFAULT_CONFIG.initial_link_orientations_rad,
            INITIAL_LINK_ANGLES_RAD,
        )

    def test_forward_inverse_round_trip(self):
        config = ArmConfig(
            link_lengths_mm=(100.0, 80.0, 30.0, 40.0),
            initial_zenith_angles_rad=(0.3, -0.2),
            base_yaw_rad=0.0,
            zenith_min_rad=(-1.5, -2.5),
            zenith_max_rad=(1.5, 2.5),
        )
        arm = NUCControl(config)
        source = (0.3, -0.2)
        target = arm.forward_kinematics(source)
        result = arm.inverse_kinematics(target, seed_zenith_angles_rad=(0.0, 0.0))
        self.assertTrue(result.ok)
        self.assertLessEqual(result.position_error_mm, 1.0e-6)
        self.assertAlmostEqual(result.zenith_angles_rad[0], source[0], places=5)
        self.assertAlmostEqual(result.zenith_angles_rad[1], source[1], places=5)

    def test_top_links_are_horizontal(self):
        arm = NUCControl()
        points = arm.forward_link_points((0.4, -0.2))
        l3_start, l3_end = points[1], points[2]
        l4_start, l4_end = points[2], points[3]
        self.assertAlmostEqual(l3_end.z_mm - l3_start.z_mm, 0.0, places=6)
        self.assertAlmostEqual(l4_end.z_mm - l4_start.z_mm, 0.0, places=6)
        self.assertGreater(l3_end.x_mm - l3_start.x_mm, 0.0)
        self.assertGreater(l4_end.x_mm - l4_start.x_mm, 0.0)
        self.assertAlmostEqual(arm.link_absolute_angles((0.4, -0.2))[2], 0.0)
        self.assertAlmostEqual(arm.link_absolute_angles((0.4, -0.2))[3], 0.0)

    def test_plane_and_unreachable_checks(self):
        arm = NUCControl()
        near_plane = arm.inverse_kinematics((700.0, 0.5, 300.0))
        self.assertTrue(near_plane.ok)
        self.assertAlmostEqual(near_plane.plane_error_mm, 0.5, places=6)
        with self.assertRaises(UnreachableTargetError):
            arm.motor_commands((500.0, 50.0, 180.0))
        with self.assertRaises(UnreachableTargetError):
            arm.motor_commands((2000.0, 0.0, 0.0))

    def test_maincode_motor_mapping(self):
        arm = NUCControl()
        commands = arm.motor_commands((700.0, 0.0, 300.0))
        self.assertEqual(len(commands), 5)
        self.assertEqual([command.node_id for command in commands], [11, 12, 13, 14, 15])
        self.assertEqual([command.driver for command in commands], ["erob"] * 4 + ["robstride"])
        self.assertEqual([command.can_bus for command in commands], [1] * 4 + [2])
        self.assertEqual(len(commands[0].udp_payload()), 28)
        self.assertEqual(struct.unpack_from("<BBBB", commands[0].udp_payload()), (1, 1, 11, 4))
        packets = arm.packets_for_commands(commands, 100)
        self.assertEqual(len(packets), 5)
        self.assertEqual(len(packets[0]), 44)

    def test_protocol_validation_matches_maincode_reserved_bits(self):
        with self.assertRaises(ArmControlError):
            NUCControl(ArmConfig(arm_option=0x8001))
        with self.assertRaises(ArmControlError):
            arm = NUCControl()
            arm.motor_packets((700.0, 0.0, 300.0), sequence=True)

    def test_protocol_format_and_trajectory(self):
        arm = NUCControl()
        self.assertEqual(
            arm.format_position_command((700.0, 0.0, 300.0)),
            b"PXYZ:+070000,+000000,+030000,/",
        )
        self.assertEqual(arm.format_gripper_command(0.5, 1000), b"JZH_+500_1000/")
        frames = arm.trajectory((700.0, 0.0, 300.0), steps=2)
        self.assertEqual(len(frames), 2)
        self.assertEqual(len(frames[-1].commands), 5)

    def test_trajectory_uses_minimum_jerk_time_scaling(self):
        arm = NUCControl()
        start_zenith = arm.current_zenith_angles_rad
        target_zenith = (0.2, -0.1)
        target = arm.forward_kinematics(target_zenith)
        frames = arm.trajectory(target, steps=10, update_state=False)
        first = frames[0].ik.zenith_angles_rad
        # t=0.1 时的最小 jerk 进度为 0.00856。
        expected_progress = 0.00856
        for actual, start, end in zip(first, start_zenith, target_zenith):
            self.assertAlmostEqual(
                actual, start + (end - start) * expected_progress, places=5
            )
        self.assertAlmostEqual(
            frames[-1].ik.zenith_angles_rad[0], target_zenith[0], places=7
        )

    def test_joint_angle_udp_packet_contains_only_six_angles(self):
        arm = NUCControl()
        packet = arm.angle_packet((700.0, 0.0, 300.0), sequence=7, gripper_rad=0.25)
        self.assertEqual(len(packet), 12 + 24 + 4)
        magic, version, message_type, flags, payload_length, sequence = struct.unpack_from(
            "<2sBBHHI", packet
        )
        self.assertEqual((magic, version, message_type, flags, payload_length, sequence),
                         (b"UW", 1, 0x12, 0, 24, 7))
        angles = struct.unpack_from("<6f", packet, 12)
        self.assertAlmostEqual(angles[-1], 0.25, places=6)

        trajectory = arm.trajectory_packets(
            (700.0, 0.0, 300.0), sequence=10, steps=3, gripper_rad=0.25
        )
        self.assertEqual(len(trajectory), 3)
        self.assertTrue(all(len(item) == 40 for item in trajectory))
        self.assertEqual(
            [struct.unpack_from("<I", item, 36)[0] for item in trajectory],
            [
                zlib.crc32(item[:36]) & 0xFFFFFFFF
                for item in trajectory
            ],
        )

    def test_explicit_elbow_branch(self):
        config = ArmConfig(
            link_lengths_mm=(100.0, 80.0, 30.0, 40.0),
            initial_zenith_angles_rad=(0.3, -0.2),
            zenith_min_rad=(-2.5, -2.5),
            zenith_max_rad=(2.5, 2.5),
            joint_min_rad=(-3.0, -3.0, -3.0, -3.0, -3.0),
            joint_max_rad=(3.0, 3.0, 3.0, 3.0, 3.0),
        )
        arm = NUCControl(config)
        target = arm.forward_kinematics((0.3, -0.2))
        first = arm.inverse_kinematics(target, elbow_branch=0)
        second = arm.inverse_kinematics(target, elbow_branch=1)
        self.assertTrue(first.ok)
        self.assertTrue(second.ok)
        self.assertNotAlmostEqual(
            first.zenith_angles_rad[0], second.zenith_angles_rad[0], places=4
        )

    def test_default_elbow_branch_minimizes_total_joint_change(self):
        config = ArmConfig(
            link_lengths_mm=(100.0, 80.0, 30.0, 40.0),
            initial_zenith_angles_rad=(0.3, -0.2),
            zenith_min_rad=(-2.5, -2.5),
            zenith_max_rad=(2.5, 2.5),
            joint_min_rad=(-3.0, -3.0, -3.0, -3.0, -3.0),
            joint_max_rad=(3.0, 3.0, 3.0, 3.0, 3.0),
        )
        arm = NUCControl(config)
        target = arm.forward_kinematics((0.3, -0.2))
        default = arm.inverse_kinematics(target)
        branch_results = [
            arm.inverse_kinematics(target, elbow_branch=branch)
            for branch in (0, 1)
        ]
        self.assertTrue(default.ok)
        valid = [result for result in branch_results if result.ok]
        self.assertGreaterEqual(len(valid), 2)
        seed_joints = arm._joint_angles_from_zenith(*arm.current_zenith_angles_rad)

        def change(result):
            return sum(
                abs(result.joint_angles_rad[index] - seed_joints[index]) ** 2
                for index in range(5)
            )

        self.assertEqual(default.branch, min(valid, key=change).branch)


if __name__ == "__main__":
    unittest.main()
