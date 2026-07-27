import unittest

import numpy as np

from uarm_xarm6_teleop.mapping import (
    XArm6Mapping,
    positions_to_radians,
    signed_delta,
)


class MappingTests(unittest.TestCase):
    def mapping(self):
        return XArm6Mapping(
            reference_degrees=(0.0, -75.0, 10.0, 0.0, 60.0, 0.0),
            joint_directions=(-1, -1, -1, -1, 1, -1),
        )

    def test_midpoint_and_wraparound(self):
        self.assertEqual(signed_delta(2047), 0)
        self.assertEqual(signed_delta(2048), 1)
        self.assertEqual(signed_delta(0), -2047)
        self.assertEqual(signed_delta(4095), -2048)

    def test_directions_are_applied(self):
        angles = positions_to_radians(
            [2048, 2048, 2047, 2047, 2047, 2047, 2047],
            midpoint=2047,
            directions=[1, -1, 1, 1, 1, 1, 1],
        )
        self.assertGreater(angles[0], 0)
        self.assertLess(angles[1], 0)

    def test_xarm_mapping(self):
        leader = np.deg2rad([10, 20, 30, 40, 50, 60, 45])
        action = self.mapping().action(leader)
        np.testing.assert_allclose(
            np.rad2deg(action[:6]),
            [-10.0, -95.0, -20.0, -40.0, 110.0, -60.0],
            atol=1e-5,
        )
        self.assertAlmostEqual(float(action[6]), 0.405, places=6)

    def test_zero_leader_pose_maps_to_xarm_reference(self):
        action = self.mapping().action(np.zeros(7))
        np.testing.assert_allclose(
            np.rad2deg(action[:6]),
            [0.0, -75.0, 10.0, 0.0, 60.0, 0.0],
            atol=1e-5,
        )

    def test_toggle_gripper_uses_hysteresis_and_one_toggle_per_pull(self):
        mapping = XArm6Mapping(
            reference_degrees=(0.0, -75.0, 10.0, 0.0, 60.0, 0.0),
            joint_directions=(-1, -1, -1, -1, 1, -1),
            gripper_mode="toggle",
            gripper_press_degrees=10.0,
            gripper_release_degrees=4.0,
        )

        def grip(trigger_degrees):
            leader = np.zeros(7)
            leader[6] = np.deg2rad(trigger_degrees)
            return float(mapping.action(leader)[6])

        self.assertEqual(grip(0), 0.0)
        self.assertAlmostEqual(grip(11), 0.81)
        self.assertAlmostEqual(grip(12), 0.81)
        self.assertAlmostEqual(grip(6), 0.81)
        self.assertAlmostEqual(grip(3), 0.81)
        self.assertEqual(grip(11), 0.0)

    def test_toggle_ignores_trigger_held_during_startup(self):
        mapping = XArm6Mapping(
            reference_degrees=(0.0, -75.0, 10.0, 0.0, 60.0, 0.0),
            joint_directions=(-1, -1, -1, -1, 1, -1),
            gripper_mode="toggle",
        )
        held = np.zeros(7)
        held[6] = np.deg2rad(12.0)
        self.assertEqual(float(mapping.action(held)[6]), 0.0)
        mapping.action(np.zeros(7))
        self.assertAlmostEqual(float(mapping.action(held)[6]), 0.81)

    def test_toggle_can_preserve_intermediate_physical_start_position(self):
        mapping = XArm6Mapping(
            reference_degrees=(0.0, -75.0, 10.0, 0.0, 60.0, 0.0),
            joint_directions=(-1, -1, -1, -1, 1, -1),
            gripper_mode="toggle",
        )
        mapping.reset_gripper(0.0, closed=False, command=0.405)
        self.assertAlmostEqual(float(mapping.action(np.zeros(7))[6]), 0.405)


if __name__ == "__main__":
    unittest.main()
