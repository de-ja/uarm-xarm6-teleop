import unittest

import numpy as np

from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.feetech import FeetechLeader


class FeetechLeaderTests(unittest.TestCase):
    def test_gripper_uses_its_own_zero_position(self):
        config = load_config()
        leader = FeetechLeader(config.serial, config.leader)
        leader.read_positions = lambda: (2047, 2047, 2047, 2047, 2047, 2047, 2457)

        sample = leader.read()

        np.testing.assert_allclose(sample.degrees, np.zeros(7), atol=1e-9)

    def test_gripper_angle_is_relative_to_saved_zero(self):
        config = load_config()
        leader = FeetechLeader(config.serial, config.leader)
        leader.read_positions = lambda: (2047, 2047, 2047, 2047, 2047, 2047, 3481)

        sample = leader.read()

        self.assertAlmostEqual(sample.degrees[6], 90.0)

    def test_saved_pressed_position_matches_configured_travel(self):
        config = load_config()
        leader = FeetechLeader(config.serial, config.leader)
        leader.read_positions = lambda: (
            2047,
            2047,
            2047,
            2047,
            2047,
            2047,
            config.leader.gripper_pressed_position,
        )

        sample = leader.read()

        self.assertAlmostEqual(
            sample.degrees[6], config.xarm6.gripper_travel_degrees
        )


if __name__ == "__main__":
    unittest.main()
