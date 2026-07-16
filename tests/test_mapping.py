import unittest

import numpy as np

from uarm_xarm6_teleop.mapping import (
    XArm6Mapping,
    positions_to_radians,
    signed_delta,
)


class MappingTests(unittest.TestCase):
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
        action = XArm6Mapping().action(leader)
        np.testing.assert_allclose(
            np.rad2deg(action[:6]),
            [10, 20, 30, 50, -40, 60],
            atol=1e-5,
        )
        self.assertAlmostEqual(float(action[6]), 0.405, places=6)


if __name__ == "__main__":
    unittest.main()
