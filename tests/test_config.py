import tempfile
import unittest
from pathlib import Path

from uarm_xarm6_teleop.config import load_config


class ConfigTests(unittest.TestCase):
    def test_defaults(self):
        config = load_config()
        self.assertEqual(config.serial.ids, (1, 2, 3, 4, 5, 6, 7))
        self.assertEqual(config.leader.midpoint, 2047)
        self.assertEqual(config.leader.gripper_zero_position, 2457)
        self.assertEqual(config.leader.gripper_pressed_position, 2757)
        self.assertEqual(len(config.xarm6.reference_degrees), 6)
        self.assertEqual(len(config.xarm6.joint_directions), 6)
        self.assertEqual(config.xarm6.gripper_mode, "proportional")
        self.assertAlmostEqual(config.xarm6.gripper_travel_degrees, 26.3671875)
        self.assertEqual(config.physical_xarm.mode, 6)
        self.assertEqual(config.physical_xarm.rate, 20.0)
        self.assertEqual(config.physical_xarm.gripper_force, 20)
        self.assertEqual(len(config.physical_xarm.joint_lower_degrees), 6)

    def test_partial_config_uses_other_defaults(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text('[serial]\ndevice = "/dev/test"\n')
            config = load_config(path)
        self.assertEqual(config.serial.device, "/dev/test")
        self.assertEqual(config.serial.baudrate, 1_000_000)

    def test_invalid_direction_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[leader]\ndirections = [1, 1, 1, 0, 1, 1, 1]\n")
            with self.assertRaisesRegex(ValueError, "directions"):
                load_config(path)

    def test_non_servo_physical_mode_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[physical_xarm]\nmode = 0\n")
            with self.assertRaisesRegex(ValueError, "mode must be 6"):
                load_config(path)

    def test_invalid_toggle_hysteresis_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[xarm6]\ngripper_press_degrees = 4\ngripper_release_degrees = 4\n")
            with self.assertRaisesRegex(ValueError, "must exceed"):
                load_config(path)


if __name__ == "__main__":
    unittest.main()
