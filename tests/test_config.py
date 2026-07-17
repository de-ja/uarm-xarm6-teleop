import tempfile
import unittest
from pathlib import Path

from uarm_xarm6_teleop.config import load_config


class ConfigTests(unittest.TestCase):
    def test_defaults(self):
        config = load_config()
        self.assertEqual(config.serial.ids, (1, 2, 3, 4, 5, 6, 7))
        self.assertEqual(config.leader.midpoint, 2047)
        self.assertEqual(len(config.xarm6.reference_degrees), 6)
        self.assertEqual(len(config.xarm6.joint_directions), 6)

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


if __name__ == "__main__":
    unittest.main()
