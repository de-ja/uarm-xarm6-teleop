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
        self.assertEqual(config.wireless.leader_timeout, 0.15)
        self.assertEqual(config.wireless.leader_max_consecutive_timeouts, 4)
        self.assertEqual(config.wireless.browser_grace_seconds, 2.0)
        self.assertEqual(config.physical_xarm.watchdog_timeout, 1.0)
        self.assertEqual(config.physical_xarm.catchup_step_degrees, 3.0)
        self.assertEqual(config.physical_xarm.catchup_max_divergence_degrees, 45.0)

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
            with self.assertRaisesRegex(ValueError, "mode must be 1"):
                load_config(path)

    def test_wireless_override_is_applied(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[wireless]\nleader_timeout = 0.05\n")
            config = load_config(path)
        self.assertEqual(config.wireless.leader_timeout, 0.05)
        self.assertEqual(config.physical_xarm.rate, 20.0)

    def test_non_positive_leader_timeout_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[wireless]\nleader_timeout = 0\n")
            with self.assertRaisesRegex(ValueError, "leader_timeout must be positive"):
                load_config(path)

    def test_blind_time_beyond_watchdog_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[wireless]\nleader_timeout = 0.3\nleader_max_consecutive_timeouts = 4\n"
            )
            with self.assertRaisesRegex(ValueError, "watchdog_timeout"):
                load_config(path)

    def test_catchup_step_beyond_jump_limit_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[physical_xarm]\ncatchup_step_degrees = 25.0\n")
            with self.assertRaisesRegex(ValueError, "catchup_step_degrees"):
                load_config(path)

    def test_zero_tolerance_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[wireless]\nleader_max_consecutive_timeouts = 0\n")
            config = load_config(path)
        self.assertEqual(config.wireless.leader_max_consecutive_timeouts, 0)

    def test_invalid_toggle_hysteresis_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[xarm6]\ngripper_press_degrees = 4\ngripper_release_degrees = 4\n")
            with self.assertRaisesRegex(ValueError, "must exceed"):
                load_config(path)

    def test_gripper_kind_defaults_to_g2(self):
        self.assertEqual(load_config().physical_xarm.gripper_kind, "g2")

    def test_unknown_gripper_kind_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text('[physical_xarm]\ngripper_kind = "bio"\n')
            with self.assertRaisesRegex(ValueError, "gripper_kind"):
                load_config(path)

    def test_g2_position_beyond_its_millimetre_range_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[physical_xarm]\ngripper_open_position = 850\n")
            with self.assertRaisesRegex(ValueError, "gripper_open_position"):
                load_config(path)

    def test_classic_accepts_pulse_positions_and_r_per_minute_speed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[physical_xarm]\n"
                'gripper_kind = "classic"\n'
                "gripper_open_position = 850\n"
                "gripper_speed = 1500\n"
                "gripper_max_step = 20\n"
            )
            physical = load_config(path).physical_xarm
        self.assertEqual(physical.gripper_kind, "classic")
        self.assertEqual(physical.gripper_open_position, 850)
        self.assertEqual(physical.gripper_speed, 1500)

    def test_classic_position_beyond_its_pulse_range_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                '[physical_xarm]\ngripper_kind = "classic"\ngripper_open_position = 900\n'
            )
            with self.assertRaisesRegex(ValueError, "gripper_open_position"):
                load_config(path)

    def test_classic_ignores_the_g2_force_range(self):
        # The classic SDK takes no force argument, so the 1-100 cap cannot apply.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[physical_xarm]\n"
                'gripper_kind = "classic"\n'
                "gripper_open_position = 850\n"
                "gripper_speed = 1500\n"
                "gripper_force = 500\n"
            )
            self.assertEqual(load_config(path).physical_xarm.gripper_force, 500)

    def test_servo_mode_is_accepted(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[physical_xarm]\nmode = 1\nrate = 100.0\n"
                "max_target_jump_degrees = 1.5\ncatchup_step_degrees = 0.8\n"
            )
            physical = load_config(path).physical_xarm
        self.assertEqual(physical.mode, 1)

    def test_servo_mode_rejects_a_jump_and_rate_beyond_the_joint_speed_limit(self):
        # 2.5 deg at 100 Hz asks for 250 deg/s, past the arm's 180 deg/s ceiling.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[physical_xarm]\nmode = 1\nrate = 100.0\n"
                "max_target_jump_degrees = 2.5\ncatchup_step_degrees = 0.8\n"
            )
            with self.assertRaisesRegex(ValueError, "180 deg/s joint limit"):
                load_config(path)

    def test_planning_mode_is_not_bound_by_the_servo_velocity_check(self):
        # Mode 6 plans within its own limits, so the same numbers are fine.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text("[physical_xarm]\nmode = 6\nrate = 100.0\n")
            self.assertEqual(load_config(path).physical_xarm.mode, 6)

    def test_no_sensors_are_configured_by_default(self):
        self.assertEqual(load_config().sensors, ())

    def test_a_sensor_table_is_parsed(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[[sensors]]\n"
                'kind = "eflesh"\n'
                'name = "gripper_tactile"\n'
                'port = "usb:serial=TEST"\n'
                "settle = 7.5\n"
            )
            sensors = load_config(path).sensors
        self.assertEqual(len(sensors), 1)
        self.assertEqual(sensors[0].kind, "eflesh")
        self.assertEqual(sensors[0].settle, 7.5)

    def test_settle_defaults_rather_than_being_required(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                '[[sensors]]\nkind = "eflesh"\nname = "gripper_tactile"\nport = "usb:serial=TEST"\n'
            )
            self.assertEqual(load_config(path).sensors[0].settle, 5.0)

    def test_a_zero_settle_is_rejected(self):
        # A baseline captured before the sensor settles reads several times
        # noisier than the true floor, and nothing downstream can detect it.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                "[[sensors]]\n"
                'kind = "eflesh"\n'
                'name = "gripper_tactile"\n'
                'port = "usb:serial=TEST"\n'
                "settle = 0.0\n"
            )
            with self.assertRaisesRegex(ValueError, "positive settle"):
                load_config(path)

    def test_a_bare_tty_node_is_rejected_for_a_sensor(self):
        # The leader and a USB CDC sensor both enumerate as ttyACM, and the
        # numbering depends on boot order, so a bare node is not safe.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text(
                '[[sensors]]\nkind = "eflesh"\nname = "gripper_tactile"\nport = "/dev/ttyACM1"\n'
            )
            with self.assertRaisesRegex(ValueError, "usb: selector"):
                load_config(path)

    def test_duplicate_sensor_names_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            entry = '[[sensors]]\nkind = "eflesh"\nname = "tactile"\nport = "usb:serial=TEST"\n'
            path.write_text(entry + entry)
            with self.assertRaisesRegex(ValueError, "Duplicate sensor name"):
                load_config(path)

    def test_an_overlay_replaces_the_sensor_list_outright(self):
        # Merging entry-wise would make removing a sensor locally impossible,
        # which is the common case when the hardware is not attached.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.toml"
            path.write_text('[serial]\ndevice = "/dev/test"\n')
            self.assertEqual(load_config(path).sensors, ())


if __name__ == "__main__":
    unittest.main()
