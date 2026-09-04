import unittest
from dataclasses import replace

import numpy as np

from uarm_xarm6_teleop.backends.xarm import (
    TargetSafety,
    XArm6Hardware,
    XArmHardwareError,
)
from uarm_xarm6_teleop.config import load_config


class FakeArm:
    def __init__(self, _ip, is_radian=True, joints=None):
        self.connected = True
        self.version = "fake-1.0"
        self.axis = 6
        self.mode = 0
        self.state = 0
        self.errors = [0, 0]
        self.joints = list(joints if joints is not None else np.zeros(6))
        self.gripper = 84
        self.gripper_force = 0
        self.gripper_status = 0
        self.gripper_status_code = 0
        self.gripper_error = 0
        self.calls = []

    def get_state(self):
        return 0, self.state

    def get_err_warn_code(self):
        return 0, self.errors

    def get_servo_angle(self, is_radian=True):
        return 0, self.joints

    def get_gripper_g2_position(self):
        return 0, self.gripper

    def get_gripper_g2_force(self):
        return 0, self.gripper_force

    def get_gripper_status(self):
        return self.gripper_status_code, self.gripper_status

    def get_gripper_err_code(self):
        return 0, self.gripper_error

    def is_joint_limit(self, joint, is_radian=True):
        return 0, False

    def motion_enable(self, enable=True):
        self.calls.append(("motion_enable", enable))
        return 0

    def set_mode(self, mode):
        self.mode = mode
        self.calls.append(("set_mode", mode))
        return 0

    def set_state(self, state):
        self.state = state
        self.calls.append(("set_state", state))
        return 0

    def set_gripper_mode(self, mode):
        self.calls.append(("set_gripper_mode", mode))
        return 0

    def set_gripper_enable(self, enable=True):
        self.calls.append(("set_gripper_enable", enable))
        return 0

    def set_servo_angle(self, **kwargs):
        self.joints = list(kwargs["angle"])
        self.calls.append(("set_servo_angle", kwargs))
        return 0

    def set_servo_angle_j(self, angles, **kwargs):
        self.joints = list(angles)
        self.calls.append(("set_servo_angle_j", angles, kwargs))
        return 0

    def set_gripper_g2_position(self, position, **kwargs):
        self.gripper = position
        self.calls.append(("set_gripper_g2_position", position, kwargs))
        return 0

    def get_gripper_position(self):
        return 0, self.gripper

    def set_gripper_position(self, position, **kwargs):
        self.gripper = position
        self.calls.append(("set_gripper_position", position, kwargs))
        return 0

    def disconnect(self):
        self.connected = False


class XArmBackendTests(unittest.TestCase):
    def setUp(self):
        base = load_config().physical_xarm
        self.config = replace(base, robot_ip="192.0.2.1", watchdog_timeout=10.0)
        self.reference = np.deg2rad([0.0, -75.0, 10.0, 0.0, 60.0, 0.0])

    def make_backend(self, joints=None):
        fake = FakeArm("192.0.2.1", joints=self.reference if joints is None else joints)
        backend = XArm6Hardware(self.config, api_factory=lambda *_args, **_kwargs: fake)
        return backend, fake

    def test_static_limit_rejects_j3_above_configured_limit(self):
        safety = TargetSafety(self.config)
        target = self.reference.copy()
        target[2] = np.deg2rad(12.0)
        with self.assertRaisesRegex(XArmHardwareError, "J3 target"):
            safety.validate(target)

    def test_per_sample_jump_is_rejected(self):
        safety = TargetSafety(self.config)
        safety.reset(self.reference)
        target = self.reference.copy()
        target[0] += np.deg2rad(11.0)
        with self.assertRaisesRegex(XArmHardwareError, "jumped"):
            safety.validate(target)

    def test_approach_clamps_a_large_gap_to_one_slew_step(self):
        safety = TargetSafety(self.config)
        safety.reset(self.reference)
        target = self.reference.copy()
        target[0] += np.deg2rad(40.0)

        slewed = safety.approach(target, self.config.catchup_step_degrees)
        moved = np.rad2deg(slewed[0] - self.reference[0])
        self.assertAlmostEqual(moved, self.config.catchup_step_degrees, places=6)
        # A slewed target must always survive the jump check it replaces.
        safety.validate(slewed)

    def test_approach_returns_the_target_once_it_is_within_one_step(self):
        safety = TargetSafety(self.config)
        safety.reset(self.reference)
        target = self.reference.copy()
        target[0] += np.deg2rad(1.0)

        slewed = safety.approach(target, self.config.catchup_step_degrees)
        np.testing.assert_allclose(slewed, target)

    def test_divergence_reports_the_largest_joint_gap(self):
        safety = TargetSafety(self.config)
        self.assertEqual(safety.divergence_degrees(self.reference), 0.0)
        safety.reset(self.reference)
        target = self.reference.copy()
        target[1] += np.deg2rad(30.0)
        self.assertAlmostEqual(safety.divergence_degrees(target), 30.0, places=6)

    def test_inspection_never_enables_motion(self):
        backend, fake = self.make_backend()
        status = backend.inspect()
        backend.close()
        self.assertEqual(status.joint_degrees[1], -75.0)
        self.assertFalse(any(call[0] == "motion_enable" for call in fake.calls))

    def test_inspection_accepts_sdk_without_public_g2_force_getter(self):
        fake = FakeArm("192.0.2.1", joints=self.reference)
        fake.get_gripper_g2_force = None
        backend = XArm6Hardware(self.config, api_factory=lambda *_args, **_kwargs: fake)

        status = backend.inspect()
        backend.close()

        self.assertIsNone(status.gripper_force)
        self.assertFalse(any(call[0] == "motion_enable" for call in fake.calls))

    def test_padded_seven_value_sdk_sample_is_accepted_for_xarm6(self):
        joints = np.concatenate([self.reference, [0.0]])
        backend, _fake = self.make_backend(joints=joints)
        status = backend.inspect()
        backend.close()
        self.assertEqual(len(status.joint_degrees), 6)

    def test_startup_mismatch_blocks_before_motion_enable(self):
        backend, fake = self.make_backend(joints=np.zeros(6))
        with self.assertRaisesRegex(XArmHardwareError, "startup tolerance"):
            backend.arm_motion(self.reference)
        backend.close()
        self.assertFalse(any(call[0] == "motion_enable" for call in fake.calls))

    def test_arm_and_command_use_mode_6_and_limit_g2_force(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        action = np.concatenate([self.reference, [0.81]])
        backend.command(action, gripper_command_max=0.81)
        backend.close()

        self.assertIn(("set_mode", 6), fake.calls)
        servo_call = next(call for call in fake.calls if call[0] == "set_servo_angle")
        self.assertFalse(servo_call[1]["wait"])
        self.assertTrue(servo_call[1]["is_radian"])
        gripper_call = next(call for call in fake.calls if call[0] == "set_gripper_g2_position")
        self.assertEqual(gripper_call[1], 82)
        self.assertEqual(gripper_call[2]["speed"], 50)
        self.assertEqual(gripper_call[2]["force"], 20)
        self.assertIn(("set_state", 4), fake.calls)

    def test_catch_up_slews_toward_a_gap_and_then_tracks_normally(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)

        # The leader moved 12 deg while the wireless link was blind.
        target = self.reference.copy()
        target[0] += np.deg2rad(12.0)
        backend.begin_catch_up()
        self.assertTrue(backend.catching_up)

        commanded = []
        for _ in range(6):
            backend.command(np.concatenate([target, [0.0]]), gripper_command_max=0.81)
            call = [c for c in fake.calls if c[0] == "set_servo_angle"][-1]
            commanded.append(np.rad2deg(call[1]["angle"][0] - self.reference[0]))

        steps = np.diff([0.0, *commanded])
        self.assertTrue(np.all(steps <= self.config.catchup_step_degrees + 1e-6))
        self.assertAlmostEqual(commanded[-1], 12.0, places=6)
        self.assertFalse(backend.catching_up)
        backend.close()

    def test_catch_up_faults_when_the_leader_diverged_too_far(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)

        target = self.reference.copy()
        target[0] += np.deg2rad(60.0)
        backend.begin_catch_up()
        with self.assertRaisesRegex(XArmHardwareError, "diverged"):
            backend.command(np.concatenate([target, [0.0]]), gripper_command_max=0.81)
        backend.close()

    def test_jump_without_a_gap_still_faults(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)

        # No catch-up was requested, so a divergent sample is still a fault.
        target = self.reference.copy()
        target[0] += np.deg2rad(12.0)
        with self.assertRaisesRegex(XArmHardwareError, "jumped"):
            backend.command(np.concatenate([target, [0.0]]), gripper_command_max=0.81)
        backend.close()

    def test_ready_feedback_state_accepts_commands(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        fake.state = 2

        backend.command(
            np.concatenate([self.reference, [0.0]]),
            gripper_command_max=0.81,
        )
        backend.close()

        self.assertTrue(any(call[0] == "set_servo_angle" for call in fake.calls))

    def test_g2_grasp_status_freezes_closing_until_open_command(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        closed = np.concatenate([self.reference, [0.81]])
        opened = np.concatenate([self.reference, [0.0]])

        backend.command(closed, gripper_command_max=0.81)
        self.assertEqual(fake.gripper, 82)
        fake.gripper = 60
        fake.gripper_status = 2
        backend.command(closed, gripper_command_max=0.81)
        self.assertTrue(backend.gripper_contact_latched)
        self.assertEqual(fake.gripper, 60)

        gripper_calls = len([call for call in fake.calls if call[0] == "set_gripper_g2_position"])
        fake.gripper_status = 0
        backend.command(closed, gripper_command_max=0.81)
        self.assertEqual(
            len([call for call in fake.calls if call[0] == "set_gripper_g2_position"]),
            gripper_calls,
        )

        backend.command(opened, gripper_command_max=0.81)
        self.assertFalse(backend.gripper_contact_latched)
        self.assertEqual(fake.gripper, 62)
        backend.close()

    def test_g2_error_stops_closing(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        fake.gripper_error = 11
        closed = np.concatenate([self.reference, [0.81]])
        with self.assertRaisesRegex(XArmHardwareError, "error 11"):
            backend.command(closed, gripper_command_max=0.81)
        backend.close()


class ClassicGripperTests(unittest.TestCase):
    """Cover the original xArm Gripper, which uses pulses and caps no force."""

    def setUp(self):
        base = load_config().physical_xarm
        self.config = replace(
            base,
            robot_ip="192.0.2.1",
            watchdog_timeout=10.0,
            gripper_kind="classic",
            gripper_open_position=850,
            gripper_closed_position=0,
            gripper_speed=1500,
            gripper_max_step=20,
        )
        self.reference = np.deg2rad([0.0, -75.0, 10.0, 0.0, 60.0, 0.0])

    def make_backend(self):
        fake = FakeArm("192.0.2.1", joints=self.reference)
        fake.gripper = 850
        backend = XArm6Hardware(self.config, api_factory=lambda *_args, **_kwargs: fake)
        return backend, fake

    def test_unknown_gripper_kind_is_rejected_before_any_sdk_call(self):
        config = replace(self.config, gripper_kind="bio")
        with self.assertRaisesRegex(XArmHardwareError, "gripper_kind"):
            XArm6Hardware(config, api_factory=lambda *_args, **_kwargs: FakeArm("192.0.2.1"))

    def test_arming_enables_the_gripper_servo_and_selects_position_mode(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.close()

        self.assertIn(("set_gripper_enable", True), fake.calls)
        self.assertIn(("set_gripper_mode", 0), fake.calls)

    def test_g2_arming_does_not_enable_a_gripper_servo(self):
        # The G2 needs no explicit enable, so the classic-only calls must not leak.
        base = load_config().physical_xarm
        config = replace(base, robot_ip="192.0.2.1", watchdog_timeout=10.0)
        fake = FakeArm("192.0.2.1", joints=self.reference)
        backend = XArm6Hardware(config, api_factory=lambda *_args, **_kwargs: fake)
        backend.arm_motion(self.reference)
        backend.close()

        self.assertFalse(any(call[0] == "set_gripper_enable" for call in fake.calls))
        self.assertFalse(any(call[0] == "set_gripper_mode" for call in fake.calls))

    def test_command_sends_pulses_without_a_force_argument(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.81]]), gripper_command_max=0.81)
        backend.close()

        gripper_call = next(call for call in fake.calls if call[0] == "set_gripper_position")
        # One closing step of gripper_max_step pulses away from fully open.
        self.assertEqual(gripper_call[1], 830)
        self.assertEqual(gripper_call[2]["speed"], 1500)
        self.assertNotIn("force", gripper_call[2])
        self.assertFalse(any(call[0] == "set_gripper_g2_position" for call in fake.calls))

    def test_inspection_reports_no_force_reading(self):
        backend, _fake = self.make_backend()
        status = backend.inspect()
        backend.close()

        # The classic controller exposes no force telemetry to report.
        self.assertIsNone(status.gripper_force)
        self.assertEqual(status.gripper_position, 850)

    def test_grasp_status_freezes_closing_until_an_open_command(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        closed = np.concatenate([self.reference, [0.81]])

        backend.command(closed, gripper_command_max=0.81)
        self.assertEqual(fake.gripper, 830)
        fake.gripper = 600
        fake.gripper_status = 2
        backend.command(closed, gripper_command_max=0.81)
        self.assertTrue(backend.gripper_contact_latched)
        self.assertEqual(fake.gripper, 600)

        gripper_calls = len([call for call in fake.calls if call[0] == "set_gripper_position"])
        fake.gripper_status = 0
        backend.command(closed, gripper_command_max=0.81)
        self.assertEqual(
            len([call for call in fake.calls if call[0] == "set_gripper_position"]),
            gripper_calls,
        )
        backend.close()


class ServoModeTests(unittest.TestCase):
    """Cover mode 1 servo streaming, which bypasses the trajectory planner."""

    def setUp(self):
        base = load_config().physical_xarm
        self.config = replace(
            base,
            robot_ip="192.0.2.1",
            watchdog_timeout=10.0,
            mode=1,
            rate=100.0,
            max_target_jump_degrees=1.5,
            catchup_step_degrees=0.8,
        )
        self.reference = np.deg2rad([0.0, -75.0, 10.0, 0.0, 60.0, 0.0])

    def make_backend(self):
        fake = FakeArm("192.0.2.1", joints=self.reference)
        return XArm6Hardware(self.config, api_factory=lambda *_a, **_k: fake), fake

    def test_mode_1_streams_through_servoj_without_planner_arguments(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)
        backend.close()

        self.assertIn(("set_mode", 1), fake.calls)
        call = next(c for c in fake.calls if c[0] == "set_servo_angle_j")
        # speed and mvacc are reserved in servoj; sending them would be misleading.
        self.assertNotIn("speed", call[2])
        self.assertNotIn("mvacc", call[2])
        self.assertTrue(call[2]["is_radian"])
        self.assertFalse(any(c[0] == "set_servo_angle" for c in fake.calls))

    def test_mode_6_still_uses_the_planning_interface(self):
        config = replace(self.config, mode=6, rate=50.0, max_target_jump_degrees=5.0)
        fake = FakeArm("192.0.2.1", joints=self.reference)
        backend = XArm6Hardware(config, api_factory=lambda *_a, **_k: fake)
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)
        backend.close()

        self.assertTrue(any(c[0] == "set_servo_angle" for c in fake.calls))
        self.assertFalse(any(c[0] == "set_servo_angle_j" for c in fake.calls))

    def test_servo_mode_still_enforces_the_jump_limit(self):
        # Without a planner the jump limit is the only bound on joint velocity.
        backend, _fake = self.make_backend()
        backend.arm_motion(self.reference)
        backend.command(np.concatenate([self.reference, [0.0]]), gripper_command_max=0.81)
        target = self.reference.copy()
        target[0] += np.deg2rad(3.0)
        with self.assertRaisesRegex(XArmHardwareError, "jumped"):
            backend.command(np.concatenate([target, [0.0]]), gripper_command_max=0.81)
        backend.close()


if __name__ == "__main__":
    unittest.main()
