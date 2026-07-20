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
        self.gripper = 730
        self.calls = []

    def get_state(self):
        return 0, self.state

    def get_err_warn_code(self):
        return 0, self.errors

    def get_servo_angle(self, is_radian=True):
        return 0, self.joints

    def get_gripper_position(self):
        return 0, self.gripper

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

    def test_inspection_never_enables_motion(self):
        backend, fake = self.make_backend()
        status = backend.inspect()
        backend.close()
        self.assertEqual(status.joint_degrees[1], -75.0)
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

    def test_arm_and_command_use_mode_6_and_rate_limit_gripper(self):
        backend, fake = self.make_backend()
        backend.arm_motion(self.reference)
        action = np.concatenate([self.reference, [0.81]])
        backend.command(action, gripper_command_max=0.81)
        backend.close()

        self.assertIn(("set_mode", 6), fake.calls)
        servo_call = next(call for call in fake.calls if call[0] == "set_servo_angle")
        self.assertFalse(servo_call[1]["wait"])
        self.assertTrue(servo_call[1]["is_radian"])
        gripper_call = next(
            call for call in fake.calls if call[0] == "set_gripper_position"
        )
        self.assertEqual(gripper_call[1], 690)
        self.assertIn(("set_state", 4), fake.calls)


if __name__ == "__main__":
    unittest.main()
