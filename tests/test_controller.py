import threading
import time
import unittest

import numpy as np

from uarm_xarm6_teleop.backends.xarm import XArmStatus
from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.controller import (
    TeleopController,
    TeleopControllerError,
    TeleopState,
)
from uarm_xarm6_teleop.feetech import LeaderSample


class FakeLeader:
    def __init__(
        self,
        _serial,
        _leader,
        *,
        torque_ids=(),
        fail_event=None,
        step_radians=0.0,
    ):
        self.torque_enabled_ids = torque_ids
        self.fail_event = fail_event
        self.step_radians = step_radians
        self.opened = False
        self.closed = False
        self.read_count = 0

    def open(self):
        self.opened = True

    def read(self):
        self.read_count += 1
        if self.fail_event is not None and self.fail_event.is_set():
            raise OSError("leader sample failed")
        return LeaderSample(
            timestamp=time.monotonic(),
            positions=(2047,) * 7,
            radians=np.full(7, self.read_count * self.step_radians, dtype=float),
        )

    def close(self):
        self.closed = True


class FakeFollower:
    def __init__(self, config):
        self.config = config
        self.armed = False
        self.stopped = False
        self.closed = False
        self.commands = []
        self._gripper_contact_latched = False

    @property
    def gripper_contact_latched(self):
        return self._gripper_contact_latched

    def inspect(self):
        return XArmStatus(
            connected=True,
            version="fake-1.0",
            mode=6 if self.armed else 0,
            state=0,
            error_code=0,
            warning_code=0,
            joint_degrees=self.config_reference,
            gripper_position=84,
            gripper_force=20,
            gripper_status=0,
            gripper_error_code=0,
        )

    @property
    def config_reference(self):
        return (0.0, -75.0, 9.0, 0.0, 70.0, 0.0)

    def arm_motion(self, _target):
        self.armed = True
        return self.inspect()

    def command(self, action, gripper_command_max):
        self.commands.append((action.copy(), gripper_command_max))

    def safe_stop(self):
        self.stopped = True
        self.armed = False

    def close(self):
        self.safe_stop()
        self.closed = True


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.leaders = []
        self.followers = []

    def make_controller(self, *, torque_ids=(), fail_event=None, step_radians=0.0):
        def leader_factory(serial, leader):
            fake = FakeLeader(
                serial,
                leader,
                torque_ids=torque_ids,
                fail_event=fail_event,
                step_radians=step_radians,
            )
            self.leaders.append(fake)
            return fake

        def follower_factory(config):
            fake = FakeFollower(config)
            self.followers.append(fake)
            return fake

        return TeleopController(
            self.config,
            leader_factory=leader_factory,
            follower_factory=follower_factory,
        )

    @staticmethod
    def wait_for_state(controller, expected, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if controller.state == expected:
                return
            time.sleep(0.005)
        raise AssertionError(
            f"controller stayed in {controller.state.value}; expected {expected.value}"
        )

    @staticmethod
    def wait_for(predicate, description, timeout=1.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.005)
        raise AssertionError(f"timed out waiting for {description}")

    def test_connect_starts_continuous_read_only_leader_monitoring(self):
        controller = self.make_controller(step_radians=0.001)
        initial = controller.connect_leader()
        initial_degrees = initial.leader_degrees
        self.assertIsNotNone(initial_degrees)

        self.wait_for(lambda: self.leaders[0].read_count >= 4, "continuous leader samples")
        current = controller.snapshot()

        self.assertEqual(current.state, TeleopState.LEADER_READY.value)
        self.assertIsNone(current.mode)
        self.assertGreater(current.leader_degrees[0], initial_degrees[0])
        self.assertLess(current.last_sample_age_ms, 100.0)
        self.assertEqual(self.followers, [])
        controller.disconnect()

    def test_dry_run_lifecycle_never_opens_robot(self):
        controller = self.make_controller()
        controller.connect_leader()
        self.assertEqual(controller.state, TeleopState.LEADER_READY)

        controller.start("dry_run")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: self.leaders[0].read_count > 1, "a worker sample")
        controller.stop()

        self.assertEqual(controller.state, TeleopState.STOPPED)
        self.assertGreater(self.leaders[0].read_count, 1)
        self.assertEqual(self.followers, [])
        stopped_count = self.leaders[0].read_count
        self.wait_for(
            lambda: self.leaders[0].read_count > stopped_count,
            "leader monitoring to resume",
        )
        controller.disconnect()
        self.assertTrue(self.leaders[0].closed)
        self.assertEqual(controller.state, TeleopState.IDLE)

    def test_physical_start_requires_inspection_and_exact_ip_confirmation(self):
        controller = self.make_controller()
        controller.connect_leader()
        with self.assertRaisesRegex(TeleopControllerError, "Inspect"):
            controller.start("physical", confirmation="192.0.2.8")

        controller.inspect_robot("192.0.2.8")
        self.assertEqual(controller.state, TeleopState.READY)
        with self.assertRaisesRegex(TeleopControllerError, "did not match"):
            controller.start("physical", confirmation="192.0.2.9")

        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(self.followers[0].commands), "a follower command")
        snapshot = controller.snapshot()
        self.assertIsNotNone(snapshot.command_latency_ms)
        self.assertGreaterEqual(snapshot.command_latency_ms, 0.0)
        controller.stop()
        self.assertTrue(self.followers[0].stopped)
        self.assertGreater(len(self.followers[0].commands), 0)
        controller.close()

    def test_leader_torque_blocks_physical_but_not_dry_run(self):
        controller = self.make_controller(torque_ids=(1,))
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        with self.assertRaisesRegex(TeleopControllerError, "torque is enabled"):
            controller.start("physical", confirmation="192.0.2.8")

        controller.start("dry_run")
        self.wait_for_state(controller, TeleopState.RUNNING)
        controller.stop()
        controller.close()

    def test_worker_fault_safe_stops_physical_follower(self):
        fail_event = threading.Event()
        controller = self.make_controller(fail_event=fail_event)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        fail_event.set()
        self.wait_for_state(controller, TeleopState.FAULT)

        self.assertTrue(self.followers[0].stopped)
        self.assertIn("leader sample failed", controller.snapshot().fault)
        controller.reset_fault()
        self.assertEqual(controller.state, TeleopState.IDLE)

    def test_monitor_failure_enters_fault_without_opening_robot(self):
        fail_event = threading.Event()
        controller = self.make_controller(fail_event=fail_event)
        controller.connect_leader()
        fail_event.set()
        self.wait_for_state(controller, TeleopState.FAULT)

        snapshot = controller.snapshot()
        self.assertIn("leader sample failed", snapshot.fault)
        self.assertEqual(self.followers, [])
        controller.reset_fault()


if __name__ == "__main__":
    unittest.main()
