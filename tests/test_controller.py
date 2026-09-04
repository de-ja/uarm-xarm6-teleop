import threading
import time
import unittest
from dataclasses import replace

import numpy as np

from uarm_xarm6_teleop.backends.xarm import XArmStatus
from uarm_xarm6_teleop.config import load_config
from uarm_xarm6_teleop.controller import (
    TeleopController,
    TeleopControllerError,
    TeleopState,
)
from uarm_xarm6_teleop.feetech import LeaderSample
from uarm_xarm6_teleop.remote_leader import RemoteLeaderTimeout


class FakeLeader:
    def __init__(
        self,
        _serial,
        _leader,
        *,
        torque_ids=(),
        fail_event=None,
        step_radians=0.0,
        timeout_event=None,
        timeout_budget=0,
    ):
        self.torque_enabled_ids = torque_ids
        self.fail_event = fail_event
        self.step_radians = step_radians
        self.timeout_event = timeout_event
        self.timeouts_remaining = timeout_budget
        self.opened = False
        self.closed = False
        self.read_count = 0
        self.timeout_count = 0

    def open(self):
        self.opened = True

    def read(self):
        self.read_count += 1
        if (
            self.timeout_event is not None
            and self.timeout_event.is_set()
            and self.timeouts_remaining > 0
        ):
            self.timeouts_remaining -= 1
            self.timeout_count += 1
            raise RemoteLeaderTimeout("timed out in 0.06s")
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
        self._catching_up = False
        self.catch_up_calls = 0

    @property
    def gripper_contact_latched(self):
        return self._gripper_contact_latched

    @property
    def catching_up(self):
        return self._catching_up

    def begin_catch_up(self):
        self.catch_up_calls += 1
        self._catching_up = True

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
        return (0.0, -75.0, 9.0, 0.0, 0.0, 0.0)

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


class FakeSimulator:
    def __init__(self, scene):
        self.scene = scene
        self.steps = []
        self.closed = False

    def step(self, action):
        self.steps.append(action.copy())

    def close(self):
        self.closed = True


class FakeEventSink:
    def __init__(self):
        self.records = []
        self.closed = False
        self.metrics = []

    def emit(self, session_id, event):
        self.records.append((session_id, event))

    def emit_metrics(self, session_id, metrics):
        self.metrics.append((session_id, metrics))

    def close(self):
        self.closed = True


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.config = load_config()
        self.leaders = []
        self.followers = []
        self.simulators = []

    def make_controller(
        self,
        *,
        torque_ids=(),
        fail_event=None,
        step_radians=0.0,
        event_sink=None,
        timeout_event=None,
        timeout_budget=0,
    ):
        def leader_factory(serial, leader):
            fake = FakeLeader(
                serial,
                leader,
                torque_ids=torque_ids,
                fail_event=fail_event,
                step_radians=step_radians,
                timeout_event=timeout_event,
                timeout_budget=timeout_budget,
            )
            self.leaders.append(fake)
            return fake

        def follower_factory(config):
            fake = FakeFollower(config)
            self.followers.append(fake)
            return fake

        def simulation_factory(scene):
            fake = FakeSimulator(scene)
            self.simulators.append(fake)
            return fake

        return TeleopController(
            self.config,
            leader_factory=leader_factory,
            follower_factory=follower_factory,
            simulation_factory=simulation_factory,
            event_sink=event_sink,
            session_id="test-session",
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

    def test_controller_events_use_one_session_and_close_the_sink(self):
        sink = FakeEventSink()
        controller = self.make_controller(event_sink=sink)

        controller.connect_leader()
        controller.close()

        self.assertGreaterEqual(len(sink.records), 3)
        self.assertEqual({session_id for session_id, _event in sink.records}, {"test-session"})
        self.assertTrue(sink.closed)

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

    def test_simulation_steps_visible_follower_without_opening_robot(self):
        controller = self.make_controller()
        controller.connect_leader()

        controller.start("simulation")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(self.simulators[0].steps), "a simulation step")
        running = controller.snapshot()
        controller.stop()

        self.assertEqual(running.mode, "simulation")
        self.assertEqual(self.simulators[0].scene, self.config.simulation.scene)
        self.assertEqual(self.simulators[0].steps[0].shape, (7,))
        self.assertTrue(self.simulators[0].closed)
        self.assertEqual(self.followers, [])
        controller.disconnect()

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

    def test_run_logs_periodic_metrics_with_stage_latencies(self):
        sink = FakeEventSink()
        controller = self.make_controller(event_sink=sink)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(sink.metrics), "a metrics sample")
        controller.stop()
        controller.close()

        session_id, sample = sink.metrics[0]
        self.assertEqual(session_id, "test-session")
        self.assertEqual(sample["mode"], "physical")
        self.assertIn("loop_rate_hz", sample)
        self.assertIn("leader_timeouts", sample)
        # The fake leader reports no transport timing, so only the stages this
        # process measured itself are present.
        self.assertIsNotNone(sample["mapping_ms"])
        self.assertIsNotNone(sample["robot_command_ms"])

    def test_run_absorbs_tolerated_leader_timeouts(self):
        # Three consecutive misses is exactly the configured budget.
        timeout_event = threading.Event()
        controller = self.make_controller(timeout_event=timeout_event, timeout_budget=3)
        controller.connect_leader()
        controller.start("dry_run")
        self.wait_for_state(controller, TeleopState.RUNNING)
        timeout_event.set()
        self.wait_for(lambda: self.leaders[0].timeouts_remaining == 0, "the timeout burst")
        sampled = self.leaders[0].read_count
        self.wait_for(lambda: self.leaders[0].read_count > sampled, "recovery after the burst")

        self.assertEqual(controller.state, TeleopState.RUNNING)
        self.assertEqual(self.leaders[0].timeout_count, 3)
        controller.stop()
        self.assertEqual(controller.state, TeleopState.STOPPED)

    def test_run_faults_once_the_timeout_budget_is_exceeded(self):
        # One miss beyond the configured budget must end the run.
        timeout_event = threading.Event()
        controller = self.make_controller(timeout_event=timeout_event, timeout_budget=5)
        controller.connect_leader()
        controller.start("dry_run")
        self.wait_for_state(controller, TeleopState.RUNNING)
        timeout_event.set()
        self.wait_for_state(controller, TeleopState.FAULT)
        self.assertIn("timed out", controller.snapshot().fault)

    def test_tolerated_timeouts_send_no_physical_command(self):
        timeout_event = threading.Event()
        controller = self.make_controller(timeout_event=timeout_event, timeout_budget=3)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(self.followers[0].commands), "a follower command")

        commanded = len(self.followers[0].commands)
        timeout_event.set()
        self.wait_for(lambda: self.leaders[0].timeouts_remaining == 0, "the timeout burst")
        # No command may be issued for a sample that never arrived.
        self.assertLessEqual(len(self.followers[0].commands), commanded + 1)

        self.wait_for(
            lambda: len(self.followers[0].commands) > commanded + 1,
            "commands to resume after recovery",
        )
        self.assertEqual(controller.state, TeleopState.RUNNING)
        controller.stop()
        controller.close()

    def test_recovery_puts_the_physical_follower_into_catch_up(self):
        timeout_event = threading.Event()
        controller = self.make_controller(timeout_event=timeout_event, timeout_budget=3)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(self.followers[0].commands), "a follower command")

        timeout_event.set()
        self.wait_for(lambda: self.leaders[0].timeouts_remaining == 0, "the timeout burst")
        self.wait_for(lambda: self.followers[0].catch_up_calls == 1, "catch-up to begin")

        messages = [event.message for event in controller.snapshot().events]
        self.assertTrue(any("slewing toward the leader" in message for message in messages))
        controller.stop()
        controller.close()

    def test_catch_up_is_not_requested_without_a_gap(self):
        controller = self.make_controller()
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: len(self.followers[0].commands) > 3, "several follower commands")
        controller.stop()

        self.assertEqual(self.followers[0].catch_up_calls, 0)
        controller.close()

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

    def test_a_loop_running_below_its_configured_rate_is_reported(self):
        # In servo mode the loop rate is what turns a per-sample jump limit into
        # a velocity limit, so a silent shortfall has no other visible symptom.
        # The fakes run a cycle in microseconds, so the configured rate has to be
        # far beyond any achievable one to make the shortfall deterministic.
        self.config = replace(
            self.config, physical_xarm=replace(self.config.physical_xarm, rate=1_000_000.0)
        )
        sink = FakeEventSink()
        controller = self.make_controller(event_sink=sink)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(
            lambda: any("Control loop is running" in record.message for _, record in sink.records),
            "a slow loop warning",
            timeout=3.0,
        )
        controller.stop()
        controller.close()

        warnings = [r for _, r in sink.records if "Control loop is running" in r.message]
        self.assertEqual(warnings[0].level, "warning")
        # Reported once on entry, not once per cycle.
        self.assertEqual(len(warnings), 1)

    def test_a_loop_meeting_its_rate_reports_nothing(self):
        sink = FakeEventSink()
        controller = self.make_controller(event_sink=sink)
        controller.connect_leader()
        controller.inspect_robot("192.0.2.8")
        controller.start("physical", confirmation="192.0.2.8")
        self.wait_for_state(controller, TeleopState.RUNNING)
        self.wait_for(lambda: bool(sink.metrics), "a metrics sample")
        controller.stop()
        controller.close()

        self.assertFalse([r for _, r in sink.records if "Control loop is running" in r.message])


if __name__ == "__main__":
    unittest.main()
