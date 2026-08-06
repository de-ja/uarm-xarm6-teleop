"""Thread-safe supervisory controller shared by the CLI and web application."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import replace
from types import TracebackType
from typing import Literal, Protocol, Self
from uuid import uuid4

import numpy as np

from .backends.maniskill import ManiSkillXArm6
from .backends.xarm import TargetSafety, XArm6Hardware, XArmHardwareError, XArmStatus
from .config import LeaderConfig, PhysicalXArmConfig, SerialConfig, TeleopConfig, validate_config
from .feetech import FeetechLeader, LeaderSample
from .event_log import EventSink
from .mapping import XArm6Mapping
from .protocol import (
    PROTOCOL_VERSION,
    ControllerEvent,
    RuntimeCapabilities,
    TeleopMode,
    TeleopSnapshot,
    TeleopState,
    default_runtime_capabilities,
)
from .scheduling import PeriodicScheduler, RateMeter


class TeleopControllerError(RuntimeError):
    """Raised when a requested supervisory transition is invalid or unsafe."""


class _Leader(Protocol):
    torque_enabled_ids: tuple[int, ...]

    def open(self) -> None: ...

    def read(self) -> LeaderSample: ...

    def close(self) -> None: ...


class _Follower(Protocol):
    @property
    def gripper_contact_latched(self) -> bool: ...

    def inspect(self) -> XArmStatus: ...

    def arm_motion(self, initial_target_radians: np.ndarray) -> XArmStatus: ...

    def command(self, action: np.ndarray, gripper_command_max: float) -> None: ...

    def safe_stop(self) -> None: ...

    def close(self) -> None: ...


class _Simulator(Protocol):
    def step(self, action: np.ndarray) -> None: ...

    def close(self) -> None: ...


LeaderFactory = Callable[[SerialConfig, LeaderConfig], _Leader]
FollowerFactory = Callable[[PhysicalXArmConfig], _Follower]
SimulationFactory = Callable[[str], _Simulator]


def make_mapping(config: TeleopConfig) -> XArm6Mapping:
    """Construct a stateful leader-to-xArm mapping from validated configuration."""
    return XArm6Mapping(
        reference_degrees=config.xarm6.reference_degrees,
        joint_directions=config.xarm6.joint_directions,
        gripper_travel_degrees=config.xarm6.gripper_travel_degrees,
        gripper_command_max=config.xarm6.gripper_command_max,
        gripper_mode=config.xarm6.gripper_mode,
        gripper_press_degrees=config.xarm6.gripper_press_degrees,
        gripper_release_degrees=config.xarm6.gripper_release_degrees,
    )


def require_safe_leader_start(config: TeleopConfig, sample: LeaderSample) -> None:
    """Require every leader arm joint to begin near its calibrated CAD pose.

    Args:
        config: Configuration containing the physical startup tolerance.
        sample: Current calibrated leader sample.

    Raises:
        XArmHardwareError: If any of the six arm joints exceeds the tolerance.
    """
    offsets = np.abs(sample.degrees[:6])
    joint = int(np.argmax(offsets))
    tolerance = config.physical_xarm.leader_start_tolerance_degrees
    if offsets[joint] > tolerance:
        raise XArmHardwareError(
            f"Leader J{joint + 1} is {sample.degrees[joint]:+.2f} deg from its calibrated "
            f"CAD pose; startup tolerance is {tolerance:.2f} deg"
        )


class TeleopController:
    """Own hardware and expose explicit, serialized teleoperation transitions.

    The controller is the safety boundary shared by CLI and web callers. Public
    operations are serialized separately from the telemetry lock so only one
    hardware lifecycle transition can proceed at a time.

    Args:
        config: Validated configuration for mapping, simulation, and hardware.
        leader_factory: Factory for a local or remote U-ARM reader.
        follower_factory: Factory for the physical xArm backend.
        simulation_factory: Factory for the visible simulation backend.
        monotonic: Clock used for scheduling and latency measurement.
        wall_time: Clock used for operator-visible timestamps.
        capabilities: Deployment features exposed in every telemetry snapshot.
        session_id: Stable session identifier, generated when omitted.
        event_sink: Optional nonblocking destination for structured controller events.
    """

    def __init__(
        self,
        config: TeleopConfig,
        *,
        leader_factory: LeaderFactory = FeetechLeader,
        follower_factory: FollowerFactory = XArm6Hardware,
        simulation_factory: SimulationFactory = ManiSkillXArm6,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
        capabilities: RuntimeCapabilities | None = None,
        session_id: str | None = None,
        event_sink: EventSink | None = None,
    ) -> None:
        self.config = validate_config(config)
        self._leader_factory = leader_factory
        self._follower_factory = follower_factory
        self._simulation_factory = simulation_factory
        self._monotonic = monotonic
        self._wall_time = wall_time
        self.capabilities = capabilities or default_runtime_capabilities()
        self.session_id = session_id or uuid4().hex
        self._event_sink = event_sink
        self._event_sink_closed = False
        self._lock = threading.RLock()
        self._operation_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._monitor_stop_event = threading.Event()
        self._monitor: threading.Thread | None = None

        self._state = TeleopState.IDLE
        self._mode: TeleopMode | None = None
        self._leader: _Leader | None = None
        self._follower: _Follower | None = None
        self._mapping = make_mapping(config)
        self._safety = TargetSafety(config.physical_xarm)
        self._physical_config = config.physical_xarm
        self._sample: LeaderSample | None = None
        self._action: np.ndarray | None = None
        self._robot_status: XArmStatus | None = None
        self._loop_rate_hz = 0.0
        self._command_latency_ms: float | None = None
        self._last_sample_received: float | None = None
        self._fault: str | None = None
        self._events: deque[ControllerEvent] = deque(maxlen=80)
        self._event("info", "Controller initialized; physical motion is disabled.")

    @property
    def state(self) -> TeleopState:
        """Return the current controller lifecycle state."""
        with self._lock:
            return self._state

    def _event(self, level: Literal["info", "warning", "error"], message: str) -> None:
        event = ControllerEvent(self._wall_time(), level, message)
        with self._lock:
            self._events.append(event)
        if self._event_sink is not None:
            try:
                self._event_sink.emit(self.session_id, event)
            except Exception:  # noqa: BLE001,S110 - logging cannot affect robot safety
                pass

    def _require_state(self, *allowed: TeleopState) -> None:
        if self._state not in allowed:
            choices = ", ".join(value.value for value in allowed)
            raise TeleopControllerError(
                f"Operation is unavailable in state {self._state.value}; expected {choices}"
            )

    def _update_sample(
        self,
        sample: LeaderSample,
        *,
        reset_safety: bool = False,
        validate_safety: bool = True,
    ) -> None:
        with self._lock:
            action = self._mapping.action(sample.radians)
            if reset_safety:
                self._safety.reset(action[:6])
            elif validate_safety:
                self._safety.validate(action[:6])
            self._sample = sample
            self._action = action
            self._last_sample_received = self._monotonic()

    def _start_leader_monitor(self) -> None:
        with self._lock:
            if (
                self._leader is None
                or self._state
                not in (TeleopState.LEADER_READY, TeleopState.READY, TeleopState.STOPPED)
                or (self._monitor is not None and self._monitor.is_alive())
            ):
                return
            self._monitor_stop_event.clear()
            monitor = threading.Thread(
                target=self._run_leader_monitor,
                name="uarm-leader-monitor",
                daemon=True,
            )
            self._monitor = monitor
            monitor.start()

    def _stop_leader_monitor(self, *, timeout: float = 2.0) -> None:
        with self._lock:
            monitor = self._monitor
            self._monitor_stop_event.set()
        if monitor is not None and monitor is not threading.current_thread():
            monitor.join(timeout=timeout)
            if monitor.is_alive():
                raise TeleopControllerError("Leader monitor did not stop within the timeout")
        with self._lock:
            if self._monitor is monitor:
                self._monitor = None

    def _run_leader_monitor(self) -> None:
        scheduler = PeriodicScheduler(
            self._physical_config.rate,
            monotonic=self._monotonic,
        )
        rate_meter = RateMeter(monotonic=self._monotonic)
        try:
            while not self._monitor_stop_event.is_set():
                with self._lock:
                    leader = self._leader
                if leader is None:
                    return
                sample = leader.read()
                self._update_sample(sample, validate_safety=False)

                now = self._monotonic()
                measured_rate = rate_meter.record(now)
                if measured_rate is not None:
                    with self._lock:
                        self._loop_rate_hz = measured_rate

                scheduler.wait(self._monitor_stop_event)
        except Exception as error:  # noqa: BLE001 - monitor boundary owns serial reads
            if not self._monitor_stop_event.is_set():
                with self._lock:
                    self._fault = str(error)
                    self._state = TeleopState.FAULT
                self._event("error", f"Leader monitoring failed: {error}")
        finally:
            with self._lock:
                if self._monitor is threading.current_thread():
                    self._monitor = None

    def connect_leader(self) -> TeleopSnapshot:
        """Open the configured leader, read a baseline sample, and start monitoring.

        Returns:
            Snapshot in the ``leader_ready`` state.

        Raises:
            TeleopControllerError: If the controller is not idle.
            FeetechError: If a local leader cannot be opened or sampled.
            RemoteLeaderError: If a remote leader cannot authenticate or respond.
        """
        with self._operation_lock:
            with self._lock:
                self._require_state(TeleopState.IDLE)
            leader = self._leader_factory(self.config.serial, self.config.leader)
            try:
                leader.open()
                sample = leader.read()
                self._update_sample(sample, reset_safety=True)
            except Exception:
                leader.close()
                raise

            with self._lock:
                self._leader = leader
                self._state = TeleopState.LEADER_READY
                torque_ids = leader.torque_enabled_ids
            self._event(
                "info",
                (
                    f"Leader connected through {leader.description}."
                    if getattr(leader, "description", None)
                    else f"Leader connected on {self.config.serial.device} at "
                    f"{self.config.serial.baudrate} baud."
                ),
            )
            if torque_ids:
                self._event("warning", f"Leader torque is enabled on IDs {torque_ids}.")
            self._start_leader_monitor()
            return self.snapshot()

    def inspect_robot(self, robot_ip: str) -> TeleopSnapshot:
        """Inspect a physical xArm without enabling motion.

        Args:
            robot_ip: Controller address reachable from the follower computer.

        Returns:
            Snapshot containing read-only robot and gripper status.

        Raises:
            TeleopControllerError: If the leader is unavailable or state is invalid.
            XArmHardwareError: If the robot cannot be verified as an xArm6.
        """
        robot_ip = robot_ip.strip()
        if not robot_ip:
            raise TeleopControllerError("Robot IP is required")
        with self._operation_lock:
            with self._lock:
                self._require_state(TeleopState.LEADER_READY, TeleopState.STOPPED)
                if self._leader is None or self._sample is None:
                    raise TeleopControllerError("Connect the leader before inspecting the robot")
            self._stop_leader_monitor()
            try:
                return self._inspect_robot(robot_ip)
            finally:
                self._start_leader_monitor()

    def _inspect_robot(self, robot_ip: str) -> TeleopSnapshot:
        with self._lock:
            self._require_state(TeleopState.LEADER_READY, TeleopState.STOPPED)
            assert self._leader is not None and self._sample is not None
            old_follower = self._follower
            self._follower = None
            self._robot_status = None
            self._state = TeleopState.LEADER_READY
        if old_follower is not None:
            old_follower.close()

        physical = replace(self.config.physical_xarm, robot_ip=robot_ip)
        config = validate_config(replace(self.config, physical_xarm=physical))
        follower = self._follower_factory(config.physical_xarm)
        try:
            status = follower.inspect()
        except Exception:
            follower.close()
            raise

        if config.xarm6.gripper_mode == "toggle":
            midpoint = (physical.gripper_open_position + physical.gripper_closed_position) / 2.0
            gripper_ratio = (physical.gripper_open_position - status.gripper_position) / (
                physical.gripper_open_position - physical.gripper_closed_position
            )
            with self._lock:
                assert self._sample is not None
                sample = self._sample
                self._mapping.reset_gripper(
                    float(sample.radians[6]),
                    closed=status.gripper_position < midpoint,
                    command=float(np.clip(gripper_ratio, 0.0, 1.0))
                    * config.xarm6.gripper_command_max,
                )
                self._update_sample(sample, reset_safety=True)

        with self._lock:
            self.config = config
            self._physical_config = physical
            self._safety = TargetSafety(physical)
            assert self._action is not None
            self._safety.reset(self._action[:6])
            self._follower = follower
            self._robot_status = status
            self._state = TeleopState.READY
        self._event("info", f"Robot {robot_ip} inspected read-only and is ready for checks.")
        return self.snapshot()

    def start(
        self,
        mode: TeleopMode,
        *,
        confirmation: str | None = None,
    ) -> TeleopSnapshot:
        """Start one dry-run, simulation, or physical control worker.

        Args:
            mode: Execution backend to run.
            confirmation: Robot IP required to authorize physical motion.

        Returns:
            Snapshot after the worker has entered its starting phase.

        Raises:
            TeleopControllerError: If prerequisites or the current state are invalid.
            XArmHardwareError: If physical startup safety checks fail.
        """
        if mode not in ("dry_run", "simulation", "physical"):
            raise TeleopControllerError(f"Unsupported teleoperation mode: {mode}")
        with self._operation_lock:
            with self._lock:
                self._require_state(
                    TeleopState.LEADER_READY, TeleopState.READY, TeleopState.STOPPED
                )
            self._stop_leader_monitor()
            try:
                return self._start(mode, confirmation=confirmation)
            except Exception:
                self._start_leader_monitor()
                raise

    def _start(
        self,
        mode: TeleopMode,
        *,
        confirmation: str | None,
    ) -> TeleopSnapshot:
        with self._lock:
            self._require_state(TeleopState.LEADER_READY, TeleopState.READY, TeleopState.STOPPED)
            if self._worker is not None and self._worker.is_alive():
                raise TeleopControllerError("A teleoperation worker is already running")
            if self._leader is None or self._sample is None or self._action is None:
                raise TeleopControllerError("The leader is not connected")
            if mode == "physical":
                if self._follower is None or not self._physical_config.robot_ip:
                    raise TeleopControllerError("Inspect the physical robot before starting")
                if confirmation is None or confirmation.strip() != self._physical_config.robot_ip:
                    raise TeleopControllerError(
                        "Confirmation did not match the inspected robot IP; motion remains disabled"
                    )
                if self._leader.torque_enabled_ids:
                    raise TeleopControllerError(
                        "Leader torque is enabled; physical teleoperation is blocked"
                    )
                require_safe_leader_start(self.config, self._sample)

            self._safety.reset(self._action[:6])
            self._mode = mode
            self._fault = None
            self._loop_rate_hz = 0.0
            self._command_latency_ms = None
            self._state = TeleopState.STARTING
            self._stop_event.clear()
            self._worker = threading.Thread(
                target=self._run_loop,
                name="uarm-teleop-controller",
                daemon=True,
            )
            self._worker.start()
        self._event(
            "warning" if mode == "physical" else "info",
            {
                "dry_run": "Starting dry run.",
                "simulation": "Starting visible ManiSkill simulation.",
                "physical": "Starting physical motion.",
            }[mode],
        )
        return self.snapshot()

    def _run_loop(self) -> None:
        mode = self._mode
        assert mode is not None
        simulator: _Simulator | None = None
        try:
            with self._lock:
                follower = self._follower
                action = None if self._action is None else self._action.copy()
            if mode == "simulation":
                simulator = self._simulation_factory(self.config.simulation.scene)
                if self._stop_event.is_set():
                    return
            if mode == "physical":
                assert follower is not None and action is not None
                status = follower.arm_motion(action[:6])
                with self._lock:
                    self._robot_status = status
                if self._stop_event.is_set():
                    return

            with self._lock:
                self._state = TeleopState.RUNNING
            self._event("info", f"Teleoperation is running in {mode.replace('_', ' ')} mode.")

            rate = (
                self.config.simulation.rate if mode == "simulation" else self._physical_config.rate
            )
            scheduler = PeriodicScheduler(rate, monotonic=self._monotonic)
            next_status_poll = self._monotonic()
            rate_meter = RateMeter(monotonic=self._monotonic)
            last_contact_state = False
            while not self._stop_event.is_set():
                with self._lock:
                    leader = self._leader
                    follower = self._follower
                if leader is None:
                    raise TeleopControllerError(
                        "Leader disconnected while teleoperation was active"
                    )
                sample = leader.read()
                self._update_sample(sample)
                with self._lock:
                    assert self._action is not None
                    action = self._action.copy()

                if mode == "physical":
                    assert follower is not None
                    follower.command(action, self.config.xarm6.gripper_command_max)
                    command_latency_ms = max(
                        0.0,
                        (self._monotonic() - sample.timestamp) * 1000.0,
                    )
                    with self._lock:
                        self._command_latency_ms = command_latency_ms
                    contact_state = follower.gripper_contact_latched
                    if contact_state and not last_contact_state:
                        self._event("warning", "G2 grasp detected; further closing is latched off.")
                    last_contact_state = contact_state
                elif mode == "simulation":
                    assert simulator is not None
                    simulator.step(action)

                now = self._monotonic()
                measured_rate = rate_meter.record(now)
                if measured_rate is not None:
                    with self._lock:
                        self._loop_rate_hz = measured_rate

                if mode == "physical" and follower is not None and now >= next_status_poll:
                    status = follower.inspect()
                    with self._lock:
                        self._robot_status = status
                    next_status_poll = now + 0.25

                scheduler.wait(self._stop_event)
        except Exception as error:  # noqa: BLE001 - worker boundary must fail safe
            with self._lock:
                self._fault = str(error)
                self._state = TeleopState.FAULT
            self._event("error", str(error))
        finally:
            with self._lock:
                follower = self._follower
            cleanup_errors = []
            if mode == "physical" and follower is not None:
                try:
                    follower.safe_stop()
                except Exception as error:  # noqa: BLE001 - cleanup must continue
                    cleanup_errors.append(f"physical stop failed: {error}")
            if simulator is not None:
                try:
                    simulator.close()
                except Exception as error:  # noqa: BLE001 - cleanup must continue
                    cleanup_errors.append(f"simulation close failed: {error}")
            if cleanup_errors:
                cleanup_fault = "; ".join(cleanup_errors)
                with self._lock:
                    self._fault = cleanup_fault
                    self._state = TeleopState.FAULT
                self._event("error", cleanup_fault)
            with self._lock:
                faulted = self._state == TeleopState.FAULT
                if not faulted:
                    self._state = TeleopState.STOPPED
                self._worker = None
            if not faulted:
                self._event("info", "Teleoperation stopped; physical motion is disabled.")
                self._start_leader_monitor()

    def stop(self, *, timeout: float = 2.0) -> TeleopSnapshot:
        """Request the active worker to stop and wait for bounded cleanup.

        Args:
            timeout: Maximum seconds to wait for the worker.

        Returns:
            Current snapshot, including a fault if the worker did not stop in time.
        """
        with self._operation_lock:
            with self._lock:
                worker = self._worker
                if self._state not in (
                    TeleopState.STARTING,
                    TeleopState.RUNNING,
                    TeleopState.STOPPING,
                ):
                    return self.snapshot()
                self._state = TeleopState.STOPPING
                self._stop_event.set()
            if worker is not None and worker is not threading.current_thread():
                worker.join(timeout=timeout)
                if worker.is_alive():
                    with self._lock:
                        self._fault = "Teleoperation worker did not stop within the timeout"
                        self._state = TeleopState.FAULT
                    self._event("error", self._fault)
            return self.snapshot()

    def disconnect(self) -> TeleopSnapshot:
        """Stop motion, close both hardware connections, and return to idle."""
        self.stop()
        with self._operation_lock:
            self._stop_leader_monitor()
            with self._lock:
                if self._worker is not None and self._worker.is_alive():
                    raise TeleopControllerError(
                        "Cannot disconnect while the teleoperation worker is still stopping"
                    )
                follower, leader = self._follower, self._leader
                self._follower = None
                self._leader = None
            if follower is not None:
                follower.close()
            if leader is not None:
                leader.close()
            with self._lock:
                self._state = TeleopState.IDLE
                self._mode = None
                self._sample = None
                self._action = None
                self._robot_status = None
                self._last_sample_received = None
                self._loop_rate_hz = 0.0
                self._command_latency_ms = None
                self._fault = None
                self._physical_config = self.config.physical_xarm
                self._mapping = make_mapping(self.config)
                self._safety = TargetSafety(self.config.physical_xarm)
            self._event("info", "Leader and robot connections closed.")
            return self.snapshot()

    def reset_fault(self) -> TeleopSnapshot:
        """Clear a fault by performing a full disconnect.

        Returns:
            Idle snapshot after resources are closed.

        Raises:
            TeleopControllerError: If the controller is not faulted.
        """
        with self._lock:
            self._require_state(TeleopState.FAULT)
        return self.disconnect()

    def snapshot(self) -> TeleopSnapshot:
        """Return a consistent immutable view of state and recent telemetry."""
        now_mono = self._monotonic()
        with self._lock:
            last_age = (
                None
                if self._last_sample_received is None
                else max(0.0, (now_mono - self._last_sample_received) * 1000.0)
            )
            leader_degrees = (
                None
                if self._sample is None
                else tuple(float(value) for value in self._sample.degrees)
            )
            target_degrees = (
                None
                if self._action is None
                else tuple(float(value) for value in np.rad2deg(self._action[:6]))
            )
            gripper_command = None if self._action is None else float(self._action[6])
            torque_ids = () if self._leader is None else self._leader.torque_enabled_ids
            return TeleopSnapshot(
                protocol_version=PROTOCOL_VERSION,
                session_id=self.session_id,
                capabilities=self.capabilities,
                timestamp=self._wall_time(),
                state=self._state,
                mode=self._mode,
                leader_connected=self._leader is not None,
                robot_connected=self._follower is not None,
                robot_ip=self._physical_config.robot_ip,
                torque_enabled_ids=torque_ids,
                leader_degrees=leader_degrees,
                target_degrees=target_degrees,
                gripper_command=gripper_command,
                robot_status=self._robot_status,
                loop_rate_hz=self._loop_rate_hz,
                command_latency_ms=self._command_latency_ms,
                last_sample_age_ms=last_age,
                fault=self._fault,
                events=tuple(self._events),
            )

    def close(self) -> None:
        """Best-effort shutdown for process exit and context-manager cleanup."""
        try:
            self.disconnect()
        except Exception:  # noqa: BLE001,S110 - process shutdown is best-effort
            # Shutdown remains best-effort; the hardware backend already requests state 4.
            pass
        with self._lock:
            sink = None if self._event_sink_closed else self._event_sink
            self._event_sink_closed = True
        if sink is not None:
            try:
                sink.close()
            except Exception:  # noqa: BLE001,S110 - shutdown remains best-effort
                pass

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.close()
