"""Thread-safe supervisory controller shared by the CLI and web application."""

from __future__ import annotations

import threading
import time
from collections import deque
from collections.abc import Callable
from dataclasses import asdict, dataclass, replace
from enum import Enum
from typing import Literal, Protocol, Self

import numpy as np

from .backends.xarm import TargetSafety, XArm6Hardware, XArmHardwareError, XArmStatus
from .config import TeleopConfig, validate_config
from .feetech import FeetechLeader, LeaderSample
from .mapping import XArm6Mapping


class TeleopControllerError(RuntimeError):
    """Raised when a requested supervisory transition is invalid or unsafe."""


class TeleopState(str, Enum):
    IDLE = "idle"
    LEADER_READY = "leader_ready"
    READY = "ready"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAULT = "fault"


TeleopMode = Literal["dry_run", "physical"]


@dataclass(frozen=True)
class ControllerEvent:
    timestamp: float
    level: Literal["info", "warning", "error"]
    message: str


@dataclass(frozen=True)
class TeleopSnapshot:
    protocol_version: int
    timestamp: float
    state: str
    mode: str | None
    leader_connected: bool
    robot_connected: bool
    robot_ip: str
    torque_enabled_ids: tuple[int, ...]
    leader_degrees: tuple[float, ...] | None
    target_degrees: tuple[float, ...] | None
    gripper_command: float | None
    robot_status: dict[str, object] | None
    loop_rate_hz: float
    last_sample_age_ms: float | None
    fault: str | None
    events: tuple[ControllerEvent, ...]

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


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


LeaderFactory = Callable[[object, object], _Leader]
FollowerFactory = Callable[[object], _Follower]


def make_mapping(config: TeleopConfig) -> XArm6Mapping:
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
    offsets = np.abs(sample.degrees[:6])
    joint = int(np.argmax(offsets))
    tolerance = config.physical_xarm.leader_start_tolerance_degrees
    if offsets[joint] > tolerance:
        raise XArmHardwareError(
            f"Leader J{joint + 1} is {sample.degrees[joint]:+.2f} deg from its calibrated "
            f"CAD pose; startup tolerance is {tolerance:.2f} deg"
        )


class TeleopController:
    """Own all hardware and expose explicit, serialized teleoperation transitions."""

    def __init__(
        self,
        config: TeleopConfig,
        *,
        leader_factory: LeaderFactory = FeetechLeader,
        follower_factory: FollowerFactory = XArm6Hardware,
        monotonic: Callable[[], float] = time.monotonic,
        wall_time: Callable[[], float] = time.time,
    ):
        self.config = validate_config(config)
        self._leader_factory = leader_factory
        self._follower_factory = follower_factory
        self._monotonic = monotonic
        self._wall_time = wall_time
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
        self._last_sample_received: float | None = None
        self._fault: str | None = None
        self._events: deque[ControllerEvent] = deque(maxlen=80)
        self._event("info", "Controller initialized; physical motion is disabled.")

    @property
    def state(self) -> TeleopState:
        with self._lock:
            return self._state

    def _event(self, level: Literal["info", "warning", "error"], message: str) -> None:
        with self._lock:
            self._events.append(ControllerEvent(self._wall_time(), level, message))

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
        period = 1.0 / self._physical_config.rate
        next_step = self._monotonic()
        rate_started = next_step
        rate_samples = 0
        try:
            while not self._monitor_stop_event.is_set():
                with self._lock:
                    leader = self._leader
                if leader is None:
                    return
                sample = leader.read()
                self._update_sample(sample, validate_safety=False)

                now = self._monotonic()
                rate_samples += 1
                elapsed = now - rate_started
                if elapsed >= 0.5:
                    with self._lock:
                        self._loop_rate_hz = rate_samples / elapsed
                    rate_started = now
                    rate_samples = 0

                next_step += period
                delay = next_step - self._monotonic()
                if delay > 0:
                    self._monitor_stop_event.wait(delay)
                else:
                    next_step = self._monotonic()
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
                f"Leader connected on {self.config.serial.device} at "
                f"{self.config.serial.baudrate} baud.",
            )
            if torque_ids:
                self._event("warning", f"Leader torque is enabled on IDs {torque_ids}.")
            self._start_leader_monitor()
            return self.snapshot()

    def inspect_robot(self, robot_ip: str) -> TeleopSnapshot:
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
        if mode not in ("dry_run", "physical"):
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
            "Starting physical motion." if mode == "physical" else "Starting dry run.",
        )
        return self.snapshot()

    def _run_loop(self) -> None:
        mode = self._mode
        assert mode is not None
        try:
            with self._lock:
                follower = self._follower
                action = None if self._action is None else self._action.copy()
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

            period = 1.0 / self._physical_config.rate
            next_step = self._monotonic()
            next_status_poll = next_step
            rate_started = next_step
            rate_samples = 0
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
                    contact_state = follower.gripper_contact_latched
                    if contact_state and not last_contact_state:
                        self._event("warning", "G2 grasp detected; further closing is latched off.")
                    last_contact_state = contact_state

                now = self._monotonic()
                rate_samples += 1
                elapsed = now - rate_started
                if elapsed >= 0.5:
                    with self._lock:
                        self._loop_rate_hz = rate_samples / elapsed
                    rate_started = now
                    rate_samples = 0

                if mode == "physical" and follower is not None and now >= next_status_poll:
                    status = follower.inspect()
                    with self._lock:
                        self._robot_status = status
                    next_status_poll = now + 0.25

                next_step += period
                delay = next_step - self._monotonic()
                if delay > 0:
                    self._stop_event.wait(delay)
                else:
                    next_step = self._monotonic()
        except Exception as error:  # noqa: BLE001 - worker boundary must fail safe
            with self._lock:
                self._fault = str(error)
                self._state = TeleopState.FAULT
            self._event("error", str(error))
        finally:
            with self._lock:
                follower = self._follower
                faulted = self._state == TeleopState.FAULT
            if mode == "physical" and follower is not None:
                follower.safe_stop()
            with self._lock:
                if not faulted:
                    self._state = TeleopState.STOPPED
                self._worker = None
            if not faulted:
                self._event("info", "Teleoperation stopped; physical motion is disabled.")
                self._start_leader_monitor()

    def stop(self, *, timeout: float = 2.0) -> TeleopSnapshot:
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
                self._fault = None
                self._physical_config = self.config.physical_xarm
                self._mapping = make_mapping(self.config)
                self._safety = TargetSafety(self.config.physical_xarm)
            self._event("info", "Leader and robot connections closed.")
            return self.snapshot()

    def reset_fault(self) -> TeleopSnapshot:
        with self._lock:
            self._require_state(TeleopState.FAULT)
        return self.disconnect()

    def snapshot(self) -> TeleopSnapshot:
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
            status = None if self._robot_status is None else asdict(self._robot_status)
            torque_ids = () if self._leader is None else self._leader.torque_enabled_ids
            return TeleopSnapshot(
                protocol_version=1,
                timestamp=self._wall_time(),
                state=self._state.value,
                mode=self._mode,
                leader_connected=self._leader is not None,
                robot_connected=self._follower is not None,
                robot_ip=self._physical_config.robot_ip,
                torque_enabled_ids=torque_ids,
                leader_degrees=leader_degrees,
                target_degrees=target_degrees,
                gripper_command=gripper_command,
                robot_status=status,
                loop_rate_hz=self._loop_rate_hz,
                last_sample_age_ms=last_age,
                fault=self._fault,
                events=tuple(self._events),
            )

    def close(self) -> None:
        try:
            self.disconnect()
        except Exception:  # noqa: BLE001,S110 - process shutdown is best-effort
            # Shutdown remains best-effort; the hardware backend already requests state 4.
            pass

    def __enter__(self) -> Self:
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
