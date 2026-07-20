"""Guarded UFACTORY xArm6 and standard xArm Gripper backend."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Protocol

import numpy as np

from ..config import PhysicalXArmConfig


class XArmHardwareError(RuntimeError):
    """Raised when a physical command cannot be proven safe to send."""


class _XArmAPI(Protocol):
    connected: bool
    version: str
    axis: int
    mode: int
    state: int


@dataclass(frozen=True)
class XArmStatus:
    connected: bool
    version: str
    mode: int
    state: int
    error_code: int
    warning_code: int
    joint_degrees: tuple[float, ...]
    gripper_position: int


class TargetSafety:
    """Validate mapped targets independently of the physical SDK."""

    def __init__(self, config: PhysicalXArmConfig):
        self.config = config
        self._previous: np.ndarray | None = None

    def reset(self, target_radians: np.ndarray | None = None) -> None:
        self._previous = None
        if target_radians is not None:
            self.validate(target_radians, check_jump=False)

    def validate(self, target_radians: np.ndarray, *, check_jump: bool = True) -> None:
        target = np.asarray(target_radians, dtype=float)
        if target.shape != (6,):
            raise XArmHardwareError("xArm target must contain exactly six joints")
        if not np.all(np.isfinite(target)):
            raise XArmHardwareError("xArm target contains a non-finite value")

        degrees = np.rad2deg(target)
        lower = np.asarray(self.config.joint_lower_degrees)
        upper = np.asarray(self.config.joint_upper_degrees)
        outside = np.flatnonzero((degrees < lower) | (degrees > upper))
        if outside.size:
            joint = int(outside[0])
            raise XArmHardwareError(
                f"J{joint + 1} target {degrees[joint]:.2f} deg is outside "
                f"[{lower[joint]:.2f}, {upper[joint]:.2f}] deg"
            )

        if check_jump and self._previous is not None:
            jumps = np.abs(np.rad2deg(target - self._previous))
            joint = int(np.argmax(jumps))
            if jumps[joint] > self.config.max_target_jump_degrees:
                raise XArmHardwareError(
                    f"J{joint + 1} target jumped {jumps[joint]:.2f} deg; limit is "
                    f"{self.config.max_target_jump_degrees:.2f} deg per sample"
                )
        self._previous = target.copy()


class XArm6Hardware:
    """Physical follower that remains read-only until :meth:`arm_motion` succeeds."""

    def __init__(
        self,
        config: PhysicalXArmConfig,
        api_factory: Callable[..., _XArmAPI] | None = None,
    ):
        if not config.robot_ip:
            raise XArmHardwareError("No xArm IP configured; pass --robot-ip")
        if api_factory is None:
            try:
                from xarm.wrapper import XArmAPI
            except ImportError as error:  # pragma: no cover - host dependency
                raise XArmHardwareError(
                    "The xArm SDK is missing. Install with `pip install -e '.[physical]'`."
                ) from error
            api_factory = XArmAPI

        self.config = config
        self.arm = api_factory(config.robot_ip, is_radian=True)
        self.safety = TargetSafety(config)
        self._lock = threading.RLock()
        self._armed = False
        self._watchdog_tripped = False
        self._last_command_time = 0.0
        self._last_gripper_position: int | None = None
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    @staticmethod
    def _check_code(operation: str, code: int) -> None:
        if code != 0:
            raise XArmHardwareError(f"xArm {operation} failed with SDK code {code}")

    def inspect(self) -> XArmStatus:
        """Read robot, controller, joints, and gripper without enabling motion."""
        with self._lock:
            if not bool(getattr(self.arm, "connected", False)):
                raise XArmHardwareError("xArm is not connected")

            code, state = self.arm.get_state()
            self._check_code("get_state", code)
            code, errors = self.arm.get_err_warn_code()
            self._check_code("get_err_warn_code", code)
            code, joints = self.arm.get_servo_angle(is_radian=True)
            self._check_code("get_servo_angle", code)
            axis = int(getattr(self.arm, "axis", len(joints)))
            if axis != 6:
                raise XArmHardwareError(
                    f"Expected an xArm6, but the controller reported {axis} axes"
                )
            if len(joints) < axis:
                raise XArmHardwareError("The controller returned an incomplete joint sample")
            joints = joints[:axis]
            code, gripper = self.arm.get_gripper_position()
            self._check_code("get_gripper_position", code)

            return XArmStatus(
                connected=True,
                version=str(getattr(self.arm, "version", "unknown")),
                mode=int(getattr(self.arm, "mode", -1)),
                state=int(state),
                error_code=int(errors[0]),
                warning_code=int(errors[1]),
                joint_degrees=tuple(float(value) for value in np.rad2deg(joints)),
                gripper_position=int(gripper),
            )

    def arm_motion(self, initial_target_radians: np.ndarray) -> XArmStatus:
        """Enable mode 6 only after status, limit, and alignment checks pass."""
        target = np.asarray(initial_target_radians, dtype=float)
        self.safety.reset(target)
        status = self.inspect()
        if status.error_code or status.warning_code:
            raise XArmHardwareError(
                f"Controller reports error={status.error_code}, warning={status.warning_code}; "
                "resolve it in xArm Studio before teleoperation"
            )

        target_degrees = np.rad2deg(target)
        actual_degrees = np.asarray(status.joint_degrees)
        mismatch = np.abs(target_degrees - actual_degrees)
        joint = int(np.argmax(mismatch))
        if mismatch[joint] > self.config.startup_tolerance_degrees:
            raise XArmHardwareError(
                f"xArm J{joint + 1} is {actual_degrees[joint]:.2f} deg but the leader "
                f"requests {target_degrees[joint]:.2f} deg (startup tolerance "
                f"{self.config.startup_tolerance_degrees:.2f} deg). Align the robot "
                "manually in xArm Studio; this program will not move it into place."
            )

        code, at_limit = self.arm.is_joint_limit(target.tolist(), is_radian=True)
        self._check_code("is_joint_limit", code)
        if at_limit is not False:
            raise XArmHardwareError("The xArm controller rejected the initial joint target")

        try:
            self._check_code("motion_enable", self.arm.motion_enable(enable=True))
            self._check_code("set_mode", self.arm.set_mode(self.config.mode))
            self._check_code("set_state", self.arm.set_state(0))
            self._check_code("set_gripper_mode", self.arm.set_gripper_mode(0))
            self._check_code(
                "set_gripper_enable", self.arm.set_gripper_enable(enable=True)
            )
        except Exception:
            self._best_effort_stop()
            raise

        with self._lock:
            self._armed = True
            self._watchdog_tripped = False
            self._last_command_time = time.monotonic()
            self._last_gripper_position = status.gripper_position
        self._start_watchdog()
        return status

    def command(self, action: np.ndarray, gripper_command_max: float) -> None:
        values = np.asarray(action, dtype=float)
        if values.shape != (7,) or not np.all(np.isfinite(values)):
            raise XArmHardwareError("Physical action must contain seven finite values")
        if gripper_command_max <= 0:
            raise XArmHardwareError("gripper_command_max must be positive")

        with self._lock:
            if self._watchdog_tripped:
                raise XArmHardwareError("Command watchdog tripped; restart teleoperation")
            if not self._armed:
                raise XArmHardwareError("Physical motion is not armed")
            if not bool(getattr(self.arm, "connected", False)):
                raise XArmHardwareError("xArm disconnected")

            code, state = self.arm.get_state()
            self._check_code("get_state", code)
            if state not in (0, 1):
                raise XArmHardwareError(f"xArm entered non-motion state {state}")
            code, errors = self.arm.get_err_warn_code()
            self._check_code("get_err_warn_code", code)
            if errors[0] or errors[1]:
                raise XArmHardwareError(
                    f"Controller reports error={errors[0]}, warning={errors[1]}"
                )

            joints = values[:6]
            self.safety.validate(joints)
            code, at_limit = self.arm.is_joint_limit(joints.tolist(), is_radian=True)
            self._check_code("is_joint_limit", code)
            if at_limit is not False:
                raise XArmHardwareError("The xArm controller rejected a joint target")

            ratio = float(np.clip(values[6] / gripper_command_max, 0.0, 1.0))
            desired_gripper = round(
                self.config.gripper_open_position
                + ratio
                * (
                    self.config.gripper_closed_position
                    - self.config.gripper_open_position
                )
            )
            assert self._last_gripper_position is not None
            gripper_delta = int(
                np.clip(
                    desired_gripper - self._last_gripper_position,
                    -self.config.gripper_max_step,
                    self.config.gripper_max_step,
                )
            )
            gripper_position = self._last_gripper_position + gripper_delta

            self._check_code(
                "set_servo_angle",
                self.arm.set_servo_angle(
                    angle=joints.tolist(),
                    speed=float(np.deg2rad(self.config.joint_speed_degrees)),
                    mvacc=float(np.deg2rad(self.config.joint_acceleration_degrees)),
                    is_radian=True,
                    wait=False,
                ),
            )
            if gripper_position != self._last_gripper_position:
                self._check_code(
                    "set_gripper_position",
                    self.arm.set_gripper_position(
                        gripper_position,
                        wait=False,
                        speed=self.config.gripper_speed,
                        auto_enable=False,
                    ),
                )
                self._last_gripper_position = gripper_position
            self._last_command_time = time.monotonic()

    def _start_watchdog(self) -> None:
        self._watchdog_stop.clear()
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="xarm-command-watchdog",
            daemon=True,
        )
        self._watchdog_thread.start()

    def _watchdog_loop(self) -> None:
        interval = min(0.05, self.config.watchdog_timeout / 4.0)
        while not self._watchdog_stop.wait(interval):
            with self._lock:
                expired = (
                    self._armed
                    and time.monotonic() - self._last_command_time
                    > self.config.watchdog_timeout
                )
                if expired:
                    self.arm.set_state(4)
                    self._watchdog_tripped = True
                    self._armed = False
                    return

    def _best_effort_stop(self) -> None:
        if bool(getattr(self.arm, "connected", False)):
            try:
                self.arm.set_state(4)
            except Exception:
                pass

    def safe_stop(self) -> None:
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            if self._armed:
                self._best_effort_stop()
            self._armed = False

    def close(self) -> None:
        self.safe_stop()
        if bool(getattr(self.arm, "connected", False)):
            self.arm.disconnect()

    def __enter__(self) -> "XArm6Hardware":
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
