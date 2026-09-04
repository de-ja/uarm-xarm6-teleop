"""Guarded UFACTORY xArm6 backend for the xArm Gripper and Gripper G2."""

from __future__ import annotations

import threading
import time
from abc import ABC, abstractmethod
from collections.abc import Callable
from dataclasses import dataclass
from types import TracebackType
from typing import Protocol, Self

import numpy as np

from ..config import PhysicalXArmConfig


class XArmHardwareError(RuntimeError):
    """Raised when a physical command cannot be proven safe to send."""


# A stalled loop must not bank acceleration budget and release it at once, so
# the measured cycle time is credited only up to this many nominal periods.
_MAX_ACCELERATION_PERIODS = 4.0


def _check_code(operation: str, code: int) -> None:
    """Raise when an SDK call reports a non-zero status code.

    Args:
        operation: SDK method name to quote in the error message.
        code: Status code the SDK returned.

    Raises:
        XArmHardwareError: If the code is not zero.
    """
    if code != 0:
        raise XArmHardwareError(f"xArm {operation} failed with SDK code {code}")


class _XArmAPI(Protocol):
    connected: bool
    version: str
    axis: int
    mode: int
    state: int


@dataclass(frozen=True)
class XArmStatus:
    """Capture a read-only snapshot of xArm, controller, and gripper state."""

    connected: bool
    version: str
    mode: int
    state: int
    error_code: int
    warning_code: int
    joint_degrees: tuple[float, ...]
    gripper_position: int
    gripper_force: int | None
    gripper_status: int | None
    gripper_error_code: int


class _GripperDriver(ABC):
    """Adapt one UFACTORY gripper family to the calls the follower makes.

    The two families disagree on units, so every position and speed handled
    here is in the units of the configured family and never interchangeable.
    """

    def __init__(self, arm: _XArmAPI, config: PhysicalXArmConfig) -> None:
        self.arm = arm
        self.config = config

    def enable(self) -> None:
        """Prepare the gripper to accept position commands."""

    @abstractmethod
    def read_position(self) -> int:
        """Return the measured jaw position in this family's units."""

    def read_force(self) -> int | None:
        """Return the measured grip force, or None when the family lacks it."""
        return None

    @abstractmethod
    def move_to(self, position: int) -> None:
        """Command an absolute jaw position in this family's units."""


class _G2Gripper(_GripperDriver):
    """Drive an xArm Gripper G2, which takes width in mm and caps force itself."""

    def read_position(self) -> int:
        """Return the measured opening width in millimetres."""
        code, position = self.arm.get_gripper_g2_position()
        _check_code("get_gripper_g2_position", code)
        return int(position)

    def read_force(self) -> int | None:
        """Return the measured force when this SDK build reports it."""
        reader = getattr(self.arm, "get_gripper_g2_force", None)
        if not callable(reader):
            return None
        code, force = reader()
        return int(force) if code == 0 else None

    def move_to(self, position: int) -> None:
        """Command an opening width in millimetres under the configured force."""
        _check_code(
            "set_gripper_g2_position",
            self.arm.set_gripper_g2_position(
                position,
                speed=self.config.gripper_speed,
                force=self.config.gripper_force,
                wait=False,
            ),
        )


class _ClassicGripper(_GripperDriver):
    """Drive the original xArm Gripper, which takes pulses and caps no force.

    This family accepts no force argument, so ``gripper_force`` is inert and
    over-grip protection rests entirely on the contact latch in
    :class:`XArm6Hardware` and on a small configured ``gripper_max_step``.
    """

    def enable(self) -> None:
        """Enable the gripper servo and select absolute position mode."""
        _check_code("set_gripper_enable", self.arm.set_gripper_enable(True))
        _check_code("set_gripper_mode", self.arm.set_gripper_mode(0))

    def read_position(self) -> int:
        """Return the measured jaw position as an encoder pulse count."""
        code, position = self.arm.get_gripper_position()
        _check_code("get_gripper_position", code)
        return int(position)

    def move_to(self, position: int) -> None:
        """Command an absolute jaw position as an encoder pulse count."""
        _check_code(
            "set_gripper_position",
            self.arm.set_gripper_position(
                position,
                speed=self.config.gripper_speed,
                wait=False,
            ),
        )


_GRIPPER_DRIVERS: dict[str, type[_GripperDriver]] = {
    "g2": _G2Gripper,
    "classic": _ClassicGripper,
}


class TargetSafety:
    """Validate mapped targets independently of the physical SDK."""

    def __init__(self, config: PhysicalXArmConfig) -> None:
        self.config = config
        self._previous: np.ndarray | None = None
        self._commanded: np.ndarray | None = None
        self._commanded_velocity: np.ndarray | None = None

    def reset(self, target_radians: np.ndarray | None = None) -> None:
        """Clear jump history and optionally validate a new baseline target.

        Args:
            target_radians: Optional six-joint baseline that bypasses the jump check.
        """
        self._previous = None
        self._commanded = None
        self._commanded_velocity = None
        if target_radians is not None:
            self.validate(target_radians, check_jump=False)

    def limit_acceleration(self, target_radians: np.ndarray, elapsed_seconds: float) -> np.ndarray:
        """Bound how fast commanded joint velocity may change, in place of mvacc.

        The jump limit caps velocity but permits reversing it between one sample
        and the next, which is a large acceleration that check cannot see. This
        clamps the change in velocity instead, so joint speed ramps rather than
        stepping.

        It deliberately does not plan an arrival. Unlike the controller's own
        planner it never decelerates toward the target, so closely spaced targets
        are tracked continuously instead of becoming a sawtooth of
        accelerate-then-decelerate profiles.

        This shapes the commanded stream only. Call it after :meth:`validate`, so
        that an impossible leader movement still faults on the raw target instead
        of being quietly clamped into a legal one.

        Args:
            target_radians: Six validated xArm joint targets in radians.
            elapsed_seconds: Measured time since the previous command. The budget
                is proportional to it, so a late cycle is granted the
                acceleration it genuinely had time for.

        Returns:
            The target itself on the first call, otherwise the target clamped to
            one acceleration budget away from the current commanded velocity.
        """
        target = np.asarray(target_radians, dtype=float)
        if self._commanded is None or self._commanded_velocity is None:
            self._commanded = target.copy()
            self._commanded_velocity = np.zeros_like(target)
            return target

        # Never credit less than a nominal period, so ordinary jitter does not
        # make the arm sluggish, and never more than a few of them, so a long
        # stall cannot bank a large budget and release it in one step.
        period = 1.0 / self.config.rate
        elapsed = float(min(max(elapsed_seconds, period), _MAX_ACCELERATION_PERIODS * period))
        budget = np.deg2rad(self.config.joint_acceleration_degrees) * elapsed

        desired_velocity = (target - self._commanded) / elapsed
        velocity = np.clip(
            desired_velocity,
            self._commanded_velocity - budget,
            self._commanded_velocity + budget,
        )
        self._commanded = self._commanded + velocity * elapsed
        self._commanded_velocity = velocity
        return self._commanded.copy()

    def validate(self, target_radians: np.ndarray, *, check_jump: bool = True) -> None:
        """Validate target shape, finiteness, static limits, and sample jump.

        Args:
            target_radians: Six xArm joint targets in radians.
            check_jump: Whether to compare the target with the previous sample.

        Raises:
            XArmHardwareError: If the target violates any configured safety constraint.
        """
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

    def approach(self, target_radians: np.ndarray, max_step_degrees: float) -> np.ndarray:
        """Return a target no further than one slew step from the previous target.

        Args:
            target_radians: Six desired xArm joint targets in radians.
            max_step_degrees: Largest per-joint change permitted this cycle.

        Returns:
            The desired target itself before any target has been validated,
            otherwise the previous target advanced toward it by at most one step.
        """
        target = np.asarray(target_radians, dtype=float)
        if self._previous is None:
            return target
        step = np.deg2rad(max_step_degrees)
        delta = np.clip(target - self._previous, -step, step)
        return self._previous + delta

    def divergence_degrees(self, target_radians: np.ndarray) -> float:
        """Return the largest per-joint gap in degrees from the previous target.

        Args:
            target_radians: Six desired xArm joint targets in radians.

        Returns:
            Zero before any target has been validated, otherwise the gap.
        """
        if self._previous is None:
            return 0.0
        target = np.asarray(target_radians, dtype=float)
        return float(np.max(np.abs(np.rad2deg(target - self._previous))))


class XArm6Hardware:
    """Physical follower that remains read-only until :meth:`arm_motion` succeeds."""

    def __init__(
        self,
        config: PhysicalXArmConfig,
        api_factory: Callable[..., _XArmAPI] | None = None,
    ) -> None:
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
        try:
            driver = _GRIPPER_DRIVERS[config.gripper_kind]
        except KeyError as error:
            raise XArmHardwareError(
                f"Unsupported physical_xarm.gripper_kind '{config.gripper_kind}'"
            ) from error
        self.gripper = driver(self.arm, config)
        self.safety = TargetSafety(config)
        self._lock = threading.RLock()
        self._armed = False
        self._watchdog_tripped = False
        self._catching_up = False
        self._last_command_time = 0.0
        self._last_gripper_position: int | None = None
        self._gripper_contact_latched = False
        self._watchdog_stop = threading.Event()
        self._watchdog_thread: threading.Thread | None = None

    @staticmethod
    def _check_code(operation: str, code: int) -> None:
        _check_code(operation, code)

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
            gripper = self.gripper.read_position()
            gripper_force = self.gripper.read_force()
            status_code, gripper_status = self.arm.get_gripper_status()
            if status_code != 0:
                gripper_status = None
            code, gripper_error = self.arm.get_gripper_err_code()
            self._check_code("get_gripper_err_code", code)

            return XArmStatus(
                connected=True,
                version=str(getattr(self.arm, "version", "unknown")),
                mode=int(getattr(self.arm, "mode", -1)),
                state=int(state),
                error_code=int(errors[0]),
                warning_code=int(errors[1]),
                joint_degrees=tuple(float(value) for value in np.rad2deg(joints)),
                gripper_position=int(gripper),
                gripper_force=gripper_force,
                gripper_status=(None if gripper_status is None else int(gripper_status) & 0x03),
                gripper_error_code=int(gripper_error),
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
        if status.gripper_error_code:
            raise XArmHardwareError(
                f"xArm Gripper reports error {status.gripper_error_code}; "
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
            self.gripper.enable()
        except Exception:
            self._best_effort_stop()
            raise

        with self._lock:
            self._armed = True
            self._watchdog_tripped = False
            self._catching_up = False
            self._last_command_time = time.monotonic()
            self._last_gripper_position = status.gripper_position
            self._gripper_contact_latched = status.gripper_status == 2
        self._start_watchdog()
        return status

    def command(self, action: np.ndarray, gripper_command_max: float) -> None:
        """Send one validated nonblocking joint and gripper command.

        Args:
            action: Six joint targets in radians followed by a gripper command.
            gripper_command_max: Mapping value that represents a fully closed gripper.

        Raises:
            XArmHardwareError: If motion is not armed or any safety/SDK check fails.
        """
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
            # set_state(0) enables motion, then the controller may report state 2
            # to mean READY while it is waiting for the next motion command.
            if state not in (0, 1, 2):
                raise XArmHardwareError(f"xArm entered non-motion state {state}")
            code, errors = self.arm.get_err_warn_code()
            self._check_code("get_err_warn_code", code)
            if errors[0] or errors[1]:
                raise XArmHardwareError(
                    f"Controller reports error={errors[0]}, warning={errors[1]}"
                )

            joints = values[:6]
            if self._catching_up:
                divergence = self.safety.divergence_degrees(joints)
                if divergence > self.config.catchup_max_divergence_degrees:
                    raise XArmHardwareError(
                        f"Leader diverged {divergence:.2f} deg during the link gap; limit is "
                        f"{self.config.catchup_max_divergence_degrees:.2f} deg"
                    )
                slewed = self.safety.approach(joints, self.config.catchup_step_degrees)
                # Catch-up ends once the slew no longer clips the desired target.
                self._catching_up = not np.allclose(slewed, joints)
                joints = slewed
            self.safety.validate(joints)
            if self.config.mode == 1:
                elapsed = time.monotonic() - self._last_command_time
                # Servo mode has no controller-side acceleration limit, so the
                # equivalent bound is applied here. It runs after validation so
                # that an impossible leader movement still faults on the raw
                # target rather than being clamped into an acceptable one.
                joints = self.safety.limit_acceleration(joints, elapsed)
            code, at_limit = self.arm.is_joint_limit(joints.tolist(), is_radian=True)
            self._check_code("is_joint_limit", code)
            if at_limit is not False:
                raise XArmHardwareError("The xArm controller rejected a joint target")

            ratio = float(np.clip(values[6] / gripper_command_max, 0.0, 1.0))
            desired_gripper = round(
                self.config.gripper_open_position
                + ratio * (self.config.gripper_closed_position - self.config.gripper_open_position)
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

            code, gripper_status = self.arm.get_gripper_status()
            if code != 0:
                gripper_status = None
            code, gripper_error = self.arm.get_gripper_err_code()
            self._check_code("get_gripper_err_code", code)
            if gripper_error:
                self._freeze_gripper()
                raise XArmHardwareError(
                    f"xArm Gripper reports error {gripper_error}; closing stopped"
                )

            opening = desired_gripper > self._last_gripper_position
            send_gripper_target = gripper_position
            if self._gripper_contact_latched:
                if opening:
                    self._gripper_contact_latched = False
                else:
                    send_gripper_target = self._last_gripper_position
            elif gripper_status is not None and int(gripper_status) & 0x03 == 2 and not opening:
                self._freeze_gripper()
                self._gripper_contact_latched = True
                send_gripper_target = self._last_gripper_position

            if self.config.mode == 1:
                # Servo mode streams the target straight to the joint
                # controller. It performs no trajectory planning, so the speed
                # and acceleration arguments are reserved and ignored, and the
                # validated per-sample jump is what bounds joint velocity.
                self._check_code(
                    "set_servo_angle_j",
                    self.arm.set_servo_angle_j(joints.tolist(), is_radian=True),
                )
            else:
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
            if send_gripper_target != self._last_gripper_position:
                self.gripper.move_to(send_gripper_target)
                self._last_gripper_position = send_gripper_target
            self._last_command_time = time.monotonic()

    @property
    def catching_up(self) -> bool:
        """Report whether the follower is still slewing toward the leader."""
        with self._lock:
            return self._catching_up

    def begin_catch_up(self) -> None:
        """Slew toward the leader instead of faulting on the next target jump.

        Called after a tolerated wireless gap, during which the follower held its
        last commanded target while the leader kept moving.
        """
        with self._lock:
            self._catching_up = True

    @property
    def gripper_contact_latched(self) -> bool:
        """Report whether detected contact is preventing further gripper closing."""
        with self._lock:
            return self._gripper_contact_latched

    def _freeze_gripper(self) -> None:
        """Best-effort replacement of a closing target with measured position."""
        try:
            position = self.gripper.read_position()
            self.gripper.move_to(position)
        except Exception:
            return
        self._last_gripper_position = position

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
                    and time.monotonic() - self._last_command_time > self.config.watchdog_timeout
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
        """Stop the watchdog and request xArm state 4 when motion is armed."""
        self._watchdog_stop.set()
        thread = self._watchdog_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        with self._lock:
            if self._armed:
                self._best_effort_stop()
            self._armed = False

    def close(self) -> None:
        """Request a safe stop and disconnect the SDK client."""
        self.safe_stop()
        if bool(getattr(self.arm, "connected", False)):
            self.arm.disconnect()

    def __enter__(self) -> Self:
        return self

    def __exit__(
        self,
        _exc_type: type[BaseException] | None,
        _exc_value: BaseException | None,
        _traceback: TracebackType | None,
    ) -> None:
        self.close()
