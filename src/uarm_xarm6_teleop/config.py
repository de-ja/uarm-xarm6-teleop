"""Typed configuration for the U-ARM teleoperation pipeline."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "configs" / "uarm_xarm6.toml"

# Reported by the controller as joint_speed_limit; commanding beyond it faults.
XARM6_MAX_JOINT_SPEED_DEGREES = 180.0

# Seconds a tactile sensor must rest before its baseline is captured.
DEFAULT_SENSOR_SETTLE_SECONDS = 5.0


@dataclass(frozen=True)
class GripperLimits:
    """Describe the command ranges a UFACTORY gripper family accepts."""

    max_position: int
    min_speed: int
    max_speed: int
    enforces_force: bool


# The two families use different units for the same arguments, so a value that
# is valid for one is silently wrong for the other. The G2 takes an opening
# width in millimetres and a speed in mm/s; the classic gripper takes an
# encoder pulse count and a speed in r/min, and accepts no force argument.
GRIPPER_LIMITS: dict[str, GripperLimits] = {
    "g2": GripperLimits(max_position=84, min_speed=15, max_speed=225, enforces_force=True),
    "classic": GripperLimits(max_position=850, min_speed=1, max_speed=5000, enforces_force=False),
}


@dataclass(frozen=True)
class SerialConfig:
    """Describe the Feetech serial bus and ordered servo IDs."""

    device: str
    baudrate: int
    ids: tuple[int, ...]


@dataclass(frozen=True)
class LeaderConfig:
    """Define leader calibration, directions, and display labels."""

    midpoint: int
    gripper_zero_position: int
    gripper_pressed_position: int
    directions: tuple[int, ...]
    labels: tuple[str, ...]


@dataclass(frozen=True)
class XArm6Config:
    """Define leader-to-follower joint and gripper mapping parameters."""

    reference_degrees: tuple[float, ...]
    joint_directions: tuple[int, ...]
    gripper_travel_degrees: float
    gripper_command_max: float
    gripper_mode: str
    gripper_press_degrees: float
    gripper_release_degrees: float


@dataclass(frozen=True)
class SimulationConfig:
    """Configure the ManiSkill scene and control frequency."""

    scene: str
    rate: float


@dataclass(frozen=True)
class WirelessConfig:
    """Define one shared tolerance policy for every wireless link.

    The follower host depends on two wireless links: the leader sample link to
    the laptop and the operator console's telemetry link to the browser. Both
    absorb a bounded amount of transient loss before the run is faulted.
    """

    leader_timeout: float
    leader_max_consecutive_timeouts: int
    browser_grace_seconds: float


@dataclass(frozen=True)
class PhysicalXArmConfig:
    """Define physical xArm connection, motion limits, and watchdog settings."""

    robot_ip: str
    rate: float
    mode: int
    joint_speed_degrees: float
    joint_acceleration_degrees: float
    startup_tolerance_degrees: float
    leader_start_tolerance_degrees: float
    max_target_jump_degrees: float
    catchup_step_degrees: float
    catchup_max_divergence_degrees: float
    watchdog_timeout: float
    joint_lower_degrees: tuple[float, ...]
    joint_upper_degrees: tuple[float, ...]
    gripper_kind: str
    gripper_open_position: int
    gripper_closed_position: int
    gripper_speed: int
    gripper_force: int
    gripper_max_step: int


@dataclass(frozen=True)
class SensorConfig:
    """Describe one auxiliary observation sensor.

    Sensors are observations, never control inputs, so a failure here degrades
    telemetry rather than stopping the robot.
    """

    kind: str
    name: str
    port: str
    settle: float


@dataclass(frozen=True)
class TeleopConfig:
    """Collect the validated configuration for every teleoperation component."""

    serial: SerialConfig
    leader: LeaderConfig
    xarm6: XArm6Config
    simulation: SimulationConfig
    wireless: WirelessConfig
    sensors: tuple[SensorConfig, ...]
    physical_xarm: PhysicalXArmConfig


def _section(data: dict, name: str) -> dict:
    value = data.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"[{name}] must be a TOML table")
    return value


def _sensor_tables(data: dict) -> list[dict]:
    value = data.get("sensors", [])
    if not isinstance(value, list) or any(not isinstance(entry, dict) for entry in value):
        raise ValueError("[[sensors]] must be an array of TOML tables")
    return value


def validate_config(config: TeleopConfig) -> TeleopConfig:
    """Validate cross-field dimensions, ranges, and physical safety invariants.

    Args:
        config: Fully constructed configuration to validate.

    Returns:
        The unchanged configuration when every invariant holds.

    Raises:
        ValueError: If a dimension, range, mode, or safety constraint is invalid.
    """
    joint_count = len(config.serial.ids)
    if joint_count != 7 or len(set(config.serial.ids)) != 7:
        raise ValueError("serial.ids must contain seven unique servo IDs")
    if len(config.leader.directions) != joint_count:
        raise ValueError("leader.directions must contain seven values")
    if any(value not in (-1, 1) for value in config.leader.directions):
        raise ValueError("leader.directions values must be either 1 or -1")
    if len(config.leader.labels) != joint_count:
        raise ValueError("leader.labels must contain seven values")
    if not 0 <= config.leader.midpoint < 4096:
        raise ValueError("leader.midpoint must be between 0 and 4095")
    if not 0 <= config.leader.gripper_zero_position < 4096:
        raise ValueError("leader.gripper_zero_position must be between 0 and 4095")
    if not 0 <= config.leader.gripper_pressed_position < 4096:
        raise ValueError("leader.gripper_pressed_position must be between 0 and 4095")
    if config.serial.baudrate <= 0:
        raise ValueError("serial.baudrate must be positive")
    if len(config.xarm6.reference_degrees) != 6:
        raise ValueError("xarm6.reference_degrees must contain six values")
    if len(config.xarm6.joint_directions) != 6:
        raise ValueError("xarm6.joint_directions must contain six values")
    if any(value not in (-1, 1) for value in config.xarm6.joint_directions):
        raise ValueError("xarm6.joint_directions values must be either 1 or -1")
    if config.xarm6.gripper_travel_degrees <= 0:
        raise ValueError("xarm6.gripper_travel_degrees must be positive")
    if config.xarm6.gripper_command_max <= 0:
        raise ValueError("xarm6.gripper_command_max must be positive")
    if config.xarm6.gripper_mode not in ("proportional", "toggle"):
        raise ValueError("xarm6.gripper_mode must be 'proportional' or 'toggle'")
    if config.xarm6.gripper_release_degrees < 0:
        raise ValueError("xarm6.gripper_release_degrees cannot be negative")
    if config.xarm6.gripper_press_degrees <= config.xarm6.gripper_release_degrees:
        raise ValueError("xarm6.gripper_press_degrees must exceed gripper_release_degrees")
    if config.simulation.rate <= 0:
        raise ValueError("simulation.rate must be positive")
    wireless = config.wireless
    if wireless.leader_timeout <= 0:
        raise ValueError("wireless.leader_timeout must be positive")
    if wireless.leader_max_consecutive_timeouts < 0:
        raise ValueError("wireless.leader_max_consecutive_timeouts cannot be negative")
    if wireless.browser_grace_seconds < 0:
        raise ValueError("wireless.browser_grace_seconds cannot be negative")
    physical = config.physical_xarm
    if physical.rate <= 0:
        raise ValueError("physical_xarm.rate must be positive")
    if physical.mode not in (1, 6):
        raise ValueError("physical_xarm.mode must be 1 (servo streaming) or 6 (online planning)")
    positive_values = {
        "joint_speed_degrees": physical.joint_speed_degrees,
        "joint_acceleration_degrees": physical.joint_acceleration_degrees,
        "startup_tolerance_degrees": physical.startup_tolerance_degrees,
        "leader_start_tolerance_degrees": physical.leader_start_tolerance_degrees,
        "max_target_jump_degrees": physical.max_target_jump_degrees,
        "catchup_step_degrees": physical.catchup_step_degrees,
        "catchup_max_divergence_degrees": physical.catchup_max_divergence_degrees,
        "watchdog_timeout": physical.watchdog_timeout,
        "gripper_speed": physical.gripper_speed,
        "gripper_force": physical.gripper_force,
        "gripper_max_step": physical.gripper_max_step,
    }
    for name, value in positive_values.items():
        if value <= 0:
            raise ValueError(f"physical_xarm.{name} must be positive")
    # A slewed catch-up target must always satisfy the per-sample jump limit,
    # otherwise recovery would trip the very check it is meant to respect.
    if physical.catchup_step_degrees > physical.max_target_jump_degrees:
        raise ValueError("physical_xarm.catchup_step_degrees cannot exceed max_target_jump_degrees")
    # The follower must fault before the robot-local watchdog trips, because a
    # tripped watchdog cannot be cleared without restarting teleoperation.
    blind_seconds = wireless.leader_timeout * (wireless.leader_max_consecutive_timeouts + 1)
    if blind_seconds >= physical.watchdog_timeout:
        raise ValueError(
            f"wireless.leader_timeout x (leader_max_consecutive_timeouts + 1) = "
            f"{blind_seconds:.3f}s must stay below physical_xarm.watchdog_timeout "
            f"({physical.watchdog_timeout:.3f}s)"
        )
    # Mode 1 streams positions with no trajectory planning, so the per-sample
    # jump and the loop rate together dictate the commanded joint velocity. The
    # xArm6 refuses joint speeds above 180 deg/s, so the configuration must not
    # be able to ask for one.
    if physical.mode == 1:
        implied_speed = physical.max_target_jump_degrees * physical.rate
        if implied_speed > XARM6_MAX_JOINT_SPEED_DEGREES:
            raise ValueError(
                f"physical_xarm.max_target_jump_degrees x rate = {implied_speed:.1f} deg/s "
                f"exceeds the {XARM6_MAX_JOINT_SPEED_DEGREES:.0f} deg/s joint limit in mode 1"
            )
    if len(physical.joint_lower_degrees) != 6 or len(physical.joint_upper_degrees) != 6:
        raise ValueError("physical_xarm joint limits must contain six values")
    if any(
        lower >= upper
        for lower, upper in zip(physical.joint_lower_degrees, physical.joint_upper_degrees)
    ):
        raise ValueError("physical_xarm lower joint limits must be below upper limits")
    seen_sensor_names: set[str] = set()
    for sensor in config.sensors:
        if not sensor.name:
            raise ValueError("Each [[sensors]] entry needs a name")
        if sensor.name in seen_sensor_names:
            raise ValueError(f"Duplicate sensor name '{sensor.name}'")
        seen_sensor_names.add(sensor.name)
        if not sensor.port:
            raise ValueError(f"Sensor '{sensor.name}' needs a port")
        # Both the leader and a USB CDC sensor enumerate as /dev/ttyACM*, and
        # the numbering depends on boot order, so a bare node can silently point
        # at the wrong device. by-id paths are stable across reboots.
        if sensor.port.startswith("/dev/ttyACM"):
            raise ValueError(
                f"Sensor '{sensor.name}' must not use a bare /dev/ttyACM node, whose "
                "number depends on boot order. Use a usb: selector such as "
                "usb:serial=XXXX, or a /dev/serial/by-id/ path. Run uarm-ports to list "
                "attached devices and their selectors."
            )
        # A baseline captured before the sensor has settled reads several times
        # noisier than the true floor: measured 9.7-16.5 uT immediately after
        # handling against 2.4-3.2 uT settled. Nothing downstream can detect
        # that, so a zero settle is refused rather than trusted.
        if sensor.settle <= 0:
            raise ValueError(
                f"Sensor '{sensor.name}' needs a positive settle. A baseline taken "
                "before the sensor has settled reads several times noisier than the "
                f"true floor; {DEFAULT_SENSOR_SETTLE_SECONDS:g} seconds is the default."
            )
    if physical.gripper_kind not in GRIPPER_LIMITS:
        raise ValueError(
            "physical_xarm.gripper_kind must be one of " + ", ".join(sorted(GRIPPER_LIMITS))
        )
    limits = GRIPPER_LIMITS[physical.gripper_kind]
    for name in ("gripper_open_position", "gripper_closed_position"):
        position = getattr(physical, name)
        if not 0 <= position <= limits.max_position:
            raise ValueError(
                f"physical_xarm.{name} must be between 0 and {limits.max_position} "
                f"for gripper_kind '{physical.gripper_kind}'"
            )
    if physical.gripper_open_position <= physical.gripper_closed_position:
        raise ValueError("physical_xarm.gripper_open_position must exceed gripper_closed_position")
    if not limits.min_speed <= physical.gripper_speed <= limits.max_speed:
        raise ValueError(
            f"physical_xarm.gripper_speed must be between {limits.min_speed} and "
            f"{limits.max_speed} for gripper_kind '{physical.gripper_kind}'"
        )
    # The classic gripper's SDK accepts no force argument, so the value is
    # carried but never sent. Only the G2 controller enforces a force ceiling.
    if limits.enforces_force and not 1 <= physical.gripper_force <= 100:
        raise ValueError("physical_xarm.gripper_force must be between 1 and 100")
    return config


def load_config(path: str | Path | None = None) -> TeleopConfig:
    """Load the base TOML configuration with an optional machine-local overlay.

    Args:
        path: Overlay file whose tables are merged onto the packaged defaults.

    Returns:
        A typed and validated teleoperation configuration.

    Raises:
        OSError: If the base or overlay file cannot be read.
        ValueError: If TOML content violates a configuration invariant.
    """
    with DEFAULT_CONFIG_PATH.open("rb") as stream:
        base_data = tomllib.load(stream)

    if path is None or Path(path).expanduser().resolve() == DEFAULT_CONFIG_PATH.resolve():
        data = base_data
    else:
        config_path = Path(path).expanduser()
        with config_path.open("rb") as stream:
            override_data = tomllib.load(stream)
        data = {}
        for name in ("serial", "leader", "xarm6", "simulation", "wireless", "physical_xarm"):
            data[name] = {**_section(base_data, name), **_section(override_data, name)}
        # An overlay that declares sensors replaces the list outright. Merging
        # entry-wise would make it impossible to remove a sensor locally, which
        # is the common case when hardware is not attached.
        data["sensors"] = (
            _sensor_tables(override_data)
            if "sensors" in override_data
            else _sensor_tables(base_data)
        )

    serial = _section(data, "serial")
    leader = _section(data, "leader")
    xarm6 = _section(data, "xarm6")
    simulation = _section(data, "simulation")
    wireless = _section(data, "wireless")
    physical_xarm = _section(data, "physical_xarm")
    sensors = tuple(
        SensorConfig(
            kind=str(entry.get("kind", "")),
            name=str(entry.get("name", "")),
            port=str(entry.get("port", "")),
            settle=float(entry.get("settle", DEFAULT_SENSOR_SETTLE_SECONDS)),
        )
        for entry in _sensor_tables(data)
    )

    config = TeleopConfig(
        serial=SerialConfig(
            device=str(serial["device"]),
            baudrate=int(serial["baudrate"]),
            ids=tuple(int(value) for value in serial["ids"]),
        ),
        leader=LeaderConfig(
            midpoint=int(leader["midpoint"]),
            gripper_zero_position=int(leader["gripper_zero_position"]),
            gripper_pressed_position=int(leader["gripper_pressed_position"]),
            directions=tuple(int(value) for value in leader["directions"]),
            labels=tuple(str(value) for value in leader["labels"]),
        ),
        xarm6=XArm6Config(
            reference_degrees=tuple(float(value) for value in xarm6["reference_degrees"]),
            joint_directions=tuple(int(value) for value in xarm6["joint_directions"]),
            gripper_travel_degrees=float(xarm6["gripper_travel_degrees"]),
            gripper_command_max=float(xarm6["gripper_command_max"]),
            gripper_mode=str(xarm6["gripper_mode"]),
            gripper_press_degrees=float(xarm6["gripper_press_degrees"]),
            gripper_release_degrees=float(xarm6["gripper_release_degrees"]),
        ),
        simulation=SimulationConfig(
            scene=str(simulation["scene"]),
            rate=float(simulation["rate"]),
        ),
        sensors=sensors,
        wireless=WirelessConfig(
            leader_timeout=float(wireless["leader_timeout"]),
            leader_max_consecutive_timeouts=int(wireless["leader_max_consecutive_timeouts"]),
            browser_grace_seconds=float(wireless["browser_grace_seconds"]),
        ),
        physical_xarm=PhysicalXArmConfig(
            robot_ip=str(physical_xarm["robot_ip"]),
            rate=float(physical_xarm["rate"]),
            mode=int(physical_xarm["mode"]),
            joint_speed_degrees=float(physical_xarm["joint_speed_degrees"]),
            joint_acceleration_degrees=float(physical_xarm["joint_acceleration_degrees"]),
            startup_tolerance_degrees=float(physical_xarm["startup_tolerance_degrees"]),
            leader_start_tolerance_degrees=float(physical_xarm["leader_start_tolerance_degrees"]),
            max_target_jump_degrees=float(physical_xarm["max_target_jump_degrees"]),
            catchup_step_degrees=float(physical_xarm["catchup_step_degrees"]),
            catchup_max_divergence_degrees=float(physical_xarm["catchup_max_divergence_degrees"]),
            watchdog_timeout=float(physical_xarm["watchdog_timeout"]),
            joint_lower_degrees=tuple(
                float(value) for value in physical_xarm["joint_lower_degrees"]
            ),
            joint_upper_degrees=tuple(
                float(value) for value in physical_xarm["joint_upper_degrees"]
            ),
            gripper_kind=str(physical_xarm.get("gripper_kind", "g2")),
            gripper_open_position=int(physical_xarm["gripper_open_position"]),
            gripper_closed_position=int(physical_xarm["gripper_closed_position"]),
            gripper_speed=int(physical_xarm["gripper_speed"]),
            gripper_force=int(physical_xarm["gripper_force"]),
            gripper_max_step=int(physical_xarm["gripper_max_step"]),
        ),
    )
    return validate_config(config)
