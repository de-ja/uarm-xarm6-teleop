"""Typed configuration for the U-ARM teleoperation pipeline."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path


DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "configs" / "uarm_xarm6.toml"


@dataclass(frozen=True)
class SerialConfig:
    device: str
    baudrate: int
    ids: tuple[int, ...]


@dataclass(frozen=True)
class LeaderConfig:
    midpoint: int
    directions: tuple[int, ...]
    labels: tuple[str, ...]


@dataclass(frozen=True)
class XArm6Config:
    reference_degrees: tuple[float, ...]
    joint_directions: tuple[int, ...]
    gripper_travel_degrees: float
    gripper_command_max: float


@dataclass(frozen=True)
class SimulationConfig:
    scene: str
    rate: float


@dataclass(frozen=True)
class PhysicalXArmConfig:
    robot_ip: str
    rate: float
    mode: int
    joint_speed_degrees: float
    joint_acceleration_degrees: float
    startup_tolerance_degrees: float
    leader_start_tolerance_degrees: float
    max_target_jump_degrees: float
    watchdog_timeout: float
    joint_lower_degrees: tuple[float, ...]
    joint_upper_degrees: tuple[float, ...]
    gripper_open_position: int
    gripper_closed_position: int
    gripper_speed: int
    gripper_max_step: int


@dataclass(frozen=True)
class TeleopConfig:
    serial: SerialConfig
    leader: LeaderConfig
    xarm6: XArm6Config
    simulation: SimulationConfig
    physical_xarm: PhysicalXArmConfig


def _section(data: dict, name: str) -> dict:
    value = data.get(name, {})
    if not isinstance(value, dict):
        raise ValueError(f"[{name}] must be a TOML table")
    return value


def validate_config(config: TeleopConfig) -> TeleopConfig:
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
    if config.simulation.rate <= 0:
        raise ValueError("simulation.rate must be positive")
    physical = config.physical_xarm
    if physical.rate <= 0:
        raise ValueError("physical_xarm.rate must be positive")
    if physical.mode != 6:
        raise ValueError("physical_xarm.mode must be 6 for joint-servo teleoperation")
    positive_values = {
        "joint_speed_degrees": physical.joint_speed_degrees,
        "joint_acceleration_degrees": physical.joint_acceleration_degrees,
        "startup_tolerance_degrees": physical.startup_tolerance_degrees,
        "leader_start_tolerance_degrees": physical.leader_start_tolerance_degrees,
        "max_target_jump_degrees": physical.max_target_jump_degrees,
        "watchdog_timeout": physical.watchdog_timeout,
        "gripper_speed": physical.gripper_speed,
        "gripper_max_step": physical.gripper_max_step,
    }
    for name, value in positive_values.items():
        if value <= 0:
            raise ValueError(f"physical_xarm.{name} must be positive")
    if len(physical.joint_lower_degrees) != 6 or len(physical.joint_upper_degrees) != 6:
        raise ValueError("physical_xarm joint limits must contain six values")
    if any(
        lower >= upper
        for lower, upper in zip(
            physical.joint_lower_degrees, physical.joint_upper_degrees
        )
    ):
        raise ValueError("physical_xarm lower joint limits must be below upper limits")
    if not 0 <= physical.gripper_open_position <= 850:
        raise ValueError("physical_xarm.gripper_open_position must be between 0 and 850")
    if not 0 <= physical.gripper_closed_position <= 850:
        raise ValueError("physical_xarm.gripper_closed_position must be between 0 and 850")
    return config


def load_config(path: str | Path | None = None) -> TeleopConfig:
    """Load the project TOML configuration, optionally overlaid by another TOML."""
    with DEFAULT_CONFIG_PATH.open("rb") as stream:
        base_data = tomllib.load(stream)

    if path is None or Path(path).expanduser().resolve() == DEFAULT_CONFIG_PATH.resolve():
        data = base_data
    else:
        config_path = Path(path).expanduser()
        with config_path.open("rb") as stream:
            override_data = tomllib.load(stream)
        data = {}
        for name in ("serial", "leader", "xarm6", "simulation", "physical_xarm"):
            data[name] = {**_section(base_data, name), **_section(override_data, name)}

    serial = _section(data, "serial")
    leader = _section(data, "leader")
    xarm6 = _section(data, "xarm6")
    simulation = _section(data, "simulation")
    physical_xarm = _section(data, "physical_xarm")

    config = TeleopConfig(
        serial=SerialConfig(
            device=str(serial["device"]),
            baudrate=int(serial["baudrate"]),
            ids=tuple(int(value) for value in serial["ids"]),
        ),
        leader=LeaderConfig(
            midpoint=int(leader["midpoint"]),
            directions=tuple(int(value) for value in leader["directions"]),
            labels=tuple(str(value) for value in leader["labels"]),
        ),
        xarm6=XArm6Config(
            reference_degrees=tuple(float(value) for value in xarm6["reference_degrees"]),
            joint_directions=tuple(int(value) for value in xarm6["joint_directions"]),
            gripper_travel_degrees=float(xarm6["gripper_travel_degrees"]),
            gripper_command_max=float(xarm6["gripper_command_max"]),
        ),
        simulation=SimulationConfig(
            scene=str(simulation["scene"]),
            rate=float(simulation["rate"]),
        ),
        physical_xarm=PhysicalXArmConfig(
            robot_ip=str(physical_xarm["robot_ip"]),
            rate=float(physical_xarm["rate"]),
            mode=int(physical_xarm["mode"]),
            joint_speed_degrees=float(physical_xarm["joint_speed_degrees"]),
            joint_acceleration_degrees=float(
                physical_xarm["joint_acceleration_degrees"]
            ),
            startup_tolerance_degrees=float(
                physical_xarm["startup_tolerance_degrees"]
            ),
            leader_start_tolerance_degrees=float(
                physical_xarm["leader_start_tolerance_degrees"]
            ),
            max_target_jump_degrees=float(
                physical_xarm["max_target_jump_degrees"]
            ),
            watchdog_timeout=float(physical_xarm["watchdog_timeout"]),
            joint_lower_degrees=tuple(
                float(value) for value in physical_xarm["joint_lower_degrees"]
            ),
            joint_upper_degrees=tuple(
                float(value) for value in physical_xarm["joint_upper_degrees"]
            ),
            gripper_open_position=int(physical_xarm["gripper_open_position"]),
            gripper_closed_position=int(physical_xarm["gripper_closed_position"]),
            gripper_speed=int(physical_xarm["gripper_speed"]),
            gripper_max_step=int(physical_xarm["gripper_max_step"]),
        ),
    )
    return validate_config(config)
