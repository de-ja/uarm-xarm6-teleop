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
class TeleopConfig:
    serial: SerialConfig
    leader: LeaderConfig
    xarm6: XArm6Config
    simulation: SimulationConfig


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
        for name in ("serial", "leader", "xarm6", "simulation"):
            data[name] = {**_section(base_data, name), **_section(override_data, name)}

    serial = _section(data, "serial")
    leader = _section(data, "leader")
    xarm6 = _section(data, "xarm6")
    simulation = _section(data, "simulation")

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
    )
    return validate_config(config)
