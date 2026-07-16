"""Typed configuration for the U-ARM teleoperation pipeline."""

from __future__ import annotations

import tomllib
from dataclasses import dataclass
from pathlib import Path


DEFAULT_IDS = (1, 2, 3, 4, 5, 6, 7)
DEFAULT_LABELS = ("base", "shoulder", "J3", "J4", "J5", "wrist", "trigger")


@dataclass(frozen=True)
class SerialConfig:
    device: str = "/dev/ttyACM0"
    baudrate: int = 1_000_000
    ids: tuple[int, ...] = DEFAULT_IDS


@dataclass(frozen=True)
class LeaderConfig:
    midpoint: int = 2047
    directions: tuple[int, ...] = (1, 1, 1, 1, 1, 1, 1)
    labels: tuple[str, ...] = DEFAULT_LABELS


@dataclass(frozen=True)
class XArm6Config:
    gripper_travel_degrees: float = 90.0
    gripper_command_max: float = 0.81


@dataclass(frozen=True)
class SimulationConfig:
    scene: str = "Empty-v1"
    rate: float = 30.0


@dataclass(frozen=True)
class TeleopConfig:
    serial: SerialConfig = SerialConfig()
    leader: LeaderConfig = LeaderConfig()
    xarm6: XArm6Config = XArm6Config()
    simulation: SimulationConfig = SimulationConfig()


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
    if config.xarm6.gripper_travel_degrees <= 0:
        raise ValueError("xarm6.gripper_travel_degrees must be positive")
    if config.xarm6.gripper_command_max <= 0:
        raise ValueError("xarm6.gripper_command_max must be positive")
    if config.simulation.rate <= 0:
        raise ValueError("simulation.rate must be positive")
    return config


def load_config(path: str | Path | None = None) -> TeleopConfig:
    """Load TOML configuration, or return validated built-in defaults."""
    if path is None:
        return validate_config(TeleopConfig())

    config_path = Path(path).expanduser()
    with config_path.open("rb") as stream:
        data = tomllib.load(stream)

    serial = _section(data, "serial")
    leader = _section(data, "leader")
    xarm6 = _section(data, "xarm6")
    simulation = _section(data, "simulation")

    defaults = TeleopConfig()
    config = TeleopConfig(
        serial=SerialConfig(
            device=str(serial.get("device", defaults.serial.device)),
            baudrate=int(serial.get("baudrate", defaults.serial.baudrate)),
            ids=tuple(int(value) for value in serial.get("ids", defaults.serial.ids)),
        ),
        leader=LeaderConfig(
            midpoint=int(leader.get("midpoint", defaults.leader.midpoint)),
            directions=tuple(
                int(value) for value in leader.get("directions", defaults.leader.directions)
            ),
            labels=tuple(str(value) for value in leader.get("labels", defaults.leader.labels)),
        ),
        xarm6=XArm6Config(
            gripper_travel_degrees=float(
                xarm6.get("gripper_travel_degrees", defaults.xarm6.gripper_travel_degrees)
            ),
            gripper_command_max=float(
                xarm6.get("gripper_command_max", defaults.xarm6.gripper_command_max)
            ),
        ),
        simulation=SimulationConfig(
            scene=str(simulation.get("scene", defaults.simulation.scene)),
            rate=float(simulation.get("rate", defaults.simulation.rate)),
        ),
    )
    return validate_config(config)
