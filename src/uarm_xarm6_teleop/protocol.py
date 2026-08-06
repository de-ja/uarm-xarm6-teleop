"""Versioned telemetry types shared by the controller and web API."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from enum import Enum
from typing import Literal

from .backends.xarm import XArmStatus

PROTOCOL_VERSION = 3
LeaderTransport = Literal["local", "remote_explicit", "remote_browser_pairing"]
TeleopMode = Literal["dry_run", "simulation", "physical"]


class TeleopState(str, Enum):
    """Enumerate serialized controller lifecycle states exposed by the API."""

    IDLE = "idle"
    LEADER_READY = "leader_ready"
    READY = "ready"
    STARTING = "starting"
    RUNNING = "running"
    STOPPING = "stopping"
    STOPPED = "stopped"
    FAULT = "fault"


@dataclass(frozen=True)
class RuntimeCapabilities:
    """Describe deployment features available from this backend process."""

    leader_transport: LeaderTransport
    simulation_available: bool
    physical_available: bool
    camera_streaming: bool
    structured_logging: bool
    video_transport: Literal["mjpeg"]
    max_robots: Literal[1]


def default_runtime_capabilities() -> RuntimeCapabilities:
    """Return conservative capabilities for a controller without web discovery."""
    return RuntimeCapabilities(
        leader_transport="local",
        simulation_available=False,
        physical_available=False,
        camera_streaming=False,
        structured_logging=False,
        video_transport="mjpeg",
        max_robots=1,
    )


@dataclass(frozen=True)
class ControllerEvent:
    """Represent one timestamped operator-visible controller event."""

    timestamp: float
    level: Literal["info", "warning", "error"]
    message: str


@dataclass(frozen=True)
class TeleopSnapshot:
    """Provide an immutable, serializable view of controller state and telemetry."""

    protocol_version: int
    session_id: str
    capabilities: RuntimeCapabilities
    timestamp: float
    state: TeleopState
    mode: TeleopMode | None
    leader_connected: bool
    robot_connected: bool
    robot_ip: str
    torque_enabled_ids: tuple[int, ...]
    leader_degrees: tuple[float, ...] | None
    target_degrees: tuple[float, ...] | None
    gripper_command: float | None
    robot_status: XArmStatus | None
    loop_rate_hz: float
    command_latency_ms: float | None
    last_sample_age_ms: float | None
    fault: str | None
    events: tuple[ControllerEvent, ...]

    def to_dict(self) -> dict[str, object]:
        """Serialize the snapshot and nested dataclasses for API clients."""
        return asdict(self)
