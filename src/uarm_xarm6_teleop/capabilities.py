"""Detect optional runtime features without importing heavy hardware packages."""

from __future__ import annotations

from importlib.util import find_spec

from .protocol import LeaderTransport, RuntimeCapabilities


def detect_runtime_capabilities(
    *,
    leader_transport: LeaderTransport = "local",
    structured_logging: bool = False,
) -> RuntimeCapabilities:
    """Describe features installed in the current backend environment.

    Args:
        leader_transport: Leader ownership and pairing mode selected at startup.
        structured_logging: Whether this process has an active event log sink.

    Returns:
        Immutable capabilities included in every telemetry snapshot.
    """
    return RuntimeCapabilities(
        leader_transport=leader_transport,
        simulation_available=find_spec("mani_skill") is not None,
        physical_available=find_spec("xarm") is not None,
        camera_streaming=find_spec("cv2") is not None,
        structured_logging=structured_logging,
        video_transport="mjpeg",
        max_robots=1,
    )
