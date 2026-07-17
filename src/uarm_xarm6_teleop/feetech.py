"""Read-only Feetech STS bus access for the seven-joint U-ARM."""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .config import LeaderConfig, SerialConfig
from .mapping import positions_to_radians

try:
    from scservo_sdk import (
        COMM_SUCCESS,
        GroupSyncRead,
        PacketHandler,
        PortHandler,
    )
except ImportError as error:  # pragma: no cover - depends on the host environment
    raise ImportError(
        "The Feetech SDK is missing. Install this project with `pip install -e .`."
    ) from error


# Feetech STS control-table addresses. The PyPI SDK provides the packet
# protocol but, unlike Waveshare's vendor copy, does not expose model-specific
# STS register constants.
STS_TORQUE_ENABLE = 40
STS_PRESENT_POSITION_L = 56


class FeetechError(RuntimeError):
    """Raised when the leader bus cannot provide a complete valid sample."""


@dataclass(frozen=True)
class LeaderSample:
    timestamp: float
    positions: tuple[int, ...]
    radians: np.ndarray

    @property
    def degrees(self) -> np.ndarray:
        return np.rad2deg(self.radians)


class FeetechLeader:
    """Synchronous, read-only connection to all configured leader servos."""

    def __init__(self, serial: SerialConfig, leader: LeaderConfig):
        self.serial_config = serial
        self.leader_config = leader
        self.port = PortHandler(serial.device)
        self.packet = PacketHandler(0)
        self.sync_read = GroupSyncRead(
            self.port, self.packet, STS_PRESENT_POSITION_L, 2
        )
        self.torque_enabled_ids: tuple[int, ...] = ()

    def open(self) -> None:
        try:
            opened = self.port.openPort()
        except Exception as error:
            raise FeetechError(
                f"Could not open {self.serial_config.device}: {error}"
            ) from error
        if not opened:
            raise FeetechError(f"Could not open {self.serial_config.device}")
        if not self.port.setBaudRate(self.serial_config.baudrate):
            self.close()
            raise FeetechError(f"Could not set baud rate {self.serial_config.baudrate}")

        missing = []
        torque_enabled = []
        for servo_id in self.serial_config.ids:
            _model, result, error = self.packet.ping(self.port, servo_id)
            if result != COMM_SUCCESS or error != 0:
                missing.append(servo_id)
                continue
            torque, result, error = self.packet.read1ByteTxRx(
                self.port, servo_id, STS_TORQUE_ENABLE
            )
            if result == COMM_SUCCESS and error == 0 and torque:
                torque_enabled.append(servo_id)

        if missing:
            self.close()
            raise FeetechError(f"Servo IDs did not respond cleanly: {missing}")
        self.torque_enabled_ids = tuple(torque_enabled)

    def read_positions(self) -> tuple[int, ...]:
        self.sync_read.clearParam()
        for servo_id in self.serial_config.ids:
            if not self.sync_read.addParam(servo_id):
                raise FeetechError(f"Could not queue servo ID {servo_id}")

        result = self.sync_read.txRxPacket()
        if result != COMM_SUCCESS:
            raise FeetechError(
                "Synchronous read failed: " + self.packet.getTxRxResult(result)
            )

        positions = []
        for servo_id in self.serial_config.ids:
            available = self.sync_read.isAvailable(
                servo_id, STS_PRESENT_POSITION_L, 2
            )
            if not available:
                raise FeetechError(f"No valid position received from servo ID {servo_id}")
            positions.append(
                self.sync_read.getData(servo_id, STS_PRESENT_POSITION_L, 2)
            )
        return tuple(positions)

    def read(self) -> LeaderSample:
        positions = self.read_positions()
        radians = positions_to_radians(
            positions,
            self.leader_config.midpoint,
            self.leader_config.directions,
        )
        return LeaderSample(time.monotonic(), positions, radians)

    def close(self) -> None:
        if getattr(self.port, "ser", None) is not None:
            self.port.closePort()

    def __enter__(self) -> "FeetechLeader":
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()
