"""Pure joint-angle conversions shared by simulation and hardware backends."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


COUNTS_PER_REVOLUTION = 4096


def signed_delta(position: int, center: int = 2047) -> int:
    """Return the shortest signed single-turn displacement from center."""
    half_turn = COUNTS_PER_REVOLUTION // 2
    return (position - center + half_turn) % COUNTS_PER_REVOLUTION - half_turn


def positions_to_radians(
    positions: tuple[int, ...] | list[int],
    midpoint: int,
    directions: tuple[int, ...] | list[int],
) -> np.ndarray:
    if len(positions) != len(directions):
        raise ValueError("positions and directions must have the same length")
    deltas = np.asarray([signed_delta(value, midpoint) for value in positions], dtype=float)
    return deltas * (2.0 * np.pi / COUNTS_PER_REVOLUTION) * np.asarray(directions)


@dataclass(frozen=True)
class XArm6Mapping:
    """Map seven U-ARM values to six xArm6 joints and one gripper command."""

    reference_degrees: tuple[float, ...]
    joint_directions: tuple[int, ...]
    gripper_travel_degrees: float = 90.0
    gripper_command_max: float = 0.81

    def action(self, leader_radians: np.ndarray) -> np.ndarray:
        if leader_radians.shape != (7,):
            raise ValueError("leader_radians must contain exactly seven values")

        action = np.empty(7, dtype=np.float32)
        reference = np.deg2rad(np.asarray(self.reference_degrees, dtype=float))
        directions = np.asarray(self.joint_directions, dtype=float)
        action[:6] = reference + leader_radians[:6] * directions

        travel = np.deg2rad(self.gripper_travel_degrees)
        ratio = np.clip(leader_radians[6] / travel, 0.0, 1.0)
        action[6] = ratio * self.gripper_command_max
        return action
