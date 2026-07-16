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

    gripper_travel_degrees: float = 90.0
    gripper_command_max: float = 0.81

    def action(self, leader_radians: np.ndarray) -> np.ndarray:
        if leader_radians.shape != (7,):
            raise ValueError("leader_radians must contain exactly seven values")

        action = np.empty(7, dtype=np.float32)
        # Original U-ARM mapping: J4/J5 swap, and follower J5 is reversed.
        action[:6] = leader_radians[[0, 1, 2, 4, 3, 5]]
        action[4] *= -1.0

        travel = np.deg2rad(self.gripper_travel_degrees)
        ratio = np.clip(leader_radians[6] / travel, 0.0, 1.0)
        action[6] = ratio * self.gripper_command_max
        return action
