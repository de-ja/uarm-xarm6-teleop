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


@dataclass
class XArm6Mapping:
    """Map seven U-ARM values to six xArm6 joints and one gripper command."""

    reference_degrees: tuple[float, ...]
    joint_directions: tuple[int, ...]
    gripper_travel_degrees: float = 90.0
    gripper_command_max: float = 0.81
    gripper_mode: str = "proportional"
    gripper_press_degrees: float = 10.0
    gripper_release_degrees: float = 4.0

    def __post_init__(self) -> None:
        self._gripper_closed = False
        self._gripper_command = 0.0
        self._trigger_armed = False

    def reset_gripper(
        self,
        trigger_radians: float,
        *,
        closed: bool = False,
        command: float | None = None,
    ) -> None:
        """Preserve follower state and require a release before the next toggle."""
        self._gripper_closed = bool(closed)
        if command is None:
            self._gripper_command = self.gripper_command_max if closed else 0.0
        else:
            self._gripper_command = float(
                np.clip(command, 0.0, self.gripper_command_max)
            )
        self._trigger_armed = (
            np.rad2deg(trigger_radians) <= self.gripper_release_degrees
        )

    def _gripper_action(self, trigger_radians: float) -> float:
        trigger_degrees = float(np.rad2deg(trigger_radians))
        if self.gripper_mode == "proportional":
            ratio = np.clip(
                trigger_degrees / self.gripper_travel_degrees, 0.0, 1.0
            )
            return float(ratio * self.gripper_command_max)
        if self.gripper_mode != "toggle":
            raise ValueError(f"Unsupported gripper mode: {self.gripper_mode}")

        if self._trigger_armed:
            if trigger_degrees >= self.gripper_press_degrees:
                self._gripper_closed = not self._gripper_closed
                self._gripper_command = (
                    self.gripper_command_max if self._gripper_closed else 0.0
                )
                self._trigger_armed = False
        elif trigger_degrees <= self.gripper_release_degrees:
            self._trigger_armed = True
        return self._gripper_command

    def action(self, leader_radians: np.ndarray) -> np.ndarray:
        if leader_radians.shape != (7,):
            raise ValueError("leader_radians must contain exactly seven values")

        action = np.empty(7, dtype=np.float32)
        reference = np.deg2rad(np.asarray(self.reference_degrees, dtype=float))
        directions = np.asarray(self.joint_directions, dtype=float)
        action[:6] = reference + leader_radians[:6] * directions

        action[6] = self._gripper_action(float(leader_radians[6]))
        return action
