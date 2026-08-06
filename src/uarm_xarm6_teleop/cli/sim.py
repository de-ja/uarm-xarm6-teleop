"""Run the Feetech leader with a visible ManiSkill xArm6 follower."""

from __future__ import annotations

import argparse
import time

from ..backends.maniskill import ManiSkillXArm6
from ..config import TeleopConfig
from ..feetech import FeetechError, FeetechLeader, LeaderSample
from ..mapping import XArm6Mapping
from ..scheduling import PeriodicScheduler
from .common import add_connection_arguments, config_from_args


def parse_args() -> argparse.Namespace:
    """Parse visible simulation and leader-check options."""
    parser = argparse.ArgumentParser(
        description="Control a visible ManiSkill xArm6 from the Feetech U-ARM."
    )
    add_connection_arguments(parser)
    parser.add_argument("--scene", help="override the configured ManiSkill scene")
    parser.add_argument("--rate", type=float, help="override the configured control rate")
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="read one leader sample without initializing ManiSkill",
    )
    args = parser.parse_args()
    if args.rate is not None and args.rate <= 0:
        parser.error("--rate must be positive")
    return args


def format_angles(config: TeleopConfig, sample: LeaderSample) -> str:
    """Format calibrated leader angles for terminal monitoring."""
    return "  ".join(
        f"{label}={degrees:+.1f}" for label, degrees in zip(config.leader.labels, sample.degrees)
    )


def run() -> None:
    """Drive a visible ManiSkill follower from the local U-ARM."""
    args = parse_args()
    config = config_from_args(args)
    rate = args.rate or config.simulation.rate
    scene = args.scene or config.simulation.scene
    mapping = XArm6Mapping(
        reference_degrees=config.xarm6.reference_degrees,
        joint_directions=config.xarm6.joint_directions,
        gripper_travel_degrees=config.xarm6.gripper_travel_degrees,
        gripper_command_max=config.xarm6.gripper_command_max,
        gripper_mode=config.xarm6.gripper_mode,
        gripper_press_degrees=config.xarm6.gripper_press_degrees,
        gripper_release_degrees=config.xarm6.gripper_release_degrees,
    )

    with FeetechLeader(config.serial, config.leader) as leader:
        if leader.torque_enabled_ids:
            print(f"WARNING: torque is enabled on IDs {leader.torque_enabled_ids}.")
        sample = leader.read()
        print(f"Connected to IDs {config.serial.ids} on {config.serial.device}.")
        print(format_angles(config, sample))
        if args.check_only:
            return

        print(f"Opening xarm6_robotiq in {scene}. Press Ctrl-C to stop.")
        scheduler = PeriodicScheduler(rate)
        next_report = time.monotonic()

        with ManiSkillXArm6(scene) as follower:
            while True:
                sample = leader.read()
                follower.step(mapping.action(sample.radians))

                now = time.monotonic()
                if now >= next_report:
                    print("\r" + format_angles(config, sample) + "  ", end="", flush=True)
                    next_report = now + 0.25

                scheduler.wait()


def main() -> None:
    """Run the ``uarm-sim`` command."""
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (FeetechError, RuntimeError, ValueError, OSError) as error:
        raise SystemExit(f"Error: {error}") from error


if __name__ == "__main__":
    main()
