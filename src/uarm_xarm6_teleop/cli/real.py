"""Dry-run-first teleoperation of a physical xArm6 and xArm Gripper."""

from __future__ import annotations

import argparse
import time
from dataclasses import replace

import numpy as np

from ..backends.xarm import TargetSafety, XArm6Hardware, XArmHardwareError, XArmStatus
from ..config import validate_config
from ..feetech import FeetechError, FeetechLeader
from ..mapping import XArm6Mapping
from .common import add_connection_arguments, config_from_args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Map the Feetech U-ARM to a physical xArm6 (dry-run by default)."
    )
    add_connection_arguments(parser)
    parser.add_argument("--robot-ip", help="xArm controller IPv4 address")
    parser.add_argument("--rate", type=float, help="override the control rate")
    parser.add_argument(
        "--inspect",
        action="store_true",
        help="read xArm status once without opening the leader or enabling motion",
    )
    parser.add_argument(
        "--enable-motion",
        action="store_true",
        help="allow physical motion after all checks and typed confirmation",
    )
    parser.add_argument("--once", action="store_true", help="process one leader sample")
    args = parser.parse_args()
    if args.rate is not None and args.rate <= 0:
        parser.error("--rate must be positive")
    if (args.inspect or args.enable_motion) and not args.robot_ip:
        parser.error("--robot-ip is required with --inspect or --enable-motion")
    if args.inspect and args.enable_motion:
        parser.error("--inspect and --enable-motion are mutually exclusive")
    return args


def make_mapping(config) -> XArm6Mapping:
    return XArm6Mapping(
        reference_degrees=config.xarm6.reference_degrees,
        joint_directions=config.xarm6.joint_directions,
        gripper_travel_degrees=config.xarm6.gripper_travel_degrees,
        gripper_command_max=config.xarm6.gripper_command_max,
    )


def format_target(action: np.ndarray) -> str:
    joints = "  ".join(
        f"J{index}={degrees:+7.2f}"
        for index, degrees in enumerate(np.rad2deg(action[:6]), start=1)
    )
    return f"{joints}  grip={action[6]:.3f}"


def print_status(status: XArmStatus) -> None:
    joints = ", ".join(f"{value:+.2f}" for value in status.joint_degrees)
    print(f"Connected: {status.connected}  SDK/firmware: {status.version}")
    print(
        f"Mode: {status.mode}  state: {status.state}  "
        f"error: {status.error_code}  warning: {status.warning_code}"
    )
    print(f"Joints (deg): [{joints}]")
    print(f"xArm Gripper position: {status.gripper_position}")


def require_safe_leader_start(config, sample) -> None:
    offsets = np.abs(sample.degrees[:6])
    joint = int(np.argmax(offsets))
    tolerance = config.physical_xarm.leader_start_tolerance_degrees
    if offsets[joint] > tolerance:
        raise XArmHardwareError(
            f"Leader J{joint + 1} is {sample.degrees[joint]:+.2f} deg from its calibrated "
            f"CAD pose; startup tolerance is {tolerance:.2f} deg"
        )


def run() -> None:
    args = parse_args()
    config = config_from_args(args)
    physical = replace(
        config.physical_xarm,
        robot_ip=args.robot_ip or config.physical_xarm.robot_ip,
        rate=args.rate or config.physical_xarm.rate,
    )
    config = validate_config(replace(config, physical_xarm=physical))

    if args.inspect:
        with XArm6Hardware(physical) as follower:
            print_status(follower.inspect())
        print("Inspection was read-only; motion was not enabled.")
        return

    mapping = make_mapping(config)
    safety = TargetSafety(physical)
    follower: XArm6Hardware | None = None

    try:
        with FeetechLeader(config.serial, config.leader) as leader:
            sample = leader.read()
            action = mapping.action(sample.radians)
            safety.reset(action[:6])
            print(f"Connected to leader IDs {config.serial.ids} on {config.serial.device}.")
            print(format_target(action))

            if leader.torque_enabled_ids:
                message = f"Leader torque is enabled on IDs {leader.torque_enabled_ids}"
                if args.enable_motion:
                    raise XArmHardwareError(message + "; physical teleoperation is blocked")
                print("WARNING: " + message)

            if args.enable_motion:
                require_safe_leader_start(config, sample)
                follower = XArm6Hardware(physical)
                status = follower.inspect()
                print_status(status)
                print("\nPhysical motion can occur after the next confirmation.")
                print("Keep the workspace clear, hold the emergency stop, and supervise the arm.")
                confirmation = input(f"Type the robot IP ({physical.robot_ip}) to arm motion: ")
                if confirmation.strip() != physical.robot_ip:
                    raise XArmHardwareError("Confirmation did not match; motion remains disabled")
                follower.arm_motion(action[:6])
                print("Mode 6 armed. Ctrl-C or a command timeout puts the arm in state 4.")
            else:
                print("DRY RUN: no xArm connection was opened and physical motion is disabled.")
                print("Add --robot-ip ADDRESS --enable-motion only after completing the checks.")

            period = 1.0 / physical.rate
            next_step = time.monotonic()
            next_report = next_step
            while True:
                sample = leader.read()
                action = mapping.action(sample.radians)
                safety.validate(action[:6])
                if follower is not None:
                    follower.command(action, config.xarm6.gripper_command_max)

                now = time.monotonic()
                if args.once or now >= next_report:
                    print("\r" + format_target(action) + "  ", end="", flush=True)
                    next_report = now + 0.25
                if args.once:
                    print()
                    return

                next_step += period
                delay = next_step - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_step = time.monotonic()
    finally:
        if follower is not None:
            follower.close()


def main() -> None:
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped; the physical follower was put in state 4 if it was armed.")
    except (EOFError, FeetechError, XArmHardwareError, ValueError, OSError) as error:
        raise SystemExit(f"Error: {error}") from error


if __name__ == "__main__":
    main()
