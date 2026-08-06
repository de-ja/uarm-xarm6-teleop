"""Interactive read-only terminal monitor."""

from __future__ import annotations

import argparse
from ..config import TeleopConfig
from ..feetech import FeetechError, FeetechLeader, LeaderSample
from ..scheduling import PeriodicScheduler
from .common import add_connection_arguments, config_from_args


def parse_args() -> argparse.Namespace:
    """Parse read-only monitor options."""
    parser = argparse.ArgumentParser(description="Monitor the Feetech U-ARM read-only.")
    add_connection_arguments(parser)
    parser.add_argument("--rate", type=float, default=10.0, help="refresh rate in Hz")
    parser.add_argument("--once", action="store_true", help="print one sample and exit")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    return args


def print_sample(config: TeleopConfig, sample: LeaderSample, clear: bool) -> None:
    """Render one leader sample as a terminal table."""
    if clear:
        print("\033[2J\033[H", end="")
    print("Feetech U-ARM (read-only; configured initial pose = 0 deg)\n")
    print(" ID  Joint         Raw     Angle")
    print(" --  ------------  ----  --------")
    for servo_id, label, position, degrees in zip(
        config.serial.ids,
        config.leader.labels,
        sample.positions,
        sample.degrees,
    ):
        print(f" {servo_id:2d}  {label:<12}  {position:4d}  {degrees:+8.2f}")


def run() -> None:
    """Open the leader and print samples at the requested rate."""
    args = parse_args()
    config = config_from_args(args)
    with FeetechLeader(config.serial, config.leader) as leader:
        scheduler = PeriodicScheduler(args.rate)
        if leader.torque_enabled_ids:
            print(f"WARNING: torque is enabled on IDs {leader.torque_enabled_ids}.")
        while True:
            print_sample(config, leader.read(), clear=not args.once)
            if args.once:
                return
            scheduler.wait()


def main() -> None:
    """Run the ``uarm-monitor`` command."""
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (FeetechError, ValueError, OSError) as error:
        raise SystemExit(f"Error: {error}") from error


if __name__ == "__main__":
    main()
