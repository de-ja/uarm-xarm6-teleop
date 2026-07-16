"""Interactive read-only terminal monitor."""

from __future__ import annotations

import argparse
import time

from ..feetech import FeetechError, FeetechLeader
from .common import add_connection_arguments, config_from_args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Monitor the Feetech U-ARM read-only.")
    add_connection_arguments(parser)
    parser.add_argument("--rate", type=float, default=10.0, help="refresh rate in Hz")
    parser.add_argument("--once", action="store_true", help="print one sample and exit")
    args = parser.parse_args()
    if args.rate <= 0:
        parser.error("--rate must be positive")
    return args


def print_sample(config, sample, clear: bool) -> None:
    if clear:
        print("\033[2J\033[H", end="")
    print("Feetech U-ARM (read-only; calibrated CAD pose = 0 deg)\n")
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
    args = parse_args()
    config = config_from_args(args)
    period = 1.0 / args.rate

    with FeetechLeader(config.serial, config.leader) as leader:
        if leader.torque_enabled_ids:
            print(f"WARNING: torque is enabled on IDs {leader.torque_enabled_ids}.")
        while True:
            started = time.monotonic()
            print_sample(config, leader.read(), clear=not args.once)
            if args.once:
                return
            time.sleep(max(0.0, period - (time.monotonic() - started)))


def main() -> None:
    try:
        run()
    except KeyboardInterrupt:
        print("\nStopped.")
    except (FeetechError, ValueError, OSError) as error:
        raise SystemExit(f"Error: {error}") from error


if __name__ == "__main__":
    main()
