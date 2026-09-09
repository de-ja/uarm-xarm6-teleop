"""List attached USB serial devices and the selector that identifies each."""

from __future__ import annotations

import argparse

from ..config import load_config
from ..serial_ports import (
    SerialPortError,
    is_selector,
    list_usb_serial_ports,
    resolve_serial_port,
)


def parse_args() -> argparse.Namespace:
    """Parse port discovery options."""
    parser = argparse.ArgumentParser(
        description="List USB serial devices and show how configured selectors resolve."
    )
    parser.add_argument("--config", help="path to a TOML configuration file")
    return parser.parse_args()


def print_attached() -> None:
    """Print every attached USB serial device with a suggested selector."""
    candidates = list_usb_serial_ports()
    if not candidates:
        print("No USB serial devices are attached.")
        print("Legacy /dev/ttyS* ports are not listed; none of them is leader or sensor hardware.")
        return
    print(f"{len(candidates)} USB serial device(s) attached:\n")
    for candidate in candidates:
        print(f"  {candidate.describe()}")
        print(f"    selector: {candidate.selector()}")
    print(
        "\nA serial= selector names one physical unit and survives reboots and "
        "re-plugging.\nPrefer it over a /dev/ttyACM* node, whose number depends on boot order."
    )


def print_configured(path: str | None) -> int:
    """Resolve every configured device and report what each points at.

    Args:
        path: Optional configuration overlay to load.

    Returns:
        Process exit status, non-zero when a configured device cannot resolve.
    """
    config = load_config(path)
    entries = [("serial.device", config.serial.device)]
    entries.extend((f"sensors.{sensor.name}", sensor.port) for sensor in config.sensors)

    print("\nConfigured devices:\n")
    status = 0
    for label, spec in entries:
        kind = "selector" if is_selector(spec) else "path"
        try:
            resolved = resolve_serial_port(spec)
        except SerialPortError as error:
            status = 1
            first = str(error).splitlines()[0]
            print(f"  {label}: {spec}  ({kind})\n    UNRESOLVED: {first}")
            continue
        suffix = "" if resolved == spec else f" -> {resolved}"
        print(f"  {label}: {spec}  ({kind}){suffix}")
    return status


def main() -> int:
    """Entry point for the ``uarm-ports`` console script."""
    args = parse_args()
    print_attached()
    return print_configured(args.config)


if __name__ == "__main__":
    raise SystemExit(main())
