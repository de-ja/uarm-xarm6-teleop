"""Shared command-line configuration helpers."""

from __future__ import annotations

import argparse
from dataclasses import replace

from ..config import TeleopConfig, load_config, validate_config


def add_connection_arguments(parser: argparse.ArgumentParser) -> None:
    """Add shared configuration and serial override flags to a parser."""
    parser.add_argument("--config", help="path to a TOML configuration file")
    parser.add_argument("--device", help="override the configured serial device")
    parser.add_argument("--baudrate", type=int, help="override the configured baud rate")


def config_from_args(args: argparse.Namespace) -> TeleopConfig:
    """Load configuration and apply command-line serial overrides.

    Args:
        args: Parsed namespace containing ``config``, ``device``, and ``baudrate``.

    Returns:
        Validated configuration with the requested serial values.
    """
    config = load_config(args.config)
    serial = replace(
        config.serial,
        device=args.device or config.serial.device,
        baudrate=args.baudrate or config.serial.baudrate,
    )
    return validate_config(replace(config, serial=serial))
