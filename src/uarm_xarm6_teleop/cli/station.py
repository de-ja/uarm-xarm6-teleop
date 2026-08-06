"""Interactive launcher for a networked xArm follower station."""

from __future__ import annotations

import argparse
import json
import subprocess
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path

from ..config import load_config
from ..remote_leader import DEFAULT_REMOTE_TOKEN_PATH, RemoteLeaderError, load_token_file
from .web import run_web

DEFAULT_TOKEN_PATH = DEFAULT_REMOTE_TOKEN_PATH
DEFAULT_CONFIG_PATH = Path("~/.config/uarm/desktop.toml").expanduser()

InputFunction = Callable[[str], str]
OutputFunction = Callable[[str], None]


@dataclass(frozen=True)
class NetworkAddress:
    """Pair an active network interface name with its IPv4 address."""

    interface: str
    address: str


def _parse_network_addresses(raw: str) -> tuple[NetworkAddress, ...]:
    try:
        links = json.loads(raw)
    except json.JSONDecodeError:
        return ()
    addresses = []
    for link in links if isinstance(links, list) else ():
        if not isinstance(link, dict) or link.get("ifname") == "lo":
            continue
        interface = str(link.get("ifname", "unknown"))
        for info in link.get("addr_info", ()):
            if not isinstance(info, dict) or info.get("family") != "inet":
                continue
            address = str(info.get("local", "")).strip()
            if address and not address.startswith("169.254."):
                addresses.append(NetworkAddress(interface, address))
    return tuple(addresses)


def discover_network_addresses() -> tuple[NetworkAddress, ...]:
    """Discover active non-loopback IPv4 addresses using ``iproute2``."""
    try:
        result = subprocess.run(
            ["ip", "-j", "-4", "address", "show", "up"],
            check=True,
            capture_output=True,
            text=True,
        )
    except (OSError, subprocess.CalledProcessError):
        return ()
    return _parse_network_addresses(result.stdout)


def console_urls(port: int, addresses: tuple[NetworkAddress, ...]) -> tuple[str, ...]:
    """Build laptop-accessible console URLs for the desktop addresses."""
    return tuple(f"http://{item.address}:{port}" for item in addresses)


def diagnose_station(
    *,
    config_path: str | Path | None,
    token_path: str | Path,
    port: int,
    output: OutputFunction = print,
) -> bool:
    """Check token, configuration, and local network visibility.

    Args:
        config_path: Optional desktop configuration overlay.
        token_path: Shared leader-token path.
        port: Desktop HTTP console port.
        output: Function used to display diagnostic messages.

    Returns:
        Whether all required local files were valid.
    """
    output("")
    output("FOLLOWER STATION DIAGNOSTICS")
    try:
        load_token_file(token_path)
        output(f"[OK] Shared token: {Path(token_path).expanduser()}")
        load_config(config_path)
        output("[OK] Teleop configuration")
    except (OSError, RemoteLeaderError, ValueError) as error:
        output(f"[FAIL] {error}")
        return False

    urls = console_urls(port, discover_network_addresses())
    if urls:
        for url in urls:
            output(f"[OK] Open from the leader laptop: {url}")
    else:
        output("[WARN] No active private-network address was detected")
    output("The U-ARM connection is checked when the laptop clicks Connect leader.")
    return True


def interactive_station(
    *,
    config_path: str | Path | None = None,
    token_path: str | Path = DEFAULT_TOKEN_PATH,
    host: str = "0.0.0.0",
    port: int = 8000,
    leader_port: int = 8765,
    event_log_path: str | Path | None = None,
    input_fn: InputFunction = input,
    output: OutputFunction = print,
) -> None:
    """Display the follower TUI and launch browser-paired web operation.

    Args:
        config_path: Optional desktop configuration overlay.
        token_path: Shared leader-token path.
        host: HTTP bind address.
        port: HTTP console port.
        leader_port: Laptop WebSocket service port.
        event_log_path: Optional JSON Lines destination for session events.
        input_fn: Interactive input function, replaceable in tests.
        output: Interactive output function, replaceable in tests.
    """
    output("")
    output("U-ARM FOLLOWER STATION")
    output("  Leader pairing: automatic from the laptop browser")
    urls = console_urls(port, discover_network_addresses())
    if urls:
        output("  Open one of these addresses on the laptop:")
        for url in urls:
            output(f"    {url}")
    else:
        output(f"  Web port: {port} (no active LAN address detected)")
    output("")
    choice = input_fn("[Enter] Start   [D] Diagnostics   [Q] Quit: ").strip().lower()
    if choice in ("q", "quit"):
        return
    if choice in ("d", "diagnostics"):
        diagnose_station(
            config_path=config_path,
            token_path=token_path,
            port=port,
            output=output,
        )
        return
    if choice not in ("", "s", "start"):
        raise ValueError("Choose Start, Diagnostics, or Quit")

    load_token_file(token_path)
    if config_path is not None:
        load_config(config_path)
    output("Starting the follower console. Open its LAN URL on the laptop.")
    run_web(
        config_path=config_path,
        host=host,
        port=port,
        leader_token_file=token_path,
        leader_timeout=0.2,
        browser_pair_leader=True,
        leader_port=leader_port,
        event_log_path=event_log_path,
        open_browser=False,
    )


def parse_args() -> argparse.Namespace:
    """Parse follower-station options and locate the default desktop overlay."""
    parser = argparse.ArgumentParser(description="Interactive xArm follower-station launcher.")
    parser.add_argument("--config", help="desktop TOML configuration file")
    parser.add_argument("--token-file", default=str(DEFAULT_TOKEN_PATH), help="shared token file")
    parser.add_argument("--host", default="0.0.0.0", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port")
    parser.add_argument("--leader-port", type=int, default=8765, help="laptop leader-service port")
    parser.add_argument("--event-log", help="optional JSON Lines controller event log")
    parser.add_argument("--diagnose", action="store_true", help="check local station setup")
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if not 1 <= args.leader_port <= 65535:
        parser.error("--leader-port must be between 1 and 65535")
    if args.config is None and DEFAULT_CONFIG_PATH.exists():
        args.config = str(DEFAULT_CONFIG_PATH)
    return args


def main() -> None:
    """Run the ``uarm-station`` command."""
    args = parse_args()
    try:
        if args.diagnose:
            success = diagnose_station(
                config_path=args.config,
                token_path=args.token_file,
                port=args.port,
            )
            raise SystemExit(0 if success else 1)
        interactive_station(
            config_path=args.config,
            token_path=args.token_file,
            host=args.host,
            port=args.port,
            leader_port=args.leader_port,
            event_log_path=args.event_log,
        )
    except KeyboardInterrupt:
        print("\nFollower station stopped.")
    except (OSError, RemoteLeaderError, ValueError) as error:
        raise SystemExit(f"Follower station failed: {error}") from error


if __name__ == "__main__":
    main()
