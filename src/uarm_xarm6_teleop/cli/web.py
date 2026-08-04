"""Launch the local web operator console."""

from __future__ import annotations

import argparse
import threading
import webbrowser
from functools import partial
from pathlib import Path

from ..config import load_config
from ..controller import TeleopController
from ..remote_leader import (
    BrowserPairedRemoteLeaderFactory,
    RemoteLeader,
    RemoteLeaderError,
    load_token_file,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the U-ARM operator console.")
    parser.add_argument("--config", help="path to a TOML configuration file")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port")
    parser.add_argument(
        "--leader-url",
        help="laptop leader service URL, for example ws://192.168.50.1:8765",
    )
    parser.add_argument(
        "--leader-token-file",
        help="path to the private token shared with the laptop leader service",
    )
    parser.add_argument(
        "--leader-timeout",
        type=float,
        default=0.2,
        help="remote leader connect/read timeout in seconds (default: 0.2)",
    )
    parser.add_argument(
        "--pair-browser-leader",
        action="store_true",
        help="derive the laptop address from the browser that connects the leader",
    )
    parser.add_argument(
        "--leader-port",
        type=int,
        default=8765,
        help="laptop leader-service port used with --pair-browser-leader",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="do not open the operator console in the default browser",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if args.pair_browser_leader and args.leader_url:
        parser.error("--pair-browser-leader cannot be combined with --leader-url")
    if args.pair_browser_leader and not args.leader_token_file:
        parser.error("--pair-browser-leader requires --leader-token-file")
    if not args.pair_browser_leader and bool(args.leader_url) != bool(args.leader_token_file):
        parser.error("--leader-url and --leader-token-file must be provided together")
    if not 1 <= args.leader_port <= 65535:
        parser.error("--leader-port must be between 1 and 65535")
    if args.leader_timeout <= 0:
        parser.error("--leader-timeout must be positive")
    return args


def run_web(
    *,
    config_path: str | Path | None = None,
    host: str = "127.0.0.1",
    port: int = 8000,
    leader_url: str | None = None,
    leader_token_file: str | Path | None = None,
    leader_timeout: float = 0.2,
    browser_pair_leader: bool = False,
    leader_port: int = 8765,
    open_browser: bool = True,
) -> None:
    try:
        import uvicorn

        from ..web.app import create_app
    except ImportError as error:
        raise SystemExit(
            "The web dependencies are missing. Install with `pip install -e '.[web]'`."
        ) from error
    try:
        config = load_config(config_path)
        browser_leader_factory = None
        if browser_pair_leader:
            if leader_url is not None:
                raise RemoteLeaderError("Browser pairing cannot be combined with a leader URL")
            if leader_token_file is None:
                raise RemoteLeaderError("A leader token file is required for browser pairing")
            browser_leader_factory = BrowserPairedRemoteLeaderFactory(
                token=load_token_file(leader_token_file),
                port=leader_port,
                timeout=leader_timeout,
            )
            controller = TeleopController(config, leader_factory=browser_leader_factory)
        elif leader_url:
            if leader_token_file is None:
                raise RemoteLeaderError("A leader token file is required for a remote leader")
            token = load_token_file(leader_token_file)
            leader_factory = partial(
                RemoteLeader,
                url=leader_url,
                token=token,
                timeout=leader_timeout,
            )
            controller = TeleopController(config, leader_factory=leader_factory)
        else:
            controller = TeleopController(config)
    except (OSError, RemoteLeaderError, ValueError) as error:
        raise SystemExit(f"Could not start uarm-web: {error}") from error
    application = create_app(controller, browser_leader_factory=browser_leader_factory)
    if open_browser:
        url_host = "127.0.0.1" if host in ("0.0.0.0", "::") else host
        threading.Timer(0.6, lambda: webbrowser.open(f"http://{url_host}:{port}")).start()
    uvicorn.run(application, host=host, port=port)


def main() -> None:
    args = parse_args()
    run_web(
        config_path=args.config,
        host=args.host,
        port=args.port,
        leader_url=args.leader_url,
        leader_token_file=args.leader_token_file,
        leader_timeout=args.leader_timeout,
        browser_pair_leader=args.pair_browser_leader,
        leader_port=args.leader_port,
        open_browser=not args.no_browser,
    )


if __name__ == "__main__":
    main()
