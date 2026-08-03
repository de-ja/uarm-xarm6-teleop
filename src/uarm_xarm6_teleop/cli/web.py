"""Launch the local web operator console."""

from __future__ import annotations

import argparse
import threading
import webbrowser
from functools import partial

from ..config import load_config
from ..controller import TeleopController
from ..remote_leader import RemoteLeader, RemoteLeaderError, load_token_file


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
        "--no-browser",
        action="store_true",
        help="do not open the operator console in the default browser",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    if bool(args.leader_url) != bool(args.leader_token_file):
        parser.error("--leader-url and --leader-token-file must be provided together")
    if args.leader_timeout <= 0:
        parser.error("--leader-timeout must be positive")
    return args


def main() -> None:
    try:
        import uvicorn

        from ..web.app import create_app
    except ImportError as error:
        raise SystemExit(
            "The web dependencies are missing. Install with `pip install -e '.[web]'`."
        ) from error

    args = parse_args()
    try:
        config = load_config(args.config)
        if args.leader_url:
            token = load_token_file(args.leader_token_file)
            leader_factory = partial(
                RemoteLeader,
                url=args.leader_url,
                token=token,
                timeout=args.leader_timeout,
            )
            controller = TeleopController(config, leader_factory=leader_factory)
        else:
            controller = TeleopController(config)
    except (OSError, RemoteLeaderError, ValueError) as error:
        raise SystemExit(f"Could not start uarm-web: {error}") from error
    application = create_app(controller)
    if not args.no_browser:
        url_host = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
        threading.Timer(0.6, lambda: webbrowser.open(f"http://{url_host}:{args.port}")).start()
    uvicorn.run(application, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
