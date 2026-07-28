"""Launch the local web operator console."""

from __future__ import annotations

import argparse
import threading
import webbrowser

from ..config import load_config
from ..controller import TeleopController


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Launch the U-ARM operator console.")
    parser.add_argument("--config", help="path to a TOML configuration file")
    parser.add_argument("--host", default="127.0.0.1", help="HTTP bind address")
    parser.add_argument("--port", type=int, default=8000, help="HTTP port")
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="do not open the operator console in the default browser",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
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
    controller = TeleopController(load_config(args.config))
    application = create_app(controller)
    if not args.no_browser:
        url_host = "127.0.0.1" if args.host in ("0.0.0.0", "::") else args.host
        threading.Timer(0.6, lambda: webbrowser.open(f"http://{url_host}:{args.port}")).start()
    uvicorn.run(application, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
