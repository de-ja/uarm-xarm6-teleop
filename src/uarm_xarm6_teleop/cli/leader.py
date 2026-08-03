"""Expose a locally attached U-ARM to one authenticated follower backend."""

from __future__ import annotations

import argparse
import asyncio

from ..feetech import FeetechError, FeetechLeader
from ..remote_leader import (
    MAX_MESSAGE_BYTES,
    RemoteLeaderError,
    RemoteLeaderService,
    load_token_file,
)
from .common import add_connection_arguments, config_from_args


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Serve this computer's U-ARM to uarm-web.")
    add_connection_arguments(parser)
    parser.add_argument(
        "--host",
        default="127.0.0.1",
        help="WebSocket bind address (use the laptop's private-network address)",
    )
    parser.add_argument("--port", type=int, default=8765, help="WebSocket port")
    parser.add_argument(
        "--token-file",
        required=True,
        help="path to a private shared-token file with mode 600",
    )
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("--port must be between 1 and 65535")
    return args


async def serve_leader(args: argparse.Namespace) -> None:
    try:
        from websockets.asyncio.server import serve
    except ImportError as error:  # pragma: no cover - optional host dependency
        raise RemoteLeaderError(
            "Remote leader support is missing. Install with `pip install -e '.[remote]'`."
        ) from error

    config = config_from_args(args)
    token = load_token_file(args.token_file)
    leader = FeetechLeader(config.serial, config.leader)
    leader.open()
    try:
        service = RemoteLeaderService(leader, config.serial, token=token)
        async with serve(
            service.handle,
            args.host,
            args.port,
            compression=None,
            max_size=MAX_MESSAGE_BYTES,
            max_queue=1,
            ping_interval=10,
            ping_timeout=5,
        ) as server:
            print(f"Serving read-only U-ARM samples on ws://{args.host}:{args.port}/ws/leader")
            if leader.torque_enabled_ids:
                print(f"WARNING: leader torque is enabled on IDs {leader.torque_enabled_ids}")
            print("Waiting for the follower backend. Press Ctrl-C to stop.")
            await server.serve_forever()
    finally:
        leader.close()


def main() -> None:
    args = parse_args()
    try:
        asyncio.run(serve_leader(args))
    except KeyboardInterrupt:
        print("\nRemote leader service stopped.")
    except (FeetechError, RemoteLeaderError, OSError, ValueError) as error:
        raise SystemExit(f"Remote leader service failed: {error}") from error


if __name__ == "__main__":
    main()
