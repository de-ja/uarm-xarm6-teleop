"""Authenticated request/response transport for a remote U-ARM leader."""

from __future__ import annotations

import asyncio
import json
import secrets
import stat
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any, Protocol, Self
from urllib.parse import urlsplit, urlunsplit

from .config import LeaderConfig, SerialConfig
from .feetech import LeaderSample
from .mapping import positions_to_radians

REMOTE_LEADER_PROTOCOL = 1
REMOTE_LEADER_PATH = "/ws/leader"
MAX_MESSAGE_BYTES = 4096
MIN_TOKEN_LENGTH = 32


class RemoteLeaderError(RuntimeError):
    """Raised when the remote leader transport is unavailable or invalid."""


class _SyncConnection(Protocol):
    def send(self, message: str) -> None: ...

    def recv(self, timeout: float | None = None) -> str | bytes: ...

    def close(self) -> None: ...


class _AsyncConnection(Protocol):
    request: Any

    async def recv(self) -> str | bytes: ...

    async def send(self, message: str) -> None: ...

    async def close(self, code: int = 1000, reason: str = "") -> None: ...


ConnectFactory = Callable[..., _SyncConnection]


def load_token_file(path: str | Path) -> str:
    token_path = Path(path).expanduser()
    try:
        mode = stat.S_IMODE(token_path.stat().st_mode)
        token = token_path.read_text(encoding="utf-8").strip()
    except OSError as error:
        raise RemoteLeaderError(
            f"Could not read leader token file {token_path}: {error}"
        ) from error
    if mode & 0o077:
        raise RemoteLeaderError(
            f"Leader token file {token_path} must not be accessible by group or other users"
        )
    if len(token) < MIN_TOKEN_LENGTH:
        raise RemoteLeaderError(
            f"Leader token in {token_path} must contain at least {MIN_TOKEN_LENGTH} characters"
        )
    return token


def normalize_leader_url(url: str) -> str:
    parsed = urlsplit(url.strip())
    if parsed.scheme not in ("ws", "wss") or not parsed.hostname:
        raise RemoteLeaderError("Remote leader URL must use ws:// or wss:// and include a host")
    if parsed.username is not None or parsed.password is not None or parsed.fragment:
        raise RemoteLeaderError("Remote leader URL cannot contain credentials or a fragment")
    path = REMOTE_LEADER_PATH if parsed.path in ("", "/") else parsed.path
    return urlunsplit((parsed.scheme, parsed.netloc, path, parsed.query, ""))


def _decode_message(raw: str | bytes) -> dict[str, object]:
    size = len(raw.encode("utf-8")) if isinstance(raw, str) else len(raw)
    if size > MAX_MESSAGE_BYTES:
        raise RemoteLeaderError("Remote leader message exceeded the size limit")
    try:
        decoded = json.loads(raw)
    except (json.JSONDecodeError, UnicodeDecodeError) as error:
        raise RemoteLeaderError("Remote leader sent invalid JSON") from error
    if not isinstance(decoded, dict):
        raise RemoteLeaderError("Remote leader message must be a JSON object")
    return decoded


def _positions_to_sample(
    positions: tuple[int, ...],
    leader: LeaderConfig,
    *,
    timestamp: float,
) -> LeaderSample:
    radians = positions_to_radians(positions, leader.midpoint, leader.directions)
    radians[6] = positions_to_radians(
        [positions[6]],
        leader.gripper_zero_position,
        [leader.directions[6]],
    )[0]
    return LeaderSample(timestamp=timestamp, positions=positions, radians=radians)


class RemoteLeader:
    """Synchronous leader interface backed by a laptop WebSocket service."""

    def __init__(
        self,
        serial: SerialConfig,
        leader: LeaderConfig,
        *,
        url: str,
        token: str,
        timeout: float = 0.5,
        monotonic: Callable[[], float] = time.monotonic,
        connect_factory: ConnectFactory | None = None,
    ):
        if timeout <= 0:
            raise ValueError("Remote leader timeout must be positive")
        if len(token) < MIN_TOKEN_LENGTH:
            raise RemoteLeaderError(
                f"Remote leader token must contain at least {MIN_TOKEN_LENGTH} characters"
            )
        self.serial_config = serial
        self.leader_config = leader
        self.url = normalize_leader_url(url)
        self.token = token
        self.timeout = timeout
        self._monotonic = monotonic
        self._connect_factory = connect_factory
        self._connection: _SyncConnection | None = None
        self._sequence = 0
        self.torque_enabled_ids: tuple[int, ...] = ()
        self.description = f"remote leader at {self.url}"

    def _connect(self) -> _SyncConnection:
        if self._connect_factory is None:
            try:
                from websockets.sync.client import connect
            except ImportError as error:  # pragma: no cover - optional host dependency
                raise RemoteLeaderError(
                    "Remote leader support is missing. Install with `pip install -e '.[remote]'`."
                ) from error
            self._connect_factory = connect
        try:
            return self._connect_factory(
                self.url,
                open_timeout=self.timeout,
                close_timeout=self.timeout,
                ping_interval=10,
                ping_timeout=self.timeout,
                compression=None,
                max_size=MAX_MESSAGE_BYTES,
                max_queue=1,
                proxy=None,
            )
        except Exception as error:
            raise RemoteLeaderError(f"Could not connect to {self.url}: {error}") from error

    def _receive(self) -> dict[str, object]:
        if self._connection is None:
            raise RemoteLeaderError("Remote leader is not connected")
        try:
            return _decode_message(self._connection.recv(timeout=self.timeout))
        except RemoteLeaderError:
            raise
        except Exception as error:
            raise RemoteLeaderError(f"Remote leader receive failed: {error}") from error

    def open(self) -> None:
        if self._connection is not None:
            raise RemoteLeaderError("Remote leader is already connected")
        connection = self._connect()
        self._connection = connection
        try:
            connection.send(
                json.dumps(
                    {
                        "type": "hello",
                        "protocol": REMOTE_LEADER_PROTOCOL,
                        "token": self.token,
                        "ids": list(self.serial_config.ids),
                    },
                    separators=(",", ":"),
                )
            )
            response = self._receive()
            if response.get("type") == "error":
                raise RemoteLeaderError(
                    str(response.get("message", "Remote leader rejected access"))
                )
            if (
                response.get("type") != "hello"
                or response.get("protocol") != REMOTE_LEADER_PROTOCOL
            ):
                raise RemoteLeaderError("Remote leader returned an incompatible handshake")
            ids = tuple(response.get("ids", ()))
            if ids != self.serial_config.ids:
                raise RemoteLeaderError(
                    f"Remote servo IDs {ids} do not match configured IDs {self.serial_config.ids}"
                )
            torque_ids = response.get("torque_enabled_ids", ())
            if not isinstance(torque_ids, list) or not all(
                isinstance(value, int) for value in torque_ids
            ):
                raise RemoteLeaderError("Remote leader returned invalid torque state")
            self.torque_enabled_ids = tuple(torque_ids)
        except Exception:
            self.close()
            raise

    def read(self) -> LeaderSample:
        if self._connection is None:
            raise RemoteLeaderError("Remote leader is not connected")
        self._sequence += 1
        sequence = self._sequence
        requested_at = self._monotonic()
        try:
            self._connection.send(
                json.dumps({"type": "sample", "sequence": sequence}, separators=(",", ":"))
            )
        except Exception as error:
            raise RemoteLeaderError(f"Remote leader send failed: {error}") from error
        response = self._receive()
        if response.get("type") == "error":
            raise RemoteLeaderError(str(response.get("message", "Remote leader read failed")))
        if response.get("type") != "sample" or response.get("sequence") != sequence:
            raise RemoteLeaderError("Remote leader returned an unexpected sample sequence")
        raw_positions = response.get("positions")
        if not isinstance(raw_positions, list) or len(raw_positions) != len(self.serial_config.ids):
            raise RemoteLeaderError("Remote leader returned an incomplete position sample")
        if not all(
            isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 4096
            for value in raw_positions
        ):
            raise RemoteLeaderError("Remote leader positions must be integers from 0 through 4095")
        positions = tuple(raw_positions)
        return _positions_to_sample(positions, self.leader_config, timestamp=requested_at)

    def close(self) -> None:
        connection, self._connection = self._connection, None
        self.torque_enabled_ids = ()
        if connection is not None:
            try:
                connection.close()
            except Exception:  # noqa: BLE001,S110 - best-effort transport cleanup
                pass

    def __enter__(self) -> Self:
        self.open()
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback) -> None:
        self.close()


class RemoteLeaderService:
    """Serve one authenticated follower while retaining local serial ownership."""

    def __init__(self, leader: Any, serial: SerialConfig, *, token: str):
        if len(token) < MIN_TOKEN_LENGTH:
            raise RemoteLeaderError(
                f"Remote leader token must contain at least {MIN_TOKEN_LENGTH} characters"
            )
        self.leader = leader
        self.serial = serial
        self.token = token
        self._active: _AsyncConnection | None = None

    async def _reject(self, connection: _AsyncConnection, message: str, code: int = 1008) -> None:
        await connection.send(
            json.dumps({"type": "error", "message": message}, separators=(",", ":"))
        )
        await connection.close(code=code, reason=message[:120])

    async def handle(self, connection: _AsyncConnection) -> None:
        path = getattr(getattr(connection, "request", None), "path", REMOTE_LEADER_PATH)
        if path.split("?", 1)[0] != REMOTE_LEADER_PATH:
            await self._reject(connection, "Unknown remote leader endpoint")
            return
        if self._active is not None:
            await self._reject(connection, "Another follower already owns the leader", code=1013)
            return
        self._active = connection
        try:
            try:
                raw_hello = await asyncio.wait_for(connection.recv(), timeout=5.0)
                hello = _decode_message(raw_hello)
            except Exception:  # noqa: BLE001 - protocol boundary
                await self._reject(connection, "Leader authentication timed out or was invalid")
                return
            supplied_token = hello.get("token")
            if (
                hello.get("type") != "hello"
                or hello.get("protocol") != REMOTE_LEADER_PROTOCOL
                or not isinstance(supplied_token, str)
                or not secrets.compare_digest(supplied_token, self.token)
            ):
                await self._reject(connection, "Leader authentication failed")
                return
            requested_ids = tuple(hello.get("ids", ()))
            if requested_ids != self.serial.ids:
                await self._reject(connection, "Follower servo ID order does not match the leader")
                return
            await connection.send(
                json.dumps(
                    {
                        "type": "hello",
                        "protocol": REMOTE_LEADER_PROTOCOL,
                        "ids": list(self.serial.ids),
                        "torque_enabled_ids": list(self.leader.torque_enabled_ids),
                    },
                    separators=(",", ":"),
                )
            )

            last_sequence = 0
            while True:
                request = _decode_message(await connection.recv())
                sequence = request.get("sequence")
                if (
                    request.get("type") != "sample"
                    or not isinstance(sequence, int)
                    or isinstance(sequence, bool)
                    or sequence <= last_sequence
                ):
                    await self._reject(connection, "Invalid or stale sample request")
                    return
                last_sequence = sequence
                try:
                    positions = self.leader.read_positions()
                except Exception as error:  # noqa: BLE001 - serial read boundary
                    await self._reject(connection, f"Leader read failed: {error}", code=1011)
                    return
                if len(positions) != len(self.serial.ids) or not all(
                    isinstance(value, int) and not isinstance(value, bool) and 0 <= value < 4096
                    for value in positions
                ):
                    await self._reject(
                        connection, "Leader returned an invalid position sample", code=1011
                    )
                    return
                await connection.send(
                    json.dumps(
                        {
                            "type": "sample",
                            "sequence": sequence,
                            "positions": list(positions),
                            "captured_at": time.time(),
                        },
                        separators=(",", ":"),
                    )
                )
        except Exception:  # noqa: BLE001 - WebSocket connection boundary
            # Normal WebSocket disconnects and malformed traffic both release ownership.
            return
        finally:
            if self._active is connection:
                self._active = None
