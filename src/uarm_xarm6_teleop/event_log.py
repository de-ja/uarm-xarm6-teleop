"""Nonblocking structured logging for controller session events."""

from __future__ import annotations

import json
import os
import queue
import threading
from pathlib import Path
from typing import Protocol

from .protocol import ControllerEvent

EVENT_LOG_SCHEMA_VERSION = 1


class EventSink(Protocol):
    """Accept controller events without performing work in the caller thread."""

    def emit(self, session_id: str, event: ControllerEvent) -> None:
        """Queue one controller event for external persistence."""

    def close(self) -> None:
        """Flush queued records and release sink resources."""


class AsyncJsonlEventSink:
    """Persist controller events as JSON Lines from a dedicated writer thread.

    Args:
        path: Output file, created with parent directories when necessary.
        queue_size: Maximum records waiting for the writer.
        close_timeout: Maximum seconds to wait for writer shutdown.

    Raises:
        OSError: If the log file cannot be opened.
        ValueError: If queue or timeout settings are not positive.
    """

    def __init__(
        self,
        path: str | Path,
        *,
        queue_size: int = 1024,
        close_timeout: float = 2.0,
    ) -> None:
        if queue_size <= 0:
            raise ValueError("Event-log queue size must be positive")
        if close_timeout <= 0:
            raise ValueError("Event-log close timeout must be positive")
        self.path = Path(path).expanduser()
        self.path.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        self.path.touch(mode=0o600, exist_ok=True)
        os.chmod(self.path, 0o600)
        self._stream = self.path.open("a", encoding="utf-8")
        self._queue: queue.Queue[dict[str, object] | None] = queue.Queue(queue_size)
        self._close_timeout = close_timeout
        self._lock = threading.Lock()
        self._closed = False
        self._dropped = 0
        self._error: str | None = None
        self._thread = threading.Thread(
            target=self._write,
            name="uarm-event-log",
            daemon=True,
        )
        self._thread.start()

    @property
    def dropped_records(self) -> int:
        """Return the number of records discarded because the queue was full."""
        with self._lock:
            return self._dropped

    @property
    def error(self) -> str | None:
        """Return a writer error captured outside the controller thread."""
        with self._lock:
            return self._error

    def emit(self, session_id: str, event: ControllerEvent) -> None:
        """Queue one serializable controller event without blocking."""
        record: dict[str, object] = {
            "schema_version": EVENT_LOG_SCHEMA_VERSION,
            "session_id": session_id,
            "timestamp": event.timestamp,
            "level": event.level,
            "message": event.message,
        }
        with self._lock:
            if self._closed:
                return
        try:
            self._queue.put_nowait(record)
        except queue.Full:
            with self._lock:
                self._dropped += 1

    def _write(self) -> None:
        try:
            while True:
                record = self._queue.get()
                if record is None:
                    return
                self._stream.write(json.dumps(record, separators=(",", ":")) + "\n")
                self._stream.flush()
        except OSError as error:
            with self._lock:
                self._error = str(error)
        finally:
            self._stream.close()

    def close(self) -> None:
        """Flush queued records and stop the writer with bounded waiting."""
        with self._lock:
            if self._closed:
                return
            self._closed = True
        try:
            # Waiting here is outside the control loop and lets queued records drain.
            self._queue.put(None, timeout=self._close_timeout)
        except queue.Full:
            with self._lock:
                self._dropped += 1
        self._thread.join(timeout=self._close_timeout)
        if self._thread.is_alive():
            raise RuntimeError("Event-log writer did not stop within the timeout")
