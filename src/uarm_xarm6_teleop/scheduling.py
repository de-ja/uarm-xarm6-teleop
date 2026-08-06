"""Reusable fixed-rate scheduling primitives for sampled control loops."""

from __future__ import annotations

import threading
import time
from collections.abc import Callable


class PeriodicScheduler:
    """Maintain fixed-rate deadlines and reset cleanly after an overrun.

    Args:
        rate_hz: Desired loop frequency in hertz.
        monotonic: Clock used for deadlines.
        sleep: Sleep function used when no stop event is supplied.

    Raises:
        ValueError: If the requested rate is not positive.
    """

    def __init__(
        self,
        rate_hz: float,
        *,
        monotonic: Callable[[], float] = time.monotonic,
        sleep: Callable[[float], None] = time.sleep,
    ) -> None:
        if rate_hz <= 0:
            raise ValueError("Periodic scheduler rate must be positive")
        self.period = 1.0 / rate_hz
        self._monotonic = monotonic
        self._sleep = sleep
        self._next_deadline = monotonic()

    def wait(self, stop_event: threading.Event | None = None) -> bool:
        """Wait until the next deadline without accumulating missed periods.

        Args:
            stop_event: Optional event that can interrupt the wait.

        Returns:
            Whether the stop event was set before or during the wait.
        """
        self._next_deadline += self.period
        delay = self._next_deadline - self._monotonic()
        if delay > 0:
            if stop_event is not None:
                return stop_event.wait(delay)
            self._sleep(delay)
            return False
        self._next_deadline = self._monotonic()
        return stop_event.is_set() if stop_event is not None else False


class RateMeter:
    """Estimate sampled-loop frequency over bounded reporting windows.

    Args:
        window_seconds: Minimum measurement window.
        monotonic: Clock used for elapsed time.
    """

    def __init__(
        self,
        *,
        window_seconds: float = 0.5,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        if window_seconds <= 0:
            raise ValueError("Rate meter window must be positive")
        self.window_seconds = window_seconds
        self._monotonic = monotonic
        self._started = monotonic()
        self._samples = 0

    def record(self, now: float | None = None) -> float | None:
        """Record one sample and return a new estimate when the window closes."""
        current = self._monotonic() if now is None else now
        self._samples += 1
        elapsed = current - self._started
        if elapsed < self.window_seconds:
            return None
        rate = self._samples / elapsed
        self._started = current
        self._samples = 0
        return rate
