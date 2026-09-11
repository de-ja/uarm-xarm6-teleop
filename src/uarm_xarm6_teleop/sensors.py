"""Auxiliary observation sources that never participate in robot control.

A sensor here is an *observation*, not an input. That distinction sets the
failure policy for the whole module: losing the leader must fault the run,
because control has no input without it, but losing a tactile sensor must not.
It costs an observation stream while teleoperation continues. :class:`SensorHub`
therefore swallows and reports every source-level failure rather than raising
into the control loop, and each source is expected to acquire its samples off
the control thread so that a slow or wedged transport cannot stall it.
"""

from __future__ import annotations

import logging
import threading
import time
from collections import deque
from dataclasses import dataclass

import numpy as np
from typing import Protocol, runtime_checkable

from .config import SensorConfig
from .serial_ports import resolve_serial_port


_LOGGER = logging.getLogger(__name__)


# Slowest cadence the reader will spin at when a source does not block. Real
# hardware paces itself on serial well below this.
MIN_SAMPLE_PERIOD_SECONDS = 1.0 / 250.0


class SensorError(RuntimeError):
    """Raised when a sensor cannot be constructed or started."""


@dataclass(frozen=True)
class SensorInfo:
    """Describe one configured sensor for browser selection.

    ``view`` names the frontend renderer this sensor's stream can drive, so a
    console can offer a sensor it was not written against. A sensor with no view
    still records; it simply has nothing to draw.
    """

    name: str
    kind: str
    view: str | None
    started: bool
    sample_rate_hz: float | None
    age_seconds: float | None

    def to_dict(self) -> dict[str, object]:
        """Serialize the sensor for the HTTP API."""
        return {
            "name": self.name,
            "kind": self.kind,
            "view": self.view,
            "started": self.started,
            "sample_rate_hz": self.sample_rate_hz,
            "age_seconds": self.age_seconds,
        }


@dataclass(frozen=True)
class SensorReading:
    """One timestamped vector sample from an auxiliary observation source.

    Two clocks are recorded because they answer different questions.
    ``timestamp`` is this process's monotonic clock at the moment the sample was
    collected, which is what aligns a reading against leader samples and camera
    captures. ``source_timestamp`` is whatever the sensor itself reported, kept
    unmodified so that a driver running in another process can still be checked
    for staleness on its own terms.
    """

    name: str
    timestamp: float
    source_timestamp: float | None
    labels: tuple[str, ...]
    values: tuple[float, ...]


@runtime_checkable
class SensorSource(Protocol):
    """Read timestamped vector samples from one auxiliary sensor."""

    @property
    def name(self) -> str:
        """Return the configured identifier for this sensor."""

    @property
    def labels(self) -> tuple[str, ...]:
        """Return one label per value, in the order values are reported."""

    def start(self) -> None:
        """Begin acquiring samples, off the caller's thread."""

    def read_latest(self) -> SensorReading | None:
        """Return the newest sample without blocking, or None if none is ready."""

    def close(self) -> None:
        """Release the transport and stop acquiring."""


class _SampleStore:
    """Hold the newest sample from one full-rate reader thread.

    A single reader means the sensor's own signal derivation advances exactly
    once per hardware sample. When two consumers polled the source directly they
    both advanced its per-finger vibration history, so that signal's meaning
    depended on how often it happened to be read: measured 3.14 with a display
    attached against 53.03 without, for identical motion.
    """

    def __init__(self, capacity: int = 4096) -> None:
        self._condition = threading.Condition()
        self._latest: object | None = None
        self._sequence = 0
        self._monotonic = 0.0
        self._pending: deque = deque(maxlen=capacity)
        self._first = 0.0
        self._count = 0

    def publish(self, reading: object) -> None:
        """Record one sample and wake everything waiting for a new one."""
        now = time.monotonic()
        with self._condition:
            self._latest = reading
            self._monotonic = now
            self._sequence += 1
            self._pending.append((now, reading))
            if self._count == 0:
                self._first = now
            self._count += 1
            self._condition.notify_all()

    def latest(self) -> tuple[object | None, float]:
        """Return the newest sample and the monotonic time it arrived."""
        with self._condition:
            return self._latest, self._monotonic

    def next_after(self, sequence: int, timeout: float = 1.0) -> tuple[object | None, int]:
        """Block until a sample newer than ``sequence`` arrives, or time out."""
        with self._condition:
            if self._sequence <= sequence:
                self._condition.wait(timeout)
            return self._latest, self._sequence

    def drain(self) -> list[tuple[float, object]]:
        """Take every sample buffered since the last call.

        The buffer is bounded, so a consumer that stops draining loses the
        oldest samples rather than growing without limit. Intended for a
        recorder that wants the full rate rather than a periodic snapshot.
        """
        with self._condition:
            taken = list(self._pending)
            self._pending.clear()
        return taken

    def observed_rate_hz(self) -> float:
        """Return the mean sample rate since the first sample, or zero."""
        with self._condition:
            elapsed = self._monotonic - self._first
            return float(self._count / elapsed) if elapsed > 0 else 0.0


class _NextSample:
    """Expose a store as a blocking source for the display throttle.

    ``iter_payloads`` throttles by discarding samples until the next display
    deadline, which only paces correctly against a source that blocks. Handing
    it the store rather than a cached value keeps that contract, and means the
    display costs no reads of its own.
    """

    def __init__(self, store: _SampleStore) -> None:
        self._store = store
        self._sequence = 0

    def read_latest(self) -> object:
        """Block until the reader publishes a sample newer than the last seen."""
        reading, self._sequence = self._store.next_after(self._sequence)
        return reading


class EFleshTactileSensor:
    """Adapt the eflesh package's source to the auxiliary sensor interface.

    Acquisition, resync framing, baselining and the derived tactile signals all
    live in the ``eflesh`` package and are deliberately not reimplemented here.
    This class does two things: it converts an eFlesh reading into the flat
    :class:`SensorReading` the hub records, and it exposes the display payloads
    the web layer needs, so that layer never imports eflesh internals or copies
    its hand-verified geometry constants.

    The finger count is left unset so the source detects it from the stream.
    The firmware sizes its frame at boot from however many boards enumerated,
    and a host that assumes the wrong width sits in its resync loop forever
    rather than raising, which is a hang rather than an error.
    """

    #: Frontend renderer this sensor's stream can drive.
    view = "tactile"

    def __init__(self, config: SensorConfig, source_factory: object | None = None) -> None:
        self._name = config.name
        self._config = config
        self._source = None
        self._store = _SampleStore()
        self._reader: threading.Thread | None = None
        self._stop = threading.Event()
        self._factory = source_factory
        if source_factory is None:
            try:
                from eflesh import EFleshSource
            except ImportError as error:  # pragma: no cover - host dependency
                raise SensorError(
                    "The eflesh package is missing. Install with `pip install -e '.[tactile]'`."
                ) from error
            self._factory = EFleshSource

    @property
    def name(self) -> str:
        """Return the configured identifier for this sensor."""
        return self._name

    @property
    def labels(self) -> tuple[str, ...]:
        """Return one label per channel, empty until the finger count is known."""
        if self._source is None or self._source.num_fingers is None:
            return ()
        return tuple(self._source.labels)

    @property
    def num_fingers(self) -> int | None:
        """Return the detected finger count, or None before starting."""
        return None if self._source is None else self._source.num_fingers

    def start(self) -> None:
        """Open the stream, detect the finger count, and capture a baseline."""
        assert self._factory is not None
        self._source = self._factory(
            # Resolved here rather than at configuration load, so a config stays
            # valid while the hardware is detached.
            port=resolve_serial_port(self._config.port),
            settle=self._config.settle,
            name=self._name,
        )
        self._source.start()
        self._stop.clear()
        self._reader = threading.Thread(
            target=self._read_forever, name=f"sensor-{self._name}", daemon=True
        )
        self._reader.start()

    def _read_forever(self) -> None:
        """Drain the sensor at its own rate, publishing every sample."""
        assert self._source is not None
        while not self._stop.is_set():
            started = time.monotonic()
            try:
                reading = self._source.read_latest()
            except Exception as error:  # noqa: BLE001 - a sensor cannot fault the robot
                _LOGGER.warning("Sensor %s stopped reading: %s", self._name, error)
                return
            if reading is not None:
                self._store.publish(reading)
            # Real hardware paces this by blocking on serial. A source that
            # returns instantly, such as the hardware-free fake, would otherwise
            # spin a core, so an idle read is slowed to the cap.
            remaining = MIN_SAMPLE_PERIOD_SECONDS - (time.monotonic() - started)
            if remaining > 0:
                self._stop.wait(remaining)

    @property
    def sample_rate_hz(self) -> float:
        """Return the mean rate at which samples have actually arrived."""
        return self._store.observed_rate_hz()

    def age_seconds(self) -> float | None:
        """Return how long ago the newest sample arrived, or None if none has."""
        _reading, monotonic = self._store.latest()
        return None if monotonic == 0.0 else time.monotonic() - monotonic

    def drain(self) -> list:
        """Take every sample buffered since the last call, for a recorder."""
        return self._store.drain()

    def read_latest(self) -> SensorReading | None:
        """Return the newest sample as a flat labelled vector.

        Reads the reader thread's snapshot rather than the sensor, so sampling
        the sensor for telemetry cannot change what any other consumer sees.
        """
        if self._source is None:
            return None
        reading, _monotonic = self._store.latest()
        if reading is None:
            return None
        return SensorReading(
            name=self._name,
            timestamp=time.monotonic(),
            source_timestamp=float(reading.timestamp),
            labels=self.labels,
            values=tuple(float(value) for value in np.asarray(reading.values).ravel()),
        )

    def close(self) -> None:
        """Stop the reader and release the transport."""
        self._stop.set()
        reader, self._reader = self._reader, None
        if reader is not None and reader is not threading.current_thread():
            reader.join(timeout=2.0)
        source, self._source = self._source, None
        if source is not None:
            source.close()

    def geometry_payload(self) -> dict:
        """Return the static pad description to send once per client.

        Raises:
            SensorError: If the sensor has not started, so the finger count and
                therefore the geometry are still unknown.
        """
        from eflesh.web import geometry_payload

        if self._source is None or self._source.num_fingers is None:
            raise SensorError(f"Sensor '{self._name}' has not started; geometry is unknown")
        return geometry_payload(self._source.num_fingers)

    def display_payloads(self, hz: float = 60.0, stop: object | None = None) -> object:
        """Yield display frames at a bounded rate, dropping stale samples.

        The sensor streams far faster than a browser can paint, so the full rate
        belongs in the backend for recording while the display is throttled.

        Raises:
            SensorError: If the sensor has not started.
        """
        from eflesh.web import iter_payloads

        if self._source is None:
            raise SensorError(f"Sensor '{self._name}' has not started")
        # Fed from the reader's store rather than the sensor, so the display
        # costs no reads of its own and cannot perturb the recorded signals.
        return iter_payloads(_NextSample(self._store), hz=hz, stop=stop)

    def rebaseline(self) -> None:
        """Re-zero every channel. The sensor must be untouched and settled."""
        if self._source is None:
            raise SensorError(f"Sensor '{self._name}' has not started")
        self._source.rebaseline()

    def set_gap_baseline(self, samples: object) -> None:
        """Replace the baseline with one conditioned on the gripper gap.

        Hook only, not yet wired to anything. Two cuboids on opposing fingers
        see and physically repel each other as a function of the finger gap, so
        a single static baseline drifts into phantom contact as the gripper
        closes. Correcting it needs a sweep logged against the gripper encoder;
        call this with the interpolated vector as the gap changes.

        Raises:
            SensorError: If the sensor has not started.
        """
        if self._source is None:
            raise SensorError(f"Sensor '{self._name}' has not started")
        # The package exposes the setter on the processor rather than the
        # source, so this reaches one level in by design.
        self._source._proc.set_baseline(samples)


def _fake_eflesh_source(port: str, settle: float, name: str) -> object:
    """Build a hardware-free eFlesh source, ignoring the transport settings."""
    from eflesh import FakeEFleshSource

    return FakeEFleshSource(num_fingers=2, name=name)


class FakeEFleshTactileSensor(EFleshTactileSensor):
    """Synthetic eFlesh sensor for exercising the console without hardware.

    Same interface and the same derived signals, driven by generated data. Named
    distinctly in configuration so a synthetic feed can never be mistaken for a
    measurement.
    """

    def __init__(self, config: SensorConfig, source_factory: object | None = None) -> None:
        super().__init__(config, source_factory=source_factory or _fake_eflesh_source)


_SENSOR_DRIVERS: dict[str, type] = {
    "eflesh": EFleshTactileSensor,
    "eflesh_fake": FakeEFleshTactileSensor,
}


def build_sensor(config: SensorConfig) -> SensorSource:
    """Construct one sensor from its configured kind.

    Args:
        config: Validated configuration for a single sensor.

    Returns:
        A started-capable source for the requested hardware.

    Raises:
        SensorError: If the kind has no registered driver.
    """
    try:
        driver = _SENSOR_DRIVERS[config.kind]
    except KeyError as error:
        raise SensorError(f"Unsupported sensor kind '{config.kind}'") from error
    return driver(config)


class SensorHub:
    """Own auxiliary sensors and keep their failures away from robot control.

    Every method is total: a source that raises is reported once, then dropped
    for the remainder of the run. Retrying a wedged transport on each control
    cycle would cost latency in the loop that matters most, and a sensor that
    has already failed is unlikely to recover on its own.
    """

    def __init__(self, sources: tuple[SensorSource, ...] = ()) -> None:
        self._sources = list(sources)
        self._failed: set[str] = set()
        self._started = False

    @property
    def names(self) -> tuple[str, ...]:
        """Return the identifiers of sensors that have not failed."""
        return tuple(source.name for source in self._sources if source.name not in self._failed)

    @property
    def failed(self) -> tuple[str, ...]:
        """Return the identifiers of sensors dropped after an error."""
        return tuple(sorted(self._failed))

    def describe(self, configs: tuple[SensorConfig, ...] = ()) -> tuple[SensorInfo, ...]:
        """Describe every configured sensor, including any that failed.

        Args:
            configs: Configurations, used to report the kind of each sensor and
                to list sensors that could not be constructed at all.

        Returns:
            One entry per configured sensor, in configuration order.
        """
        kinds = {config.name: config.kind for config in configs}
        by_name = {source.name: source for source in self._sources}
        names = list(kinds) or [source.name for source in self._sources]
        described = []
        for name in names:
            source = by_name.get(name)
            rate = getattr(source, "sample_rate_hz", None)
            age = getattr(source, "age_seconds", None)
            described.append(
                SensorInfo(
                    name=name,
                    kind=kinds.get(name, ""),
                    view=getattr(source, "view", None),
                    started=source is not None and name not in self._failed,
                    sample_rate_hz=None if rate is None else float(rate),
                    age_seconds=age() if callable(age) else None,
                )
            )
        return tuple(described)

    def source(self, name: str) -> SensorSource | None:
        """Return the live sensor with this name, or None if absent or failed.

        Args:
            name: Configured sensor identifier.

        Returns:
            The source, or None when it does not exist or has been dropped.
        """
        if name in self._failed:
            return None
        for source in self._sources:
            if source.name == name:
                return source
        return None

    def _drop(self, source: SensorSource, action: str, error: Exception) -> None:
        self._failed.add(source.name)
        _LOGGER.warning(
            "Sensor %s failed to %s and will be ignored for this run: %s",
            source.name,
            action,
            error,
        )

    def start(self) -> None:
        """Start every sensor once, dropping any that cannot be started.

        Repeated calls are ignored. A run may be stopped and started again
        within one session, but an acquirer that has been joined cannot be
        restarted, so sensors outlive individual runs and close with the owner.
        """
        if self._started:
            return
        self._started = True
        for source in self._sources:
            if source.name in self._failed:
                continue
            try:
                source.start()
            except Exception as error:  # noqa: BLE001 - a sensor cannot fault the robot
                self._drop(source, "start", error)

    def read_all(self) -> dict[str, SensorReading]:
        """Return the newest reading from each live sensor.

        Returns:
            Readings keyed by sensor name. Sensors that have not produced a
            sample yet are absent; sensors that raise are dropped and absent
            from every subsequent call.
        """
        readings: dict[str, SensorReading] = {}
        for source in self._sources:
            if source.name in self._failed:
                continue
            try:
                reading = source.read_latest()
            except Exception as error:  # noqa: BLE001 - a sensor cannot fault the robot
                self._drop(source, "read", error)
                continue
            if reading is not None:
                readings[reading.name] = reading
        return readings

    def close(self) -> None:
        """Close every sensor, reporting but not propagating failures."""
        for source in self._sources:
            try:
                source.close()
            except Exception as error:  # noqa: BLE001 - cleanup must continue
                _LOGGER.warning("Sensor %s failed to close: %s", source.name, error)
        self._sources.clear()
        self._started = False


def build_sensor_hub(configs: tuple[SensorConfig, ...]) -> SensorHub:
    """Construct a hub for every configured sensor.

    A sensor that cannot even be constructed is reported and skipped, so an
    unplugged or misconfigured accessory never prevents teleoperation.

    Args:
        configs: Validated sensor configurations.

    Returns:
        A hub owning each sensor that could be constructed.
    """
    sources: list[SensorSource] = []
    for config in configs:
        try:
            sources.append(build_sensor(config))
        except Exception as error:  # noqa: BLE001 - a sensor cannot fault the robot
            _LOGGER.warning("Sensor %s could not be created: %s", config.name, error)
    return SensorHub(tuple(sources))
