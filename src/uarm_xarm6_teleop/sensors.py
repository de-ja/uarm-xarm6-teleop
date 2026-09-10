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
import time
from dataclasses import dataclass

import numpy as np
from typing import Protocol, runtime_checkable

from .config import SensorConfig
from .serial_ports import resolve_serial_port


_LOGGER = logging.getLogger(__name__)


class SensorError(RuntimeError):
    """Raised when a sensor cannot be constructed or started."""


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

    def __init__(self, config: SensorConfig, source_factory: object | None = None) -> None:
        self._name = config.name
        self._config = config
        self._source = None
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

    def read_latest(self) -> SensorReading | None:
        """Return the newest baselined sample as a flat labelled vector."""
        if self._source is None:
            return None
        reading = self._source.read_latest()
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
        """Stop streaming and release the transport."""
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
        return iter_payloads(self._source, hz=hz, stop=stop)

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


_SENSOR_DRIVERS: dict[str, type] = {"eflesh": EFleshTactileSensor}


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
