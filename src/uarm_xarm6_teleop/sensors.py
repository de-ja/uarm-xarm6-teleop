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
from typing import Protocol, runtime_checkable

from .config import SensorConfig
from .serial_ports import resolve_serial_port


_LOGGER = logging.getLogger(__name__)

# eFlesh streams magnetometers in physical position order, not address order.
EFLESH_POSITIONS = ("middle", "left", "right", "top", "bottom")
EFLESH_AXES = ("bx", "by", "bz")
EFLESH_MAGS_PER_BOARD = len(EFLESH_POSITIONS)
# A failed magnetometer read surfaces as raw 0xFFFF converted, 78640.8 uT. Full
# scale is 126.9 mT on Z, so nothing legitimate approaches this.
EFLESH_INVALID_MICROTESLA = 50000.0


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


def eflesh_labels(num_mags: int, fingers: tuple[str, ...]) -> tuple[str, ...]:
    """Build one label per streamed value for an eFlesh configuration.

    Args:
        num_mags: Total magnetometers across all boards, five per board.
        fingers: Name for each board, in ascending mux channel order.

    Returns:
        ``finger_position_axis`` labels ordered to match the value stream.

    Raises:
        SensorError: If the magnetometer count and finger names disagree.
    """
    if num_mags <= 0 or num_mags % EFLESH_MAGS_PER_BOARD:
        raise SensorError(f"eFlesh num_mags must be a positive multiple of {EFLESH_MAGS_PER_BOARD}")
    boards = num_mags // EFLESH_MAGS_PER_BOARD
    if len(fingers) != boards:
        raise SensorError(
            f"eFlesh num_mags={num_mags} implies {boards} board(s), but "
            f"{len(fingers)} finger name(s) were configured"
        )
    return tuple(
        f"{finger}_{position}_{axis}"
        for finger in fingers
        for position in EFLESH_POSITIONS
        for axis in EFLESH_AXES
    )


class EFleshSensor:
    """Stream an eFlesh magnetic tactile array through the anyskin reader.

    The serial framing is deliberately not reimplemented here. ``anyskin`` runs
    its reader in a separate process, timestamps each sample, and performs the
    fixed-length-plus-resync framing the sensor requires, which also satisfies
    this module's rule that acquisition stays off the control thread.

    Values are absolute field dominated by a large DC offset from the cuboid's
    own magnets, so they are reported raw and unbaselined. Baselining belongs
    downstream: on a gripper the neighbouring array's contribution varies with
    the finger gap, so a correct baseline has to be conditioned on gripper
    position and cannot be computed from the sample stream alone.
    """

    def __init__(self, config: SensorConfig, process_factory: object | None = None) -> None:
        self._name = config.name
        self._labels = eflesh_labels(config.num_mags, config.fingers)
        self._config = config
        self._process = None
        self._factory = process_factory
        if process_factory is None:
            try:
                from anyskin import AnySkinProcess
            except ImportError as error:  # pragma: no cover - host dependency
                raise SensorError(
                    "The anyskin package is missing. Install with `pip install -e '.[tactile]'`."
                ) from error
            self._factory = AnySkinProcess

    @property
    def name(self) -> str:
        """Return the configured identifier for this sensor."""
        return self._name

    @property
    def labels(self) -> tuple[str, ...]:
        """Return one ``finger_position_axis`` label per streamed value."""
        return self._labels

    def start(self) -> None:
        """Open the serial stream and begin background acquisition."""
        assert self._factory is not None
        # temp_filtered drops the per-magnetometer temperature channel, leaving
        # three field axes each, which is what the labels above describe.
        self._process = self._factory(
            num_mags=self._config.num_mags,
            # Resolved here rather than at configuration load, so a config
            # stays valid while the hardware is detached.
            port=resolve_serial_port(self._config.port),
            temp_filtered=True,
        )
        self._process.start()
        self._process.start_streaming()

    def read_latest(self) -> SensorReading | None:
        """Return the newest sample, or None before the stream produces one."""
        if self._process is None:
            return None
        reading = self._process.last_reading
        if reading is None or len(reading) < 2:
            return None
        source_timestamp = float(reading[0])
        values = tuple(float(value) for value in reading[1:])
        if len(values) != len(self._labels):
            raise SensorError(
                f"{self._name} reported {len(values)} values but "
                f"{len(self._labels)} labels are configured"
            )
        if any(abs(value) > EFLESH_INVALID_MICROTESLA for value in values):
            # Firmware normally holds the last good sample, so this is a
            # belt-and-braces check rather than an expected path.
            _LOGGER.warning("%s reported an out-of-range magnetometer value", self._name)
            return None
        return SensorReading(
            name=self._name,
            timestamp=time.monotonic(),
            source_timestamp=source_timestamp,
            labels=self._labels,
            values=values,
        )

    def close(self) -> None:
        """Stop streaming and join the reader process."""
        process, self._process = self._process, None
        if process is None:
            return
        process.pause_streaming()
        process.join()


_SENSOR_DRIVERS: dict[str, type] = {"eflesh": EFleshSensor}


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
