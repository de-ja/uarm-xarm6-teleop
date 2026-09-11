#!/usr/bin/env python
"""Serial acquisition for eFlesh, shaped as a pollable sensor source.

`EFleshSource` implements the start / read_latest / close lifecycle that a
teleop stack's sensor registry expects, and `FakeEFleshSource` implements the
same interface with no hardware so the seam can be tested in CI.

Reading the port is delegated to `anyskin.AnySkinProcess`, which already runs
the reader in its own process, timestamps every sample, and does the
fixed-length-plus-resync framing the wire format requires. See PROTOCOL.md.
"""

from dataclasses import dataclass
from typing import List, Optional

import time
import numpy as np

from .signals import (
    CHIP_NAMES,
    MAGS_PER_FINGER,
    GripperProcessor,
    GripperSignals,
)

AXES = ("bx", "by", "bz")
BAUDRATE = 115200


def detect_num_mags(port: str, baudrate: int = BAUDRATE, timeout: float = 2.0) -> int:
    """Work out how many magnetometers the firmware is actually streaming.

    The mux firmware sizes its frame from however many boards it found at boot,
    so a board that fails to enumerate silently shrinks the frame from 162 bytes
    to 82. A host that assumes the larger size never sees a valid frame and sits
    in its resync loop forever rather than reporting an error, so measure the
    width instead of assuming it.
    """
    import serial

    with serial.Serial(port, baudrate, timeout=0.3) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.read_until(b"\r\n")            # discard the partial frame
        deadline = time.time() + timeout
        sizes = []
        while time.time() < deadline and len(sizes) < 20:
            n = len(ser.read_until(b"\r\n"))
            if n > 2:
                sizes.append(n)

    if not sizes:
        raise RuntimeError(f"No frames on {port}. Is the firmware running?")

    width = max(set(sizes), key=sizes.count)
    if (width - 2) % 16:
        raise RuntimeError(
            f"Frame of {width} bytes on {port} is not 16*num_mags+2; "
            "the stream may be misframed or the wrong firmware is loaded."
        )
    return (width - 2) // 16


def channel_labels(num_fingers: int) -> List[str]:
    """Names for the raw channels, in stream order."""
    return [
        f"f{f}_{chip}_{axis}"
        for f in range(num_fingers)
        for chip in CHIP_NAMES
        for axis in AXES
    ]


@dataclass
class SensorReading:
    """One sample: raw channels plus everything derived from them."""

    timestamp: float
    values: np.ndarray
    """(num_fingers * 15,) baselined dB in microtesla, aligned with `labels`."""

    signals: GripperSignals
    """Per-finger force, shear, centroid, contact, vibration; grip balance."""


class EFleshSource:
    """Polled eFlesh sensor source.

    Parameters
    ----------
    port : serial device, e.g. "/dev/ttyACM0".
    num_fingers : None to detect from the stream (recommended), else 1 or 2.
    settle : seconds to wait before the initial baseline. A baseline taken
        immediately after the sensor has been handled reads several times
        noisier than the true floor, so do not set this to zero.
    name : identifier for the sensor registry.
    """

    def __init__(
        self,
        port: str,
        num_fingers: Optional[int] = None,
        settle: float = 5.0,
        name: str = "eflesh",
        history: int = 16,
        calibration: Optional[dict] = None,
    ):
        self._name = name
        self.port = port
        self.settle = settle
        self._requested_fingers = num_fingers
        self._history = history
        self._calibration = calibration
        self.num_fingers: Optional[int] = None
        self._sensor = None
        self._proc: Optional[GripperProcessor] = None

    @property
    def name(self) -> str:
        return self._name

    @property
    def labels(self) -> List[str]:
        if self.num_fingers is None:
            raise RuntimeError("Call start() before reading labels.")
        return channel_labels(self.num_fingers)

    def start(self) -> None:
        from anyskin import AnySkinProcess

        if self._requested_fingers is None:
            num_mags = detect_num_mags(self.port)
            if num_mags % MAGS_PER_FINGER:
                raise RuntimeError(
                    f"{num_mags} magnetometers is not a whole number of fingers."
                )
            self.num_fingers = num_mags // MAGS_PER_FINGER
        else:
            self.num_fingers = self._requested_fingers
            num_mags = self.num_fingers * MAGS_PER_FINGER

        self._sensor = AnySkinProcess(num_mags=num_mags, port=self.port)
        self._sensor.start()
        time.sleep(1.0)

        self._proc = GripperProcessor(
            num_fingers=self.num_fingers,
            history=self._history,
            calibration=self._calibration,
        )
        time.sleep(self.settle)
        self.rebaseline()

    def rebaseline(self, num_samples: int = 20) -> None:
        """Re-zero every channel. The sensor must be untouched and settled."""
        raw = np.array(self._sensor.get_data(num_samples=num_samples))[:, 1:]
        self._proc.set_baseline(raw)

    def read_latest(self) -> SensorReading:
        sample = self._sensor.get_data(num_samples=1)[0]
        timestamp, flat = float(sample[0]), np.asarray(sample[1:], dtype=float)
        signals = self._proc(flat)
        values = np.concatenate([f.dB.reshape(-1) for f in signals.fingers])
        return SensorReading(timestamp=timestamp, values=values, signals=signals)

    def close(self) -> None:
        if self._sensor is not None:
            self._sensor.pause_streaming()
            self._sensor.join()
            self._sensor = None


class FakeEFleshSource(EFleshSource):
    """Hardware-free stand-in with the same interface.

    Produces quantisation-scale noise plus an optional synthetic press, so the
    registry, the config plumbing and any consumer can be tested without a
    sensor attached.
    """

    def __init__(self, num_fingers: int = 2, name: str = "eflesh_fake",
                 press_hz: float = 0.4, press_ut: float = 600.0, **kwargs):
        super().__init__(port="", num_fingers=num_fingers, settle=0.0,
                         name=name, **kwargs)
        self.press_hz = press_hz
        self.press_ut = press_ut
        self._t0 = 0.0

    def start(self) -> None:
        self.num_fingers = self._requested_fingers
        self._proc = GripperProcessor(
            num_fingers=self.num_fingers,
            history=self._history,
            calibration=self._calibration,
        )
        self._proc.set_baseline(np.zeros((1, self.num_fingers * MAGS_PER_FINGER * 3)))
        self._t0 = time.time()

    def rebaseline(self, num_samples: int = 20) -> None:
        samples = np.array([self._raw() for _ in range(num_samples)])
        self._proc.set_baseline(samples)

    def _raw(self) -> np.ndarray:
        from .signals import QUANT_UT

        n = self.num_fingers * MAGS_PER_FINGER * 3
        v = np.random.normal(0.0, QUANT_UT, n)
        phase = 2 * np.pi * self.press_hz * (time.time() - self._t0)
        depth = self.press_ut * max(0.0, np.sin(phase))
        for f in range(self.num_fingers):
            v[f * 15 + 2] -= depth          # middle chip, Bz
            v[f * 15 + 5] -= depth * 0.6    # left chip, Bz
        return v

    def read_latest(self) -> SensorReading:
        flat = self._raw()
        signals = self._proc(flat)
        values = np.concatenate([f.dB.reshape(-1) for f in signals.fingers])
        return SensorReading(timestamp=time.time(), values=values, signals=signals)

    def close(self) -> None:
        pass
