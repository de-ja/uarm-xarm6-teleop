#!/usr/bin/env python
"""JSON payloads for a browser frontend.

Framework-agnostic on purpose: these functions turn readings into plain dicts
of Python scalars and lists, ready for `json.dumps`. Wiring them to a
WebSocket, SSE stream or polling endpoint is a few lines in whatever server you
already run -- see eflesh/README.md.

Send `geometry_payload()` once when a client connects, then `reading_payload()`
per frame. The geometry is what lets the frontend draw the pad without
hardcoding constants that were verified by hand on hardware.
"""

import time
from typing import Iterator, Optional

import numpy as np

from .signals import (
    CHIP_NAMES,
    CHIP_POSITIONS_MM,
    CONTACT_THRESHOLD_UT,
    MAGS_PER_FINGER,
    QUANT_UT,
    FingerSignals,
    GripperSignals,
    to_pad_frame,
)

#: Half-width of the sensor pad in millimetres. The cuboid body is 40.33 mm
#: square, so the drawable area is a little over +/-20 mm.
PAD_HALF_MM = 20.0


def geometry_payload(num_fingers: int = 2) -> dict:
    """Static description of the sensor, sent once per client connection."""
    return {
        "num_fingers": num_fingers,
        "mags_per_finger": MAGS_PER_FINGER,
        "chip_names": list(CHIP_NAMES),
        "chip_positions_mm": CHIP_POSITIONS_MM.tolist(),
        "pad_half_mm": PAD_HALF_MM,
        "contact_threshold_ut": CONTACT_THRESHOLD_UT,
        "quantisation_ut": QUANT_UT,
    }


def finger_payload(finger: FingerSignals) -> dict:
    """One finger's signals as JSON-safe primitives.

    `shear_xy` is already rotated into the shared pad frame, so the frontend can
    draw it directly without knowing the per-chip rotations.
    """
    return {
        "contact": bool(finger.contact),
        "force": float(finger.force),
        "normal": float(finger.normal),
        "shear": [float(v) for v in finger.shear],
        "shear_magnitude": float(finger.shear_magnitude),
        "shear_angle": float(finger.shear_angle),
        "vibration": float(finger.vibration),
        "centroid_mm": (None if finger.centroid is None
                        else [float(v) for v in finger.centroid]),
        "per_chip": [float(v) for v in finger.per_chip],
        "dbz": [float(v) for v in finger.dB[:, 2]],
        "shear_xy": to_pad_frame(finger.dB).tolist(),
        "calibrated": {k: float(v) for k, v in finger.calibrated.items()},
    }


def signals_payload(signals: GripperSignals) -> dict:
    balance = signals.balance
    return {
        "fingers": [finger_payload(f) for f in signals.fingers],
        "any_contact": bool(signals.any_contact),
        "grip_force": float(signals.grip_force),
        "balance": None if balance is None else float(balance),
    }


def reading_payload(reading, include_raw: bool = False) -> dict:
    """A full frame for the browser.

    `include_raw` adds the 30 baselined channels. Leave it off for display --
    the derived signals are what a frontend draws, and the raw channels roughly
    triple the payload for nothing.
    """
    out = {"t": float(reading.timestamp)}
    out.update(signals_payload(reading.signals))
    if include_raw:
        out["values"] = [float(v) for v in np.asarray(reading.values).ravel()]
    return out


def iter_payloads(source, hz: float = 60.0,
                  include_raw: bool = False,
                  stop: Optional[callable] = None) -> Iterator[dict]:
    """Yield display frames at a bounded rate.

    The sensor streams at roughly 194 Hz with two boards, which is far more than
    a browser can paint and more than a human can perceive. Throttle for
    display and keep the full-rate stream in the backend for recording; pushing
    every sample to the client wastes bandwidth and stalls the render loop.

    Drains to the newest sample each tick, so the client always sees current
    data rather than a growing backlog.
    """
    period = 1.0 / hz
    next_due = time.monotonic()
    while stop is None or not stop():
        reading = source.read_latest()
        now = time.monotonic()
        if now < next_due:
            continue                     # drop this sample, it would not be drawn
        next_due = now + period
        yield reading_payload(reading, include_raw=include_raw)
