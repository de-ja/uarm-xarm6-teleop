"""eFlesh magnetic tactile sensor: acquisition and derived signals.

Typical use in a teleop stack::

    from eflesh import EFleshSource

    src = EFleshSource(port="/dev/ttyACM0")   # detects 1 or 2 fingers
    src.start()
    r = src.read_latest()
    r.signals.fingers[0].force        # uncalibrated contact intensity
    r.signals.balance                 # -1..+1 across the pair
    src.close()

See PROTOCOL.md for the wire format and the gotchas a driver must handle.
"""

from .signals import (
    CENTROID_FLOOR_UT,
    CHIP_NAMES,
    CHIP_POSITIONS_MM,
    CHIP_XY_ROTATIONS,
    CONTACT_THRESHOLD_UT,
    MAGS_PER_FINGER,
    QUANT_UT,
    FingerProcessor,
    FingerSignals,
    GripperProcessor,
    GripperSignals,
    to_pad_frame,
)
from .stream import (
    EFleshSource,
    FakeEFleshSource,
    SensorReading,
    channel_labels,
    detect_num_mags,
)

__all__ = [
    "CENTROID_FLOOR_UT", "CHIP_NAMES", "CHIP_POSITIONS_MM", "CHIP_XY_ROTATIONS",
    "CONTACT_THRESHOLD_UT", "MAGS_PER_FINGER", "QUANT_UT",
    "FingerProcessor", "FingerSignals", "GripperProcessor", "GripperSignals",
    "to_pad_frame",
    "EFleshSource", "FakeEFleshSource", "SensorReading", "channel_labels",
    "detect_num_mags",
]
