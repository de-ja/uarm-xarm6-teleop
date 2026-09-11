#!/usr/bin/env python
"""Derived tactile signals from eFlesh magnetometer data.

Pure numpy -- no pygame, no serial -- so the visualizer, a teleop client and a
data recorder can all import the same definitions and agree on what "force" or
"contact" means.

The eFlesh paper characterises three quantities (see characterization/):
normal force, shear force, and spatial resolution. Each has a probe dataset and
an MLP in characterization/train.py that regresses the quantity from baselined
dB. No checkpoints ship with the repo, and a model fitted on the authors' build
will not transfer to another one -- channel order, magnet polarity and seating
all change the mapping.

So this module computes *uncalibrated proxies* for those three quantities from
geometry alone. They are monotone in the physical quantity and immediately
usable for operator feedback, which mostly needs responsiveness and legibility
rather than newtons. Attach a trained model via `FingerProcessor.calibration`
to convert a proxy into physical units once you have probe data from your own
rig.
"""

from dataclasses import dataclass, field
from collections import deque
from typing import Callable, Optional, Sequence

import numpy as np

MAGS_PER_FINGER = 5

#: Stream order, which is physical position order rather than I2C address order.
CHIP_NAMES = ("middle", "left", "right", "top", "bottom")

#: Chip positions in the sensor plane, millimetres, origin at the pad centre.
#: Chips sit on the axes at +/-7 mm; the magnets sit on the corners at +/-8 mm,
#: so no chip is directly beneath a magnet.
CHIP_POSITIONS_MM = np.array(
    [[0.0, 0.0], [-7.0, 0.0], [7.0, 0.0], [0.0, 7.0], [0.0, -7.0]]
)

#: Per-chip rotation taking that chip's local in-plane axes into a common pad
#: frame. Verified on hardware with a handheld magnet: held beside a chip the
#: resulting vector points along the chip-magnet line, and directly overhead it
#: collapses to zero. See DEFAULT_CUBOID_SPECS.md.
CHIP_XY_ROTATIONS = np.array([-np.pi / 2, -np.pi / 2, np.pi, np.pi / 2, 0.0])

#: XY quantisation step at the firmware's gain and resolution settings. The
#: sensor is quantisation-limited, so this is also very close to the noise floor.
QUANT_UT = 2.4

#: Total ||dB|| below which the pad is treated as untouched. Matches the
#: threshold the original single-sensor visualizer used.
CONTACT_THRESHOLD_UT = 200.0

#: Per-chip floor for centroid weighting: below this a chip contributes nothing,
#: which keeps the centroid from wandering on noise alone.
CENTROID_FLOOR_UT = 3.0 * QUANT_UT


def _rotation(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    return np.array([[c, -s], [s, c]])


_ROTATIONS = np.stack([_rotation(t) for t in CHIP_XY_ROTATIONS])


def to_pad_frame(dB: np.ndarray) -> np.ndarray:
    """Rotate each chip's in-plane dB into the shared pad frame.

    Parameters
    ----------
    dB : (5, 3) baselined field change in microtesla.

    Returns
    -------
    (5, 2) in-plane vectors, all expressed in the same frame so they can be
    summed or compared across chips.
    """
    # The sign flip on x and y is part of the verified sensor-to-pad mapping.
    xy = -dB[:, :2]
    return np.einsum("nij,nj->ni", _ROTATIONS, xy)


@dataclass
class FingerSignals:
    """Everything derived from one finger's five magnetometers."""

    dB: np.ndarray
    """(5, 3) baselined field change, microtesla."""

    per_chip: np.ndarray
    """(5,) ||dB|| per chip, microtesla."""

    force: float
    """||dB|| over all 15 channels. Uncalibrated proxy for total contact
    intensity. Polarity-invariant, so it survives a differently-built cuboid."""

    normal: float
    """Sum of |dBz| across chips. Uncalibrated proxy for normal force."""

    shear: np.ndarray
    """(2,) resultant in-plane vector in the pad frame, microtesla.
    Uncalibrated proxy for shear force; its direction is meaningful."""

    centroid: Optional[np.ndarray]
    """(2,) magnitude-weighted contact location in millimetres, or None when
    there is no contact. This is the geometry-only version of the paper's
    spatial-resolution output."""

    contact: bool
    """Whether `force` exceeds the contact threshold."""

    vibration: float
    """RMS of successive differences across all channels, in units of the
    quantisation step. Rises during sliding and micro-slip, but also during any
    fast change -- it is an activity measure, not a trained slip detector."""

    calibrated: dict = field(default_factory=dict)
    """Outputs of any attached calibration models, by name."""

    @property
    def shear_magnitude(self) -> float:
        return float(np.linalg.norm(self.shear))

    @property
    def shear_angle(self) -> float:
        """Direction of shear in the pad frame, radians."""
        return float(np.arctan2(self.shear[1], self.shear[0]))


class FingerProcessor:
    """Baseline tracking and signal derivation for one finger.

    Parameters
    ----------
    history : number of samples retained for the vibration estimate.
    calibration : optional mapping of name -> callable taking the (5, 3) dB
        array and returning a scalar. Use it to plug in an MLP trained by
        characterization/train.py once you have probe data from your own rig.
    """

    def __init__(
        self,
        history: int = 16,
        calibration: Optional[dict] = None,
    ):
        self.baseline = np.zeros((MAGS_PER_FINGER, 3))
        self.calibration = calibration or {}
        self._hist: deque = deque(maxlen=max(2, history))

    def set_baseline(self, raw: np.ndarray) -> None:
        """Set the baseline from raw samples.

        `raw` is either (5, 3) for a single sample or (n, 5, 3) to average.
        Take it only after the sensor has settled: a baseline captured seconds
        after the cuboid was handled reads several times noisier than the true
        floor.
        """
        raw = np.asarray(raw, dtype=float)
        self.baseline = raw.mean(axis=0) if raw.ndim == 3 else raw.copy()
        self._hist.clear()

    def __call__(self, raw: np.ndarray) -> FingerSignals:
        dB = np.asarray(raw, dtype=float).reshape(MAGS_PER_FINGER, 3) - self.baseline

        per_chip = np.linalg.norm(dB, axis=1)
        force = float(np.linalg.norm(dB))
        normal = float(np.abs(dB[:, 2]).sum())
        shear = to_pad_frame(dB).sum(axis=0)
        contact = force > CONTACT_THRESHOLD_UT

        centroid = None
        if contact:
            w = np.clip(per_chip - CENTROID_FLOOR_UT, 0.0, None)
            if w.sum() > 0:
                centroid = (w[:, None] * CHIP_POSITIONS_MM).sum(axis=0) / w.sum()

        self._hist.append(dB.flatten())
        vibration = 0.0
        if len(self._hist) >= 2:
            diffs = np.diff(np.array(self._hist), axis=0)
            vibration = float(np.sqrt((diffs**2).mean()) / QUANT_UT)

        calibrated = {name: float(fn(dB)) for name, fn in self.calibration.items()}

        return FingerSignals(
            dB=dB,
            per_chip=per_chip,
            force=force,
            normal=normal,
            shear=shear,
            centroid=centroid,
            contact=contact,
            vibration=vibration,
            calibrated=calibrated,
        )


@dataclass
class GripperSignals:
    """Per-finger signals plus quantities that only exist across a pair."""

    fingers: Sequence[FingerSignals]

    @property
    def any_contact(self) -> bool:
        return any(f.contact for f in self.fingers)

    @property
    def grip_force(self) -> float:
        """Mean force proxy across fingers."""
        return float(np.mean([f.force for f in self.fingers]))

    @property
    def balance(self) -> Optional[float]:
        """Signed asymmetry between two fingers, -1 to +1, or None.

        Zero means both fingers are loaded equally. A large magnitude during a
        grasp usually means the object is off-centre or one finger is slipping.
        """
        if len(self.fingers) != 2:
            return None
        a, b = self.fingers[0].force, self.fingers[1].force
        total = a + b
        return float((a - b) / total) if total > 1e-6 else 0.0


class GripperProcessor:
    """Split a flat AnySkin reading into fingers and derive signals for each.

    The wire format concatenates fingers in ascending mux-channel order, five
    magnetometers each, so a two-finger rig arrives as 30 values ordered
    Bx, By, Bz per magnetometer.
    """

    def __init__(self, num_fingers: int = 2, **finger_kwargs):
        self.num_fingers = num_fingers
        self.fingers = [FingerProcessor(**finger_kwargs) for _ in range(num_fingers)]

    @property
    def num_values(self) -> int:
        return self.num_fingers * MAGS_PER_FINGER * 3

    def _split(self, flat: np.ndarray) -> np.ndarray:
        arr = np.asarray(flat, dtype=float).reshape(self.num_fingers, MAGS_PER_FINGER, 3)
        return arr

    def set_baseline(self, samples: np.ndarray) -> None:
        """Set every finger's baseline from (n, num_values) raw samples."""
        samples = np.atleast_2d(np.asarray(samples, dtype=float))
        mean = samples.mean(axis=0)
        per_finger = self._split(mean)
        for proc, base in zip(self.fingers, per_finger):
            proc.set_baseline(base)

    def __call__(self, flat: np.ndarray) -> GripperSignals:
        per_finger = self._split(flat)
        return GripperSignals(
            fingers=[proc(raw) for proc, raw in zip(self.fingers, per_finger)]
        )
