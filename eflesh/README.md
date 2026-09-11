# `eflesh`

Acquisition and derived tactile signals for the eFlesh magnetic tactile sensor,
packaged for use as one sensor driver inside a larger teleop stack.

Self-contained: copy this directory plus [`../PROTOCOL.md`](../PROTOCOL.md).
Requires `numpy`, `pyserial`, and `anyskin`. The `anyskin` import is lazy, so
the signal layer and the fake source work without hardware or that dependency.

```
signals.py   pure numpy -- baselines and derived signals, no I/O
stream.py    serial acquisition, shaped as a pollable sensor source
web.py       JSON payloads for a browser frontend
```

## Quick start

```python
from eflesh import EFleshSource

src = EFleshSource(port="/dev/ttyACM0")   # 1 or 2 fingers, detected
src.start()

r = src.read_latest()
r.signals.fingers[0].force        # contact intensity, microtesla
r.signals.fingers[0].centroid     # contact location in mm, or None
r.signals.balance                 # -1..+1 across the pair

src.close()
```

`EFleshSource` implements `name`, `labels`, `start()`, `read_latest()` and
`close()`, so it drops into a driver registry directly:

```toml
[[sensors]]
kind = "eflesh"
name = "gripper_tactile"
port = "/dev/ttyACM0"
settle = 5.0          # see "Baselining" below -- do not set this to 0
```

`FakeEFleshSource(num_fingers=2)` has the same interface with synthetic data and
no serial port, for testing the registry and the frontend without hardware.

## What the signals mean

Per finger, from `r.signals.fingers[i]`:

| field | meaning |
|---|---|
| `force` | `norm(dB)` over all 15 channels. Overall contact intensity. |
| `normal` | sum of `abs(dBz)`. Proxy for normal force. |
| `shear` | resultant in-plane vector in the pad frame. Direction is meaningful. |
| `centroid` | magnitude-weighted contact location, millimetres, or `None`. |
| `contact` | `force` above `CONTACT_THRESHOLD_UT`. |
| `vibration` | RMS of successive differences, in quantisation steps. |
| `per_chip` | `norm(dB)` per chip, in stream order. |

Across the pair, from `r.signals`: `grip_force`, `balance` (signed asymmetry,
zero when both fingers are loaded equally), `any_contact`.

**These are uncalibrated proxies, not newtons.** They are monotone in the
physical quantity, which is what operator feedback needs. See "Calibration"
for physical units.

**`force` is polarity-invariant** because it is a magnitude. It survives a
rebuilt cuboid with different magnet polarity. `shear` direction and the signed
per-channel values do not -- they are tied to one physical build.

## Baselining

Everything is computed against a baseline captured at `start()` and refreshed by
`rebaseline()`. The sensor must be untouched and **settled** for both.

A baseline taken seconds after the cuboid has been handled reads several times
noisier than the true floor -- measured 9.7-16.5 uT immediately after mounting
against 2.4-3.2 uT once settled. Thermal and mechanical settling, not noise.
Hence `settle=5.0` by default; do not set it to zero on real hardware.

The true floor is about one quantisation step (2.4 uT on XY), so the sensor is
quantisation-limited rather than physics-limited.

### On a gripper, one baseline is not enough

Two cuboids on opposing fingers see each other, and with same-polarity builds
they also physically repel, deforming the elastomer and producing a phantom
contact reading that grows as the gripper closes. Both effects are deterministic
functions of the gripper gap.

The fix is a **gap-conditioned baseline**: sweep the gripper through its range
with nothing between the fingers, log every channel against the encoder value,
and subtract the interpolated baseline for the current gap. Not implemented
here, because it needs your gripper's encoder; hook it in by calling
`FingerProcessor.set_baseline()` with the interpolated vector as the gap changes.

## Web frontend

`web.py` produces plain dicts ready for `json.dumps`. Transport is yours.

Send the geometry **once** when a client connects, then a frame per tick:

```python
from eflesh.web import geometry_payload, iter_payloads

await ws.send_json(geometry_payload(src.num_fingers))
for frame in iter_payloads(src, hz=60):
    await ws.send_json(frame)
```

`geometry_payload()` carries chip names and positions, pad size, and the contact
threshold, so the frontend draws the pad without hardcoding constants. Those
values were verified by hand on hardware; a second copy in JS will drift.

A frame is about 1.5 KB of JSON, or 2.2 KB with `include_raw=True`.

**Throttle for display.** The sensor streams at ~194 Hz with two boards, more
than a browser can paint. `iter_payloads(hz=60)` drops stale samples and always
yields current data. Keep the full-rate stream in the backend for recording --
display rate and record rate are different problems.

`shear_xy` in the payload is already rotated into the shared pad frame, so the
frontend never needs the per-chip rotation matrices.

[`../visualizer/viz_gripper.py`](../visualizer/viz_gripper.py) is a desktop
implementation of the same view. It is a working spec for the web version: pad
outline, per-chip circles sized by `abs(dBz)` and hollow when negative, shear
vectors, a centroid crosshair, and strip charts for force, shear and vibration.

## Calibration

`characterization/` holds probe datasets for the three quantities the eFlesh
paper characterises, and `characterization/train.py` fits an MLP from baselined
dB to any one of them:

| dataset | ground truth |
|---|---|
| `normal_force` | probe position + force in newtons |
| `shear_force` | probe position + force in newtons, 10 magnetometers |
| `spatial_resolution` | probe position only |

**No checkpoints ship, and a model fitted on another build will not transfer.**
Channel order, magnet polarity and magnet seating all change the mapping. To get
newtons you need probe data from your own rig.

Once you have a model, attach it and its output appears alongside the proxies:

```python
src = EFleshSource(port="/dev/ttyACM0",
                   calibration={"normal_N": my_mlp_wrapper})
r.signals.fingers[0].calibrated["normal_N"]
```

A calibration callable takes the `(5, 3)` dB array and returns a scalar.

## Gotchas

**Frame width is set by the firmware at boot** from however many boards
enumerated. If one fails to come up you get 82-byte frames instead of 162, and a
host expecting the larger size sits in its resync loop forever rather than
raising -- a hang, not an error. `detect_num_mags()` measures the width instead;
leave `num_fingers=None` so it runs.

**`vibration` is not a slip detector.** It is an activity measure that rises on
any fast change, including a deliberate press. Real slip detection needs a
classifier trained on labelled windows -- see `slip_detection/`.

**The centroid is coarse.** Five chips interpolating over a 40 mm pad. It tracks
a single contact well; it cannot resolve two simultaneous contacts.

**Channel order is physical, not by address**: middle, left, right, top, bottom,
then the next finger in ascending mux-channel order. This assumes the
`10X_eflesh_mux_stream` firmware. Pin that in your deployment notes.
