# Auxiliary sensors

Cameras and the leader are inputs the teleoperation loop depends on. A sensor is
not: it is an *observation*. That single distinction determines almost every
design decision in this subsystem, so it is worth stating first.

## Failure policy

**Losing the leader must fault the run. Losing a sensor must not.**

Control has no input without the leader, so `controller.py` faults on any
exception escaping the teleoperation worker. A tactile sensor failing should
cost an observation stream while teleoperation continues — stopping the arm
mid-grasp because a magnetometer stopped answering would be the worse outcome.

`SensorHub` is therefore total. Construction, `start`, `read_all` and `close`
all report failures rather than raising, and a source that fails once is dropped
for the remainder of the run instead of being retried on every cycle. Retrying a
wedged transport would spend latency in the loop that can least afford it.

A consequence worth internalising: a sensor built by copying the leader pattern
would inherit fail-closed behaviour that is actively wrong for it.

## Configuration

Sensors are an array of tables, so adding hardware is a configuration entry and
one driver class rather than a change to the controller.

```toml
[[sensors]]
kind = "eflesh"                  # or "eflesh_fake", a generated feed
name = "gripper_tactile"
port = "usb:serial=XXXXXXXXXXXX" # run uarm-ports to find this
settle = 5.0
```

An overlay that declares `[[sensors]]` replaces the list outright rather than
merging entry-wise, because removing a sensor locally is the common case when
the hardware is not attached.

See [device selection](#device-selection) for `port`. Two rules are enforced
rather than documented, because both fail silently otherwise:

- **A bare `/dev/ttyACM*` path is rejected.** The leader and a USB CDC sensor
  both enumerate as `ttyACM` and the numbering depends on boot order.
- **A zero `settle` is rejected.** A baseline captured before the sensor has
  settled reads several times noisier than the true floor, measured 9.7–16.5 µT
  immediately after handling against 2.4–3.2 µT settled. Nothing downstream can
  distinguish a bad baseline from a good one.

## Device selection

The leader and any USB CDC sensor compete for the same `/dev/ttyACM*`
numbering, which depends on boot order. Identify a device by what it is:

| selector | meaning |
| --- | --- |
| `usb:serial=ABC123` | one physical unit, survives reboots and replugging |
| `usb:vid=239a,pid=800f` | any board of that type, if only one is attached |
| `usb:product=QT Py` | substring match on the USB product string |

Criteria combine with AND. Plain paths and `/dev/serial/by-id` links pass
through untouched. Resolution happens when the device is opened rather than when
configuration is loaded, so a configuration stays valid while hardware is
detached.

**Resolution refuses to guess.** A selector matching several devices raises and
lists them; silently attaching to the wrong device would mean reading a tactile
sensor as servo positions. A selector matching nothing reports what *is*
attached.

```bash
uarm-ports    # lists attached devices with a selector for each,
              # then shows how every configured device resolves
```

## One reader per sensor

Each sensor is drained by a single background thread that publishes every sample
to a store. Consumers take snapshots and never read the sensor themselves.

This is not merely an optimisation. A sensor that derives signals statefully —
eFlesh keeps a rolling window for its vibration estimate — produces *different
values* depending on how often it is read, because the window is measured in
samples rather than in seconds. With two consumers polling directly, the same
physical motion measured 3.14 with a display attached and 53.03 without.

Consequences of the single-reader design:

- `read_latest()` returns the newest published sample and `None` until the first
  one arrives. It never blocks and never perturbs another consumer.
- Every sample is buffered for `drain()`, bounded so a consumer that stops
  draining loses the oldest rather than growing without limit. This is the
  full-rate capture path; **its sink does not exist yet**, so the only persisted
  record is still the periodic metrics entry.
- A source that returns instantly is paced to 250 Hz, so a generated feed cannot
  spin a core. Real hardware blocks on serial well below that.

## Live view

`GET /api/sensors` lists every configured sensor with its kind, whether it
started, its observed rate and the age of its newest sample. Sensors that failed
to build stay listed as not started: a console that cannot see a sensor cannot
explain why its panel is empty.

Each driver declares the `view` it can drive, so the console renders by
declaration rather than by inspecting for a method. A sensor with no view still
records and simply has nothing to draw.

`GET /ws/tactile?name=<sensor>&frequency=<hz>` sends the geometry once on
connect, then throttled frames. WebSocket rather than SSE because the
geometry-then-frames shape maps onto a connection lifecycle, and because the
repository already runs that pattern at `/ws/telemetry`. MJPEG over
`StreamingResponse` remains reserved for video by
[ADR 0001](decisions/0001-video-transport.md).

Two implementation constraints that are easy to get wrong:

- The payload generator blocks on serial and throttles by discarding samples, so
  it runs on a worker thread feeding a depth-one queue. On the event loop it
  would freeze every other request.
- The sender exits on a flag rather than being cancelled. At display rates the
  socket is nearly always mid-send, and cancelling there leaves the connection
  half-written.

**Staleness is detected in the browser.** A stalled producer leaves the socket
open and simply stops sending, so the backend cannot report its own silence.
Without that check a dead sensor renders as a calm, empty pad, which is worse
than a blank panel because it looks like an answer.

## Adding a driver

1. Implement `name`, `labels`, `start()`, `read_latest()`, `close()`.
2. Declare `view` if the console can draw it, and expose whatever payload that
   view needs.
3. Register the class in `_SENSOR_DRIVERS` under a new `kind`.
4. Extend configuration validation if the driver takes new fields.

The controller, the endpoints and the panel need no changes.

## eFlesh specifics

The [`eflesh`](../eflesh/README.md) package owns acquisition, framing,
baselining and signal derivation; the driver here is an adapter. See
[PROTOCOL.md](../PROTOCOL.md) for the wire format.

**The finger count is deliberately not configurable.** The mux firmware sizes
its frame at boot from however many boards enumerated, so a board that fails to
come up silently shrinks frames from 162 bytes to 82. A host that assumed the
larger width sits in its resync loop forever rather than raising — a hang, not
an error — so the count is measured from the stream.

**Geometry is never duplicated in the frontend.** Chip positions, pad size and
the contact threshold arrive in the geometry frame, and `shear_xy` is already
rotated into the shared pad frame. Those constants were verified by hand on
hardware; a second copy in JavaScript would drift.

### Known limits

- **`vibration` is not slip detection.** It is an activity measure that rises on
  any fast change, including a deliberate press. During teleoperation it
  correlates with arm motion rather than with slipping, because gripper motion
  changes the gap and therefore the field. Real detection needs spectral
  features, labelled windows, and the gripper state as a covariate.
- **A single baseline is not enough on a gripper.** Two cuboids on opposing
  fingers see and physically repel each other as a function of gripper gap, so a
  static baseline drifts into phantom contact as the gripper closes.
  `set_gap_baseline()` is a hook for the fix and is wired to nothing; it needs a
  sweep logged against the gripper encoder. Until then the contact indicator
  will cry wolf on a closing gripper.
- **Signals are uncalibrated proxies, not newtons.** They are monotone in the
  physical quantity, which is what operator feedback needs. `force` is a
  magnitude and survives a rebuilt cuboid; `shear` direction and the signed
  per-channel values are tied to one physical build.
