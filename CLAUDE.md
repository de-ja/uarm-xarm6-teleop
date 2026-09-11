# CLAUDE.md

Orientation for anyone — human or agent — working in this repository. It covers
what the system is, the invariants that must survive refactoring, and the
environment details that otherwise cost an hour to rediscover.

## What this is

A Feetech U-ARM acts as a **leader**; a UFACTORY xArm6 or a ManiSkill
simulation acts as the **follower**. An operator moves the leader by hand and
the follower tracks it. A FastAPI backend with a React console supervises the
run, streams camera video, and displays telemetry.

The leader is **read-only and unpowered**. It has no torque, so it falls under
gravity when released — which is a safety consideration, not a detail.

Two deployment layouts:

- **Single computer** — U-ARM and xArm on one machine.
- **Distributed** — laptop owns the U-ARM and serves samples; desktop owns the
  xArm, the cameras and the console. The desktop pulls one fresh sample per
  cycle, so a delayed sample can never accumulate in a queue.

## Map

```
src/uarm_xarm6_teleop/
  feetech.py        U-ARM serial reads, read-only
  remote_leader.py  the same samples over the network
  mapping.py        leader angles -> follower targets, pure
  backends/xarm.py  TargetSafety + XArm6Hardware, the only SDK caller
  backends/maniskill.py   simulated follower
  controller.py     TeleopController: lifecycle, workers, snapshots
  camera.py         V4L2 discovery and MJPEG capture sessions
  sensors.py        auxiliary observation sensors (see docs/sensors.md)
  serial_ports.py   usb: selectors, so devices are named by identity
  web/app.py        FastAPI: request validation, WebSockets, static console
  cli/              uarm-monitor, uarm-real, uarm-web, uarm-ports, ...
eflesh/             self-contained eFlesh tactile package (root, not src/)
frontend/           React console; src/types.ts is GENERATED
configs/            packaged defaults; local.toml is a gitignored overlay
docs/               architecture, sensors, wireless, ADRs
```

`TeleopController` is the **only** route from an operator request to a hardware
transition. FastAPI endpoints call controller operations; they never touch
`FeetechLeader` or `XArm6Hardware` directly. Preserve this.

## Invariants

Full list in [docs/architecture.md](docs/architecture.md). The ones most easily
broken by a well-meaning refactor:

1. **The U-ARM path is read-only.** Never write torque, IDs, EEPROM,
   calibration, or goal position.
2. **Arming is fail-closed.** Inspection never calls `motion_enable`,
   `set_mode`, or a motion command. Motion begins only after status, limit,
   alignment and typed-confirmation checks pass.
3. **Every physical sample is validated** — finite, six joints plus a gripper
   command, within static limits and the per-sample jump limit.
4. **Failures stop the run**, with one bounded exception for wireless leader
   *timeouts* (see ADR 0003). No command is ever issued for a sample that did
   not arrive.
5. **Sensors are the exception to (4).** They are observations, not inputs; a
   sensor failure must never fault a run. See [docs/sensors.md](docs/sensors.md).
6. **Secrets stay out of the repository** — robot IP, serial device and the
   wireless token live in `configs/local.toml`, which is gitignored.

## Control modes

`physical_xarm.mode` selects how targets reach the arm:

- **Mode 6** (default) — online trajectory planning. The controller plans an
  arrival profile per target under its acceleration limit. Safe, and smooths
  jerky input, but with targets a degree or two apart it never has room to
  accelerate, so it visibly trails the leader.
- **Mode 1** — servo streaming through `set_servo_angle_j`. No planning, near
  zero added latency. It also removes the planner's acceleration bound, so
  `TargetSafety.limit_acceleration` supplies one in software, and validation
  rejects a configuration whose `max_target_jump_degrees × rate` would exceed
  the arm's 180 °/s joint limit.

Controller-enforced ceilings worth knowing: **180 °/s** joint speed and
**20 rad/s² (1146 °/s²)** joint acceleration. Values above these are silently
clipped, so raising them in configuration does nothing.

## Gripper families

Two UFACTORY families take the same arguments in **different units**, selected
by `physical_xarm.gripper_kind`:

| | `g2` | `classic` |
| --- | --- | --- |
| position | mm, 0–84 | pulses, 0–850 |
| speed | mm/s | r/min |
| force | 1–100, controller-enforced | **none** |

They cannot be told apart by reading: `get_gripper_g2_position` calls
`get_gripper_position` and transforms the shared register, so both families
answer every read. Only a **write** distinguishes them — a classic gripper
rejects the G2 frame with SDK code 23.

On the classic family `gripper_force` is inert, so over-grip protection rests
entirely on the contact latch backed by a small `gripper_max_step`. That step is
applied **per cycle**, so it must be rescaled whenever `rate` changes.

## Validation gate

CI runs two jobs. Reproduce both before claiming work is done.

```bash
ruff format --check src tests scripts
ruff check src tests scripts
python scripts/generate_protocol.py --check   # easy to forget
pytest -q
```

```bash
cd frontend && npm ci && npm test && npm run check && npm run build
```

`frontend/src/types.ts` is **generated** from FastAPI's OpenAPI schemas. Editing
it by hand, or changing a Pydantic response model without regenerating, fails CI
with a drift error that is not obvious from the diff.

## Conventions

- Ruff enforces type annotations and Google-style docstrings on all production
  code. Tests are exempt.
- Comments explain *why*, especially where a value was measured or a failure
  mode is silent. Several constants here exist because of a specific empirical
  finding; say which.
- Configuration layers: packaged `configs/uarm_xarm6.toml`, overlaid by a
  machine-local file. Tables merge key-wise; `[[sensors]]` replaces outright.
- Prefer failing loudly over guessing. Several subsystems deliberately raise on
  ambiguity — device selection, frame width, gripper family — because the silent
  alternative is a hang or a wrong device.

## Hardware gotchas

- **`/dev/ttyACM*` numbering is not stable.** The leader and any USB CDC sensor
  compete for it. Use `usb:` selectors; run `uarm-ports` to find them.
- **An unpowered leader falls when released**, which trips the jump limiter.
  That is the limiter working.
- **Cameras need MJPG.** Set `CAP_PROP_FOURCC` before geometry, or the driver
  silently grants a much lower frame rate.
- **The arm must be posed to match the leader before arming.** Nothing moves it
  into place for you; that is deliberate.

## Testing without hardware

Most of the system runs with none attached. `FakeEFleshSource` and the
`eflesh_fake` driver kind provide a generated tactile feed; `uarm-sim` runs the
simulated follower; the backend suite uses fake SDK and leader objects
throughout. Prefer extending those over adding hardware-gated tests.
