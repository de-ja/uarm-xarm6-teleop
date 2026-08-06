# Architecture

This document describes the runtime boundaries of U-ARM xArm6 Teleoperation.
It is intended for maintainers changing control flow, networking, safety, or
the operator interface. Setup commands remain in the README and the wireless
deployment guide.

## Design goals

- Keep U-ARM access read-only.
- Make every transition into physical motion explicit and fail closed.
- Keep one authoritative controller state shared by CLI and web callers.
- Request fresh leader samples instead of buffering motion commands.
- Preserve robot-local command timeout protection when browsers or Wi-Fi fail.
- Allow simulation and dry-run validation without connecting to an xArm.

## Runtime components

| Component | Responsibility | Must not own |
| --- | --- | --- |
| `FeetechLeader` | Open the U-ARM serial bus and return complete calibrated samples | Mapping or follower commands |
| `RemoteLeaderService` | Authenticate one follower and expose request/response samples | xArm state or a sample queue |
| `RemoteLeader` | Pull and validate one fresh laptop sample | Serial configuration changes |
| `XArm6Mapping` | Convert six leader joints and trigger state to follower targets | Hardware connections |
| `TargetSafety` | Enforce finite values, static joint limits, and per-sample jump limits | SDK state transitions |
| `XArm6Hardware` | Inspect, arm, command, stop, and watch the physical follower | Browser or FastAPI state |
| `ManiSkillXArm6` | Render and step the visible simulated follower | Physical SDK access |
| `TeleopController` | Own resources, serialize transitions, run workers, and publish snapshots | HTTP or UI policy |
| `CameraManager` | Discover cameras and share low-latency capture sessions | Motion decisions |
| FastAPI application | Validate requests, supervise browser presence, and serve current state | Direct hardware commands |
| React application | Present capabilities and request guarded transitions | Authoritative safety state |

The controller is deliberately the only route from an operator request to a
hardware lifecycle transition. A FastAPI endpoint calls a public controller
operation; it does not call `FeetechLeader` or `XArm6Hardware` directly.

## Data flow

### Single-computer mode

```text
U-ARM USB
   |
   v
FeetechLeader -> LeaderSample -> XArm6Mapping -> TargetSafety
                                                    |
                         +--------------------------+------------------+
                         |                          |                  |
                      dry run                ManiSkillXArm6      XArm6Hardware
```

### Wireless two-computer mode

```text
Laptop                                             Desktop follower

U-ARM USB -> FeetechLeader                         operator browser
                   |                                      |
            uarm-leader :8765                             | HTTP/WS :8000
                   ^                                      v
                   +--- authenticated fresh request -- FastAPI
                                                          |
                                                  TeleopController
                                                          |
                                                mapping and safety
                                                          |
                                                  xArm SDK / simulator
```

Opening the desktop console from the laptop establishes an ordinary HTTP
connection. When the operator clicks **Connect leader**, the desktop derives
the laptop address from that request and opens the authenticated leader
WebSocket back to port 8765. The browser assists pairing but never carries
joint samples.

Each remote sample is request/response. The desktop sends a monotonically
increasing sequence number, the laptop reads the U-ARM only after receiving
that request, and the response must contain the same sequence and all seven
valid raw positions. There is no playback queue that can continue issuing old
motions after network delay.

## Controller state machine

```text
idle -- connect leader --> leader_ready -- inspect robot --> ready
                              |                              |
                              +--------- start -------------+
                                            |
                                         starting
                                            |
                                          running
                                            |
                                           stop
                                            |
                                          stopped

Any worker, transport, validation, or hardware failure -> fault
fault -- reset/disconnect --> idle
```

Dry run and simulation can start from `leader_ready`, `ready`, or `stopped`.
Physical mode additionally requires a read-only robot inspection, an exact
typed robot-IP confirmation, torque disabled on the leader, and a leader pose
within its configured startup tolerance.

Stopped physical sessions never resume automatically. Restarting repeats the
startup checks.

## Concurrency and ownership

`TeleopController` uses two lock roles:

- The operation lock serializes public lifecycle transitions such as connect,
  inspect, start, stop, and disconnect.
- The reentrant state lock protects snapshots, resources, samples, metrics,
  events, and worker state.

When teleoperation is not running, a leader-monitor thread refreshes displayed
joint pose without validating sample-to-sample target jumps. Before inspection
or start, the controller stops that monitor so only one thread reads the
leader. The active control worker then owns sampling until cleanup completes.

The physical backend has its own lock and watchdog thread. If the command gap
exceeds `physical_xarm.watchdog_timeout`, the watchdog requests xArm state 4
and marks motion unarmed. This protection does not depend on FastAPI, browser
telemetry, or Wi-Fi remaining available.

Camera capture is independent of the control loop. Subscribers to the same
camera share one capture thread, consume only the newest frame, and stop the
session when the final subscriber leaves.

Fixed-rate deadline and rate-measurement mechanics live in `scheduling.py`.
The scheduler does not know about leaders, robots, modes, or safety. This keeps
timing reusable while transition and arming policy remains centralized in
`TeleopController`.

When configured with `--event-log`, the controller sends versioned events to a
bounded queue. A dedicated writer thread appends owner-only JSON Lines records.
Control and monitor threads never wait for disk I/O; a saturated queue drops
records rather than delaying a physical command.

## Safety invariants

The following invariants must remain true during refactoring:

1. U-ARM code never writes torque, IDs, EEPROM, calibration, or goal position.
2. Robot inspection does not call `motion_enable`, `set_mode`, or a motion
   command.
3. Physical arming occurs only after controller status, gripper status, joint
   shape, configured limits, startup alignment, and controller limit checks.
4. Every physical sample is finite, contains exactly six joint targets plus
   one gripper command, and passes static and jump limits.
5. SDK errors, warnings, non-motion states, transport failures, and worker
   failures end the run and request a safe stop.
6. Loss of the final telemetry browser requests a software stop; the hardware
   emergency stop remains authoritative.
7. Authentication tokens remain outside the repository and owner-readable
   only. Plain `ws://` is used only on a trusted private network.

## Public API and documentation policy

Production Python public APIs use type hints and Google-style docstrings.
Docstrings describe contracts visible to callers: arguments, returned values,
raised errors, units, ownership, and safety effects. Inline comments are
reserved for constraints that cannot be made obvious through naming and types,
such as vendor SDK state semantics or hardware-specific workarounds.

Ruff enforces production function annotations and public module/API
documentation. Tests are excluded because test names already state behavior
and fake SDK methods intentionally mirror an external interface.

## Implemented architecture foundations

The current package boundaries are appropriate for a small research system:
hardware adapters, pure mapping, orchestration, transport, HTTP, and frontend
are separate. The first roadmap pass added these foundations:

1. FastAPI response schemas generate `frontend/src/types.ts`; CI rejects
   protocol drift.
2. Fixed-rate scheduling and rate measurement are reusable primitives while
   safety and transition policy stay in `TeleopController`.
3. Protocol version 3 includes session identity and explicit runtime
   capabilities for transport, optional backends, cameras, logging, video, and
   robot process limits.
4. Optional structured JSONL event logging uses an asynchronous bounded writer
   and owner-only files.
5. MJPEG and one-robot-per-process constraints are accepted, measurable
   architecture decisions rather than implicit assumptions.

Future structural changes should remain incremental. If feedback modes or
followers multiply, extract a transition-policy object before adding plugins;
do not distribute physical arming rules across backends. Experiment-grade
logging can add sampled telemetry records and run metadata later, but any disk
serialization must remain outside the sampled control loop.

Large framework adoption is not currently justified. The system benefits more
from preserving its explicit ownership and fail-safe behavior than from
introducing a generic robotics middleware into the physical command boundary.

Accepted architecture decisions:

- [ADR 0001: Keep MJPEG for private-LAN video](decisions/0001-video-transport.md)
- [ADR 0002: Keep one robot per backend process](decisions/0002-one-robot-per-process.md)
