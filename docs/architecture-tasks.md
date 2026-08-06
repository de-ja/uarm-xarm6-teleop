# Architecture task list

This is the implementation tracker for the improvement roadmap in
[`architecture.md`](architecture.md). A task is complete only when code,
tests, generated artifacts, CI, and operator documentation agree.

## Active implementation

- [x] **A1 — Backend-owned frontend protocol types**
  - FastAPI response models are the source of truth.
  - TypeScript types are generated deterministically from their JSON schema.
  - CI fails when the committed generated file is stale.
- [x] **A2 — Explicit runtime capabilities**
  - Every telemetry snapshot reports leader transport, simulation, physical,
    camera, logging, video transport, and robot-count capabilities.
  - The UI disables unavailable modes using backend capabilities.
- [x] **A3 — Reusable sampled-loop scheduling**
  - Periodic deadline handling is outside `TeleopController`.
  - Leader monitoring and active control retain one-reader ownership and the
    existing overrun behavior.
- [x] **A4 — Structured session logging**
  - Optional JSON Lines logs include a session ID and controller events.
  - Control workers never perform file I/O.
  - Shutdown flushes and closes the log with bounded waiting.
- [x] **A5 — Video transport decision**
  - MJPEG remains the selected private-LAN transport.
  - Measurable WebRTC adoption criteria and non-goals are recorded.
- [x] **A6 — Robot process-isolation decision**
  - Runtime capabilities declare one robot per backend process.
  - Multi-robot expansion requires isolated controller instances and routing,
    never mutable global robot selection.
- [x] **A7 — Continuous enforcement**
  - Backend tests, frontend tests, type generation, lint, builds, and wheel
    packaging pass locally and in CI.

## Deferred only by explicit acceptance criteria

WebRTC implementation is not an unfinished task while MJPEG meets the current
private-LAN requirements. It becomes active when repeatable testing shows one
or more of the following:

- required camera traffic exceeds 70% of available link capacity;
- sustained capture-to-browser latency exceeds 150 ms at minimum JPEG quality;
- audio, NAT traversal, or internet-routed operation becomes a requirement;
- more than two simultaneous high-resolution camera streams are required.

Multi-robot operation is not an unfinished task while one backend owns one
xArm. It becomes active only with a concrete experiment requiring coordinated
robots and an isolation design that preserves independent watchdogs, state
machines, confirmations, and emergency-stop procedures.
