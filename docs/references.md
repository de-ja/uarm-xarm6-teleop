# Operator-console references

The U-ARM operator console is implemented in this repository and does not use
Git submodules or embed either reference project. The projects below were
cloned temporarily for architectural study. No reference clone is required to
build or run U-ARM teleoperation.

## Hugging Face LeLab

- Repository: <https://github.com/huggingface/leLab>
- Reviewed commit: `308c7c30b2fd941811d2c643a6fd3ac4bf13938c`
- License: Apache-2.0
- Ideas adopted: one Python process serving a Vite build, FastAPI request
  schemas, REST lifecycle commands, and WebSocket joint telemetry.
- Ideas deliberately not adopted: module-global feature state, implicit
  mutual exclusion, SO-101/LeRobot coupling, and an unbounded telemetry queue.

## Pollen Robotics Reachy Mini Control

- Repository: <https://github.com/pollen-robotics/reachy-mini-desktop-app>
- Reviewed commit: `0f150976f4a44db0cc4c3e30247f4d71e1fff42c`
- License: Apache-2.0
- Ideas adopted: a single authoritative state model, a central live-state
  WebSocket, guarded UI capabilities derived from backend state, responsive
  connection cards, and a browser-first frontend that can be packaged later.
- Deferred ideas: Tauri packaging, WebRTC media, URDF visualization, discovery,
  updates, and multi-window synchronization.

## Local design boundary

FastAPI routes may request operations only through `TeleopController`. The
controller owns the leader, mapping, follower, worker thread, and state
transitions. The browser never calls `XArm6Hardware` directly, and serial or
xArm SDK operations never run in the web event loop.

Telemetry uses current-state snapshots rather than a growing event queue. When
the final telemetry client disconnects, the web layer requests a controller
stop. The existing robot-local command watchdog remains the final software
guard if the control worker stops producing commands.
