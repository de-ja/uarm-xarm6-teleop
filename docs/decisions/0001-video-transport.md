# ADR 0001: Keep MJPEG for private-LAN video

- Status: Accepted
- Date: 2026-08-06

## Context

The operator console needs low-complexity live video on a laptop-to-desktop
private router or hotspot. The current camera manager already provides newest
frame delivery, shared capture, browser latency feedback, and adaptive JPEG
quality. The deployment does not require audio, NAT traversal, or internet
routing.

## Decision

MJPEG over the existing FastAPI HTTP connection remains the only advertised
video transport. Runtime capabilities report `video_transport = "mjpeg"`.
WebRTC is not added until one of the measurable activation criteria in
`docs/architecture-tasks.md` is met.

## Consequences

- Camera traffic remains observable with ordinary HTTP tools.
- There is no signaling server, ICE configuration, TURN service, or browser
  peer lifecycle coupled to robot supervision.
- MJPEG uses more bandwidth than an inter-frame codec, so high-resolution or
  internet-routed deployments must repeat latency and capacity measurements.
- Choosing not to implement WebRTC under current requirements is a completed
  architecture decision, not an untracked feature gap.
