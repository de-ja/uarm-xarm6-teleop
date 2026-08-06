# ADR 0002: Keep one robot per backend process

- Status: Accepted
- Date: 2026-08-06

## Context

`TeleopController` owns one leader, one mapping, one optional simulator, one
physical follower, and one worker lifecycle. Physical confirmation, watchdog
state, faults, and browser supervision all apply to that ownership unit.
Adding mutable robot selection inside the same controller would make failures
and confirmations ambiguous.

## Decision

One backend process owns at most one physical robot. Runtime capabilities
report `max_robots = 1`. Any future multi-robot system must run isolated
controller instances with explicit routing and independent watchdogs,
confirmations, session logs, and stop procedures.

## Consequences

- Current safety reasoning remains local to one controller lifecycle.
- A process crash or fault cannot silently transfer commands to another robot.
- Coordinated multi-robot experiments will require a separate supervisory
  layer; that layer may request operations but must not absorb the robot-local
  watchdog or arming rules.
