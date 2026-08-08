# ADR 0003: Absorb bounded transient loss on wireless links

- Status: Accepted
- Date: 2026-08-08

## Context

The follower host depends on two wireless links: the leader sample link to the
laptop and the operator console's telemetry link to the browser. Both were
first-strike fatal. One leader round trip slower than the deadline faulted an
active physical run, and losing the final telemetry client stopped motion with
no reconnect window. On a shared hotspot, ordinary jitter therefore ended runs
that were never unsafe.

The two settings were also silently coupled. `wireless.leader_timeout` was 0.2 s
against a `physical_xarm.watchdog_timeout` of 0.25 s, so a single timeout
consumed 80% of the watchdog budget. Raising the timeout past 0.25 s does not
buy tolerance; it only causes the robot-local watchdog to trip first, and a
tripped watchdog cannot be cleared without restarting teleoperation.

## Decision

Both links absorb a bounded amount of transient loss under one policy in
`[wireless]`.

Leader link: only `RemoteLeaderTimeout` is retryable. A timed-out sample skips
the cycle without commanding the follower, up to
`leader_max_consecutive_timeouts` in a row; the next miss faults as before.
Authentication, protocol, validation, and closed-connection failures remain
immediately fatal.

Console link: losing the final telemetry client schedules a stop after
`browser_grace_seconds` instead of stopping at once. A client that reconnects
inside the window cancels it. The pending stop runs as an independent task so
it survives cancellation of the disconnecting handler.

Configuration validation enforces the watchdog coupling. It rejects any pair
where `leader_timeout x (leader_max_consecutive_timeouts + 1)` is not below
`physical_xarm.watchdog_timeout`, so the controller always faults before the
robot-local guard trips. To make a useful gap fit, `watchdog_timeout` is widened
from 0.25 s to 1.0 s against a 0.75 s blind budget.

Recovery is rate limited rather than rejected. The leader keeps moving while the
follower holds still, so the first target after a gap normally exceeds
`max_target_jump_degrees`. After a tolerated gap the follower enters catch-up and
advances toward each fresh target by at most `catchup_step_degrees` per cycle,
the same slew idiom the G2 gripper already uses through `gripper_max_step`.
Catch-up ends automatically once the slew stops clipping. A divergence beyond
`catchup_max_divergence_degrees` is too far to chase and still faults.

Catch-up is engaged only after a recorded gap. On a healthy link a divergent
sample still faults on the jump limit, because there it indicates corruption or
a mapping error rather than lost time.

## Consequences

- Safety invariant 5 is narrowed, not removed: transport *failures* still end
  the run, but a bounded burst of transport *timeouts* no longer does.
- No command is issued for a sample that never arrived. The follower holds its
  last commanded target while blind, then slews back to the leader at a bounded
  rate, so a gap cannot produce a catch-up lunge.
- The robot-local watchdog is 4x weaker than before. This is the main cost of a
  larger gap. It is accepted because an uncommanded arm in servo mode 6 holds
  position, so the watchdog mainly covers a dead control process rather than
  runaway motion, and the operator retains the hardware emergency stop.
- Catch-up moves the follower through joint space toward the leader while the
  operator may still be moving. `catchup_step_degrees` is therefore set well
  below `max_target_jump_degrees` so recovery is visibly deliberate, and the
  session log records `slewing toward the leader` and `caught up` events that
  the console surfaces.
- Worst-case blind time is bounded by configuration and stays below the
  watchdog window, which remains the final software guard.
- Invariant 6 gains a bounded delay. Motion continues during the console grace
  window while the operator has no telemetry, so the window is kept short and
  `browser_grace_seconds = 0` restores the previous stop-immediately behavior.
  The hardware emergency stop remains authoritative throughout.
- Setting `leader_max_consecutive_timeouts = 0` restores first-strike faulting
  for the leader link.
