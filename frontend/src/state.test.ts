import { describe, expect, it } from "vitest";
import { getCapabilities, isActiveState } from "./state";
import type { TeleopSnapshot } from "./types";

function snapshot(overrides: Partial<TeleopSnapshot>): TeleopSnapshot {
  return {
    protocol_version: 1,
    timestamp: 0,
    state: "idle",
    mode: null,
    leader_connected: false,
    robot_connected: false,
    robot_ip: "",
    torque_enabled_ids: [],
    leader_degrees: null,
    target_degrees: null,
    gripper_command: null,
    robot_status: null,
    loop_rate_hz: 0,
    last_sample_age_ms: null,
    fault: null,
    events: [],
    ...overrides,
  };
}

describe("operator capabilities", () => {
  it("treats transitional motion states as active", () => {
    expect(isActiveState("starting")).toBe(true);
    expect(isActiveState("running")).toBe(true);
    expect(isActiveState("stopping")).toBe(true);
    expect(isActiveState("stopped")).toBe(false);
  });

  it("requires both inspected robot and leader for physical start", () => {
    const leaderOnly = getCapabilities(
      snapshot({ state: "leader_ready", leader_connected: true }),
    );
    expect(leaderOnly.canStartDryRun).toBe(true);
    expect(leaderOnly.canStartPhysical).toBe(false);

    const ready = getCapabilities(
      snapshot({ state: "ready", leader_connected: true, robot_connected: true }),
    );
    expect(ready.canStartPhysical).toBe(true);
  });

  it("blocks starts while faulted or already active", () => {
    const faulted = getCapabilities(
      snapshot({ state: "fault", leader_connected: true, robot_connected: true }),
    );
    expect(faulted.canStartDryRun).toBe(false);
    expect(faulted.canStartPhysical).toBe(false);

    const running = getCapabilities(
      snapshot({ state: "running", leader_connected: true, robot_connected: true }),
    );
    expect(running.canStartPhysical).toBe(false);
    expect(running.canStop).toBe(true);
  });
});
