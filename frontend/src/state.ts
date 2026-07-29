import type { TeleopSnapshot, TeleopState } from "./types";

export function isActiveState(state: TeleopState): boolean {
  return state === "starting" || state === "running" || state === "stopping";
}

export function getCapabilities(snapshot: TeleopSnapshot) {
  const active = isActiveState(snapshot.state);
  const healthy = snapshot.state !== "fault";
  return {
    active,
    canConnectLeader: snapshot.state === "idle",
    canInspect: snapshot.leader_connected && !active && healthy,
    canStartDryRun: snapshot.leader_connected && !active && healthy,
    canStartSimulation: snapshot.leader_connected && !active && healthy,
    canStartPhysical:
      snapshot.leader_connected && snapshot.robot_connected && !active && healthy,
    canStop: active,
  };
}
