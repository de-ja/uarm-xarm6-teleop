import type { TeleopSnapshot, TeleopState } from "./types";

export function isActiveState(state: TeleopState): boolean {
  return state === "starting" || state === "running" || state === "stopping";
}

export function getCapabilities(snapshot: TeleopSnapshot) {
  const active = isActiveState(snapshot.state);
  const healthy = snapshot.state !== "fault";
  const runtime = snapshot.capabilities;
  return {
    active,
    canConnectLeader: snapshot.state === "idle",
    canInspect: runtime.physical_available && snapshot.leader_connected && !active && healthy,
    canStartDryRun: snapshot.leader_connected && !active && healthy,
    canStartSimulation:
      runtime.simulation_available && snapshot.leader_connected && !active && healthy,
    canStartPhysical:
      runtime.physical_available &&
      snapshot.leader_connected &&
      snapshot.robot_connected &&
      !active &&
      healthy,
    canStop: active,
  };
}
