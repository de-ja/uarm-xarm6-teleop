import type { CameraInfo, TeleopSnapshot } from "./types";

export class ApiError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
  }
}

async function request(path: string, body?: unknown): Promise<TeleopSnapshot> {
  const response = await fetch(path, {
    method: "POST",
    headers: body === undefined ? undefined : { "content-type": "application/json" },
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  if (!response.ok) {
    const payload = (await response.json().catch(() => null)) as { detail?: string } | null;
    throw new ApiError(payload?.detail ?? `Request failed with HTTP ${response.status}`, response.status);
  }
  return response.json() as Promise<TeleopSnapshot>;
}

export async function getStatus(): Promise<TeleopSnapshot> {
  const response = await fetch("/api/status");
  if (!response.ok) throw new ApiError("Could not reach the operator backend", response.status);
  return response.json() as Promise<TeleopSnapshot>;
}

export async function getCameras(): Promise<CameraInfo[]> {
  const response = await fetch("/api/cameras");
  if (!response.ok) throw new ApiError("Could not load camera sources", response.status);
  return response.json() as Promise<CameraInfo[]>;
}

export async function reportCameraLatency(cameraId: string, latencyMs: number): Promise<void> {
  await fetch(`/api/cameras/${encodeURIComponent(cameraId)}/latency`, {
    method: "POST",
    headers: { "content-type": "application/json" },
    body: JSON.stringify({ latency_ms: latencyMs }),
  });
}

export interface ClockCalibration {
  offsetMs: number;
  roundTripMs: number;
}

export async function calibrateServerClock(samples = 3): Promise<ClockCalibration> {
  let best: ClockCalibration | null = null;
  for (let index = 0; index < samples; index += 1) {
    const startedAt = Date.now();
    const response = await fetch("/api/time", { cache: "no-store" });
    const receivedAt = Date.now();
    if (!response.ok) throw new ApiError("Could not calibrate follower clock", response.status);
    const payload = (await response.json()) as { timestamp: number };
    const roundTripMs = receivedAt - startedAt;
    const offsetMs = payload.timestamp * 1000 - (startedAt + receivedAt) / 2;
    if (best === null || roundTripMs < best.roundTripMs) best = { offsetMs, roundTripMs };
  }
  if (best === null) throw new ApiError("Could not calibrate follower clock", 503);
  return best;
}

export const commands = {
  connectLeader: () => request("/api/leader/connect"),
  inspectRobot: (robotIp: string) => request("/api/robot/inspect", { robot_ip: robotIp }),
  startDryRun: () => request("/api/teleop/start", { mode: "dry_run" }),
  startSimulation: () => request("/api/teleop/start", { mode: "simulation" }),
  startPhysical: (confirmation: string) =>
    request("/api/teleop/start", { mode: "physical", confirmation }),
  stop: () => request("/api/teleop/stop"),
  disconnect: () => request("/api/session/disconnect"),
  resetFault: () => request("/api/fault/reset"),
};
