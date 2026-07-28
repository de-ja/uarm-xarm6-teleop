import type { TeleopSnapshot } from "./types";

export class ApiError extends Error {
  constructor(message: string, readonly status: number) {
    super(message);
  }
}

async function request(path: string, body?: unknown): Promise<TeleopSnapshot> {
  const response = await fetch(path, {
    method: body === undefined ? "POST" : "POST",
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

export const commands = {
  connectLeader: () => request("/api/leader/connect"),
  inspectRobot: (robotIp: string) => request("/api/robot/inspect", { robot_ip: robotIp }),
  startDryRun: () => request("/api/teleop/start", { mode: "dry_run" }),
  startPhysical: (confirmation: string) =>
    request("/api/teleop/start", { mode: "physical", confirmation }),
  stop: () => request("/api/teleop/stop"),
  disconnect: () => request("/api/session/disconnect"),
  resetFault: () => request("/api/fault/reset"),
};
