import { useEffect, useMemo, useState } from "react";
import {
  Activity,
  AlertTriangle,
  Cable,
  Camera,
  Check,
  CircleStop,
  Gauge,
  Play,
  Power,
  Radio,
  RefreshCw,
  ShieldCheck,
  Unplug,
  Usb,
  VideoOff,
} from "lucide-react";
import { ApiError, cameraStreamUrl, commands, getCameras } from "./api";
import { getCapabilities } from "./state";
import type { CameraInfo, ControllerEvent, TeleopSnapshot } from "./types";
import { useTelemetry } from "./useTelemetry";

const JOINTS = ["Base", "Shoulder", "Elbow", "Forearm", "Wrist", "Tool"];

const stateLabels: Record<string, string> = {
  idle: "Disconnected",
  leader_ready: "Leader ready",
  ready: "Ready for teleoperation",
  starting: "Starting",
  running: "Teleoperation active",
  stopping: "Stopping",
  stopped: "Motion stopped",
  fault: "Fault",
};

function valueAt(values: number[] | null | undefined, index: number) {
  return values?.[index] === undefined ? "—" : `${values[index].toFixed(1)}°`;
}

function ago(timestamp: number) {
  return new Date(timestamp * 1000).toLocaleTimeString([], {
    hour: "2-digit",
    minute: "2-digit",
    second: "2-digit",
  });
}

function Metric({ label, value, good = true }: { label: string; value: string; good?: boolean }) {
  return (
    <div className="metric">
      <span>{label}</span>
      <strong className={good ? "" : "metric-warning"}>{value}</strong>
    </div>
  );
}

function EventLog({ events }: { events: ControllerEvent[] }) {
  return (
    <div className="event-list">
      {[...events].reverse().map((event, index) => (
        <div className={`event event-${event.level}`} key={`${event.timestamp}-${index}`}>
          <time>{ago(event.timestamp)}</time>
          <span>{event.message}</span>
        </div>
      ))}
    </div>
  );
}

function CameraFeed({ camera }: { camera: CameraInfo }) {
  const [status, setStatus] = useState<"connecting" | "live" | "offline">("connecting");

  return (
    <article className="camera-feed">
      <div className="camera-feed-heading">
        <div>
          <strong>{camera.name}</strong>
          <span>{camera.device}</span>
        </div>
        <span className={`camera-feed-status ${status}`}>{status}</span>
      </div>
      <div className="camera-frame">
        {status !== "live" && (
          <div className="camera-placeholder">
            {status === "offline" ? <VideoOff size={24} /> : <Camera size={24} />}
            <span>{status}</span>
          </div>
        )}
        <img
          src={cameraStreamUrl(camera.id)}
          alt={`${camera.name} live view`}
          onLoad={() => setStatus("live")}
          onError={() => setStatus("offline")}
        />
      </div>
    </article>
  );
}

function PhysicalStartDialog({
  snapshot,
  onCancel,
  onStart,
  busy,
}: {
  snapshot: TeleopSnapshot;
  onCancel: () => void;
  onStart: (confirmation: string) => void;
  busy: boolean;
}) {
  const [confirmation, setConfirmation] = useState("");
  const matches = confirmation.trim() === snapshot.robot_ip;
  return (
    <div className="modal-backdrop" role="presentation">
      <section className="modal" role="dialog" aria-modal="true" aria-labelledby="arm-title">
        <div className="modal-icon"><ShieldCheck size={24} /></div>
        <p className="eyebrow">Physical motion</p>
        <h2 id="arm-title">Final safety confirmation</h2>
        <p className="muted">
          The next action can enable xArm mode 6. Keep the workspace clear and hold the hardware
          emergency stop.
        </p>
        <ul className="checklist">
          <li><Check size={16} /> Leader starts in its calibrated CAD pose</li>
          <li><Check size={16} /> xArm pose matches the displayed target</li>
          <li><Check size={16} /> Controller errors and warnings are clear</li>
        </ul>
        <label className="field-label" htmlFor="confirmation">
          Type <code>{snapshot.robot_ip}</code> to enable motion
        </label>
        <input
          id="confirmation"
          autoFocus
          value={confirmation}
          onChange={(event) => setConfirmation(event.target.value)}
          placeholder={snapshot.robot_ip}
        />
        <div className="modal-actions">
          <button className="button secondary" onClick={onCancel}>Cancel</button>
          <button className="button danger" disabled={!matches || busy} onClick={() => onStart(confirmation)}>
            <Play size={16} /> Start physical motion
          </button>
        </div>
      </section>
    </div>
  );
}

export function App() {
  const { snapshot, setSnapshot, connection, connectionError } = useTelemetry();
  const [robotIp, setRobotIp] = useState("");
  const [busyAction, setBusyAction] = useState<string | null>(null);
  const [actionError, setActionError] = useState<string | null>(null);
  const [showPhysicalStart, setShowPhysicalStart] = useState(false);
  const [cameras, setCameras] = useState<CameraInfo[]>([]);
  const [selectedCameraIds, setSelectedCameraIds] = useState<string[]>([]);
  const [cameraError, setCameraError] = useState<string | null>(null);
  const [camerasLoading, setCamerasLoading] = useState(true);

  const capabilities = snapshot === null ? null : getCapabilities(snapshot);
  const active = capabilities?.active ?? false;
  const canConnectLeader = capabilities?.canConnectLeader ?? false;
  const canInspect = capabilities?.canInspect ?? false;
  const canStart = capabilities?.canStartDryRun ?? false;
  const canStartPhysical = capabilities?.canStartPhysical ?? false;
  const actualJoints = snapshot?.robot_status?.joint_degrees;
  const selectedCameras = cameras.filter((camera) => selectedCameraIds.includes(camera.id));

  async function refreshCameras() {
    setCamerasLoading(true);
    setCameraError(null);
    try {
      const available = await getCameras();
      const availableIds = new Set(available.map((camera) => camera.id));
      setCameras(available);
      setSelectedCameraIds((current) => current.filter((id) => availableIds.has(id)));
    } catch (error) {
      setCameraError(error instanceof ApiError ? error.message : "Could not load camera sources");
    } finally {
      setCamerasLoading(false);
    }
  }

  useEffect(() => {
    void refreshCameras();
  }, []);

  const statusTone = useMemo(() => {
    if (snapshot?.state === "fault") return "fault";
    if (snapshot?.state === "running") return "active";
    if (snapshot?.state === "ready" || snapshot?.state === "leader_ready") return "ready";
    return "neutral";
  }, [snapshot?.state]);

  async function run(label: string, operation: () => Promise<TeleopSnapshot>) {
    setBusyAction(label);
    setActionError(null);
    try {
      setSnapshot(await operation());
    } catch (error) {
      setActionError(error instanceof ApiError ? error.message : "The command failed unexpectedly");
    } finally {
      setBusyAction(null);
    }
  }

  function toggleCamera(cameraId: string) {
    setSelectedCameraIds((current) =>
      current.includes(cameraId)
        ? current.filter((id) => id !== cameraId)
        : [...current, cameraId],
    );
  }

  if (!snapshot) {
    return (
      <main className="loading-screen">
        <div className="brand-mark"><Activity /></div>
        <h1>U-ARM Operator</h1>
        <p>{connectionError ?? "Connecting to the teleoperation backend…"}</p>
      </main>
    );
  }

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand">
          <div className="brand-mark"><Activity /></div>
          <div><strong>U-ARM</strong><span>xArm6 operator console</span></div>
        </div>
        <div className="topbar-status">
          <span className={`connection-pill ${connection}`}><Radio size={14} /> {connection}</span>
          <span className={`state-pill ${statusTone}`}>{stateLabels[snapshot.state]}</span>
          <button
            className="button stop-button"
            disabled={!active || busyAction !== null}
            onClick={() => run("stop", commands.stop)}
          >
            <CircleStop size={18} /> Stop motion
          </button>
        </div>
      </header>

      {(actionError || cameraError || connectionError || snapshot.fault) && (
        <div className="alert-banner">
          <AlertTriangle size={18} />
          <span>{actionError ?? cameraError ?? snapshot.fault ?? connectionError}</span>
        </div>
      )}

      <section className="workspace">
        <aside className="control-column">
          <section className="panel connection-panel">
            <div className="panel-heading"><div><p className="eyebrow">Step 1</p><h2>Leader</h2></div><Usb /></div>
            <p className="muted">Read-only Feetech U-ARM connection.</p>
            <div className="connection-state">
              <span className={`status-dot ${snapshot.leader_connected ? "online" : ""}`} />
              {snapshot.leader_connected ? "Connected" : "Not connected"}
            </div>
            {snapshot.torque_enabled_ids.length > 0 && (
              <p className="inline-warning">Torque enabled on IDs {snapshot.torque_enabled_ids.join(", ")}</p>
            )}
            <button
              className="button primary full"
              disabled={!canConnectLeader || busyAction !== null}
              onClick={() => run("connect", commands.connectLeader)}
            >
              <Cable size={16} /> Connect leader
            </button>
          </section>

          <section className="panel connection-panel">
            <div className="panel-heading"><div><p className="eyebrow">Step 2</p><h2>Follower</h2></div><Power /></div>
            <p className="muted">Inspection is read-only and never enables motion.</p>
            <label className="field-label" htmlFor="robot-ip">xArm controller IP</label>
            <input
              id="robot-ip"
              value={robotIp}
              onChange={(event) => setRobotIp(event.target.value)}
              placeholder="192.168.1.XXX"
              disabled={active}
            />
            <button
              className="button secondary full"
              disabled={!canInspect || !robotIp.trim() || busyAction !== null}
              onClick={() => run("inspect", () => commands.inspectRobot(robotIp))}
            >
              <ShieldCheck size={16} /> Inspect robot
            </button>
            {snapshot.robot_connected && (
              <div className="robot-summary">
                <strong>{snapshot.robot_ip}</strong>
                <span>SDK {snapshot.robot_status?.version ?? "unknown"}</span>
              </div>
            )}
          </section>

          <section className="panel run-panel">
            <div className="panel-heading"><div><p className="eyebrow">Step 3</p><h2>Run</h2></div><Play /></div>
            <button
              className="button secondary full"
              disabled={!canStart || busyAction !== null}
              onClick={() => run("dry-run", commands.startDryRun)}
            >
              <Gauge size={16} /> Start dry run
            </button>
            <button
              className="button danger-outline full"
              disabled={!canStartPhysical || busyAction !== null}
              onClick={() => setShowPhysicalStart(true)}
            >
              <Play size={16} /> Start physical
            </button>
            {snapshot.state === "fault" ? (
              <button className="button secondary full" onClick={() => run("reset", commands.resetFault)}>
                <RefreshCw size={16} /> Reset and disconnect
              </button>
            ) : (
              <button
                className="button ghost full"
                disabled={active || !snapshot.leader_connected || busyAction !== null}
                onClick={() => run("disconnect", commands.disconnect)}
              >
                <Unplug size={16} /> Disconnect all
              </button>
            )}
          </section>
        </aside>

        <section className="telemetry-column">
          <section className="panel camera-panel">
            <div className="section-title camera-title">
              <div><p className="eyebrow">Live vision</p><h2>Cameras</h2></div>
              <div className="camera-title-actions">
                <span>{selectedCameras.length} / {cameras.length} active</span>
                <button
                  className="icon-button"
                  type="button"
                  title="Refresh cameras"
                  aria-label="Refresh cameras"
                  disabled={camerasLoading}
                  onClick={() => void refreshCameras()}
                >
                  <RefreshCw size={16} className={camerasLoading ? "spin" : ""} />
                </button>
              </div>
            </div>

            <div className="camera-picker" aria-label="Available cameras">
              {cameras.map((camera) => (
                <label className="camera-option" key={camera.id}>
                  <input
                    type="checkbox"
                    checked={selectedCameraIds.includes(camera.id)}
                    onChange={() => toggleCamera(camera.id)}
                  />
                  <Camera size={16} />
                  <span><strong>{camera.name}</strong><small>{camera.device}</small></span>
                </label>
              ))}
              {!camerasLoading && cameras.length === 0 && (
                <div className="camera-empty"><VideoOff size={18} /> No cameras available</div>
              )}
            </div>

            {selectedCameras.length > 0 && (
              <div className="camera-grid">
                {selectedCameras.map((camera) => <CameraFeed camera={camera} key={camera.id} />)}
              </div>
            )}
          </section>

          <section className="panel pose-panel">
            <div className="section-title">
              <div><p className="eyebrow">Live telemetry</p><h2>Joint pose</h2></div>
              <span className="sample-age">{snapshot.last_sample_age_ms === null ? "No sample" : `${snapshot.last_sample_age_ms.toFixed(0)} ms old`}</span>
            </div>
            <div className="joint-table">
              <div className="joint-row joint-header"><span>Joint</span><span>Leader</span><span>Target</span><span>Follower</span></div>
              {JOINTS.map((joint, index) => (
                <div className="joint-row" key={joint}>
                  <strong><span className="joint-index">J{index + 1}</span>{joint}</strong>
                  <span>{valueAt(snapshot.leader_degrees, index)}</span>
                  <span>{valueAt(snapshot.target_degrees, index)}</span>
                  <span>{valueAt(actualJoints, index)}</span>
                </div>
              ))}
            </div>
          </section>

          <div className="metrics-grid">
            <section className="panel">
              <div className="section-title compact"><h2>Runtime</h2><Activity size={18} /></div>
              <Metric label="Control mode" value={snapshot.mode?.replace("_", " ") ?? "disabled"} />
              <Metric label="Loop rate" value={`${snapshot.loop_rate_hz.toFixed(1)} Hz`} good={!active || snapshot.loop_rate_hz > 15} />
              <Metric label="xArm mode / state" value={snapshot.robot_status ? `${snapshot.robot_status.mode} / ${snapshot.robot_status.state}` : "—"} />
              <Metric label="Error / warning" value={snapshot.robot_status ? `${snapshot.robot_status.error_code} / ${snapshot.robot_status.warning_code}` : "—"} good={!snapshot.robot_status || (!snapshot.robot_status.error_code && !snapshot.robot_status.warning_code)} />
            </section>
            <section className="panel">
              <div className="section-title compact"><h2>Gripper G2</h2><Gauge size={18} /></div>
              <Metric label="Command" value={snapshot.gripper_command === null ? "—" : snapshot.gripper_command.toFixed(3)} />
              <Metric label="Opening" value={snapshot.robot_status ? `${snapshot.robot_status.gripper_position} mm` : "—"} />
              <Metric label="Force limit" value={snapshot.robot_status ? `${snapshot.robot_status.gripper_force} / 100` : "—"} />
              <Metric label="Gripper error" value={snapshot.robot_status ? String(snapshot.robot_status.gripper_error_code) : "—"} good={!snapshot.robot_status?.gripper_error_code} />
            </section>
          </div>

          <section className="panel log-panel">
            <div className="section-title compact"><h2>Session events</h2><span>{snapshot.events.length} entries</span></div>
            <EventLog events={snapshot.events} />
          </section>
        </section>
      </section>

      <footer>
        <span>Software stop requests xArm state 4. It is not a hardware emergency stop.</span>
        <span>Protocol v{snapshot.protocol_version}</span>
      </footer>

      {showPhysicalStart && (
        <PhysicalStartDialog
          snapshot={snapshot}
          busy={busyAction !== null}
          onCancel={() => setShowPhysicalStart(false)}
          onStart={(confirmation) => {
            setShowPhysicalStart(false);
            void run("physical-start", () => commands.startPhysical(confirmation));
          }}
        />
      )}
    </main>
  );
}
