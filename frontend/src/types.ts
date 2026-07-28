export type TeleopState =
  | "idle"
  | "leader_ready"
  | "ready"
  | "starting"
  | "running"
  | "stopping"
  | "stopped"
  | "fault";

export interface ControllerEvent {
  timestamp: number;
  level: "info" | "warning" | "error";
  message: string;
}

export interface RobotStatus {
  connected: boolean;
  version: string;
  mode: number;
  state: number;
  error_code: number;
  warning_code: number;
  joint_degrees: number[];
  gripper_position: number;
  gripper_force: number;
  gripper_status: number | null;
  gripper_error_code: number;
}

export interface TeleopSnapshot {
  protocol_version: number;
  timestamp: number;
  state: TeleopState;
  mode: "dry_run" | "physical" | null;
  leader_connected: boolean;
  robot_connected: boolean;
  robot_ip: string;
  torque_enabled_ids: number[];
  leader_degrees: number[] | null;
  target_degrees: number[] | null;
  gripper_command: number | null;
  robot_status: RobotStatus | null;
  loop_rate_hz: number;
  last_sample_age_ms: number | null;
  fault: string | null;
  events: ControllerEvent[];
}
