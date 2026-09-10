// Tactile sensor types and live-frame hook.
//
// Deliberately separate from types.ts, which is generated from the FastAPI
// OpenAPI schema. These payloads travel over a WebSocket rather than a typed
// response model, so they are declared by hand here and the generated file
// stays owned by the generator.
//
// No chip geometry, pad size or contact threshold is declared in this file.
// Those constants were verified by hand on hardware and arrive in the geometry
// frame; a second copy in the frontend would drift from the sensor.

import { useEffect, useRef, useState } from "react";

export interface TactileGeometry {
  type: "geometry";
  num_fingers: number;
  mags_per_finger: number;
  chip_names: string[];
  chip_positions_mm: number[][];
  pad_half_mm: number;
  contact_threshold_ut: number;
  quantisation_ut: number;
}

export interface TactileFinger {
  contact: boolean;
  force: number;
  normal: number;
  shear: number[];
  shear_magnitude: number;
  shear_angle: number;
  /** Activity measure that rises on any fast change. Not a slip detector. */
  vibration: number;
  centroid_mm: number[] | null;
  per_chip: number[];
  dbz: number[];
  /** Already rotated into the shared pad frame; needs no client-side matrices. */
  shear_xy: number[][];
  calibrated: Record<string, number>;
}

export interface TactileFrame {
  type: "frame";
  t: number;
  fingers: TactileFinger[];
  any_contact: boolean;
  grip_force: number;
  /** Null on a single-finger rig, where balance has no meaning. */
  balance: number | null;
}

export interface TactileUnavailable {
  type: "unavailable";
  reason: string;
}

type TactileMessage = TactileGeometry | TactileFrame | TactileUnavailable;

export type TactileState =
  | { status: "connecting" }
  | { status: "unavailable"; reason: string }
  | { status: "live"; geometry: TactileGeometry; frame: TactileFrame | null };

export function useTactile(frequency = 60) {
  const [state, setState] = useState<TactileState>({ status: "connecting" });
  const retryRef = useRef(0);

  useEffect(() => {
    let disposed = false;
    let socket: WebSocket | null = null;
    let retryTimer: number | null = null;
    let geometry: TactileGeometry | null = null;

    const connect = () => {
      if (disposed) return;
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      socket = new WebSocket(
        `${protocol}//${window.location.host}/ws/tactile?frequency=${frequency}`,
      );
      socket.onmessage = (event) => {
        let message: TactileMessage;
        try {
          message = JSON.parse(event.data) as TactileMessage;
        } catch {
          return;
        }
        if (message.type === "unavailable") {
          setState({ status: "unavailable", reason: message.reason });
          return;
        }
        if (message.type === "geometry") {
          geometry = message;
          setState({ status: "live", geometry: message, frame: null });
          return;
        }
        if (geometry !== null) {
          const known = geometry;
          setState({ status: "live", geometry: known, frame: message });
        }
      };
      socket.onclose = () => {
        if (disposed) return;
        geometry = null;
        setState({ status: "connecting" });
        const delay = Math.min(1000 * 2 ** retryRef.current, 10_000);
        retryRef.current += 1;
        retryTimer = window.setTimeout(connect, delay);
      };
      socket.onopen = () => {
        retryRef.current = 0;
      };
    };
    connect();

    return () => {
      disposed = true;
      if (retryTimer !== null) window.clearTimeout(retryTimer);
      socket?.close(1000, "operator console closed");
    };
  }, [frequency]);

  return state;
}
