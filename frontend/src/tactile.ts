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
  | {
      status: "live";
      geometry: TactileGeometry;
      frame: TactileFrame | null;
      /** True when frames have stopped arriving. A stalled sensor keeps its
       *  last reading, which is indistinguishable from a live measurement of
       *  the same value, so the view must be able to say "unknown" instead. */
      stale: boolean;
    };

/** Frames older than this mean the stream has stopped, not that nothing is
 *  touching. Generous next to a 60 Hz stream so ordinary jitter never trips it. */
export const STALE_AFTER_MS = 500;

export function useTactile(name: string | null, frequency = 60) {
  const [state, setState] = useState<TactileState>({ status: "connecting" });
  const retryRef = useRef(0);
  const lastFrameRef = useRef(0);

  useEffect(() => {
    let disposed = false;
    let socket: WebSocket | null = null;
    let retryTimer: number | null = null;
    let geometry: TactileGeometry | null = null;

    const connect = () => {
      if (disposed) return;
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      socket = new WebSocket(
        `${protocol}//${window.location.host}/ws/tactile?frequency=${frequency}${name === null ? "" : `&name=${encodeURIComponent(name)}`}`,
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
          lastFrameRef.current = 0;
          setState({ status: "live", geometry: message, frame: null, stale: false });
          return;
        }
        if (geometry !== null) {
          const known = geometry;
          lastFrameRef.current = Date.now();
          setState({ status: "live", geometry: known, frame: message, stale: false });
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

    // The socket can stay open while the producer behind it dies, so silence is
    // detected here rather than reported by the backend.
    const staleTimer = window.setInterval(() => {
      const last = lastFrameRef.current;
      if (last === 0) return;
      if (Date.now() - last <= STALE_AFTER_MS) return;
      setState((current) =>
        current.status === "live" && !current.stale ? { ...current, stale: true } : current,
      );
    }, 200);

    return () => {
      window.clearInterval(staleTimer);
      disposed = true;
      if (retryTimer !== null) window.clearTimeout(retryTimer);
      socket?.close(1000, "operator console closed");
    };
  }, [frequency, name]);

  return state;
}
