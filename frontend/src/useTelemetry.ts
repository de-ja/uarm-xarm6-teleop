import { useEffect, useRef, useState } from "react";
import { getStatus } from "./api";
import type { TeleopSnapshot } from "./types";

type ConnectionState = "connecting" | "live" | "offline";

export function useTelemetry() {
  const [snapshot, setSnapshot] = useState<TeleopSnapshot | null>(null);
  const [connection, setConnection] = useState<ConnectionState>("connecting");
  const [connectionError, setConnectionError] = useState<string | null>(null);
  const retryRef = useRef(0);

  useEffect(() => {
    let disposed = false;
    let socket: WebSocket | null = null;
    let retryTimer: number | null = null;

    getStatus()
      .then((value) => !disposed && setSnapshot(value))
      .catch((error: Error) => !disposed && setConnectionError(error.message));

    const connect = () => {
      if (disposed) return;
      setConnection("connecting");
      const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
      socket = new WebSocket(`${protocol}//${window.location.host}/ws/telemetry?frequency=10`);
      socket.onopen = () => {
        retryRef.current = 0;
        setConnection("live");
        setConnectionError(null);
      };
      socket.onmessage = (event) => {
        try {
          setSnapshot(JSON.parse(event.data) as TeleopSnapshot);
        } catch {
          setConnectionError("The backend sent an invalid telemetry frame");
        }
      };
      socket.onerror = () => setConnectionError("Telemetry connection failed");
      socket.onclose = () => {
        if (disposed) return;
        setConnection("offline");
        const delay = Math.min(1000 * 2 ** retryRef.current, 10_000);
        retryRef.current += 1;
        retryTimer = window.setTimeout(connect, delay);
      };
    };
    connect();

    return () => {
      disposed = true;
      if (retryTimer !== null) window.clearTimeout(retryTimer);
      socket?.close(1000, "operator console closed");
    };
  }, []);

  return { snapshot, setSnapshot, connection, connectionError };
}
