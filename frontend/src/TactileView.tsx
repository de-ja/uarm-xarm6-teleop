// Gripper tactile view: a pad per finger plus the pair-level readouts.
//
// Mirrors the desktop visualiser: per finger a pad outline, a circle per chip
// sized by |dBz| and drawn hollow when negative, a shear vector per chip, a
// centroid crosshair, and strip charts. Across the pair, a grip force readout
// and a signed balance bar.
//
// Every geometric constant comes from the geometry frame. Nothing about chip
// positions, pad size or the contact threshold is written here.

import { useEffect, useRef } from "react";
import type { TactileFinger, TactileGeometry, TactileState } from "./tactile";

const PAD_VIEW = 120;
const CHART_POINTS = 120;
const CHART_WIDTH = 168;
const CHART_HEIGHT = 30;

/** Largest |dBz| a chip circle is scaled against, in microtesla. */
const DBZ_FULL_SCALE_UT = 900;
/** Microtesla of shear drawn as one pad-half of arrow length. */
const SHEAR_FULL_SCALE_UT = 600;

function Sparkline({ values, max, label }: { values: number[]; max: number; label: string }) {
  const scale = Math.max(max, 1e-6);
  const step = CHART_WIDTH / (CHART_POINTS - 1);
  const points = values
    .map((value, index) => {
      const x = index * step;
      const y = CHART_HEIGHT - Math.min(value / scale, 1) * CHART_HEIGHT;
      return `${x.toFixed(1)},${y.toFixed(1)}`;
    })
    .join(" ");
  return (
    <div className="tactile-chart">
      <span className="tactile-chart-label">{label}</span>
      <svg viewBox={`0 0 ${CHART_WIDTH} ${CHART_HEIGHT}`} preserveAspectRatio="none" aria-hidden>
        <polyline points={points} fill="none" stroke="currentColor" strokeWidth="1.5" />
      </svg>
    </div>
  );
}

function FingerPad({
  finger,
  geometry,
  name,
}: {
  finger: TactileFinger;
  geometry: TactileGeometry;
  name: string;
}) {
  // The pad frame is millimetres with the origin at the centre; map it onto a
  // square viewBox using the half-width the sensor reported.
  const half = geometry.pad_half_mm;
  const toView = (mm: number) => (mm / half) * (PAD_VIEW / 2);

  return (
    <svg
      className="tactile-pad"
      viewBox={`${-PAD_VIEW / 2} ${-PAD_VIEW / 2} ${PAD_VIEW} ${PAD_VIEW}`}
      role="img"
      aria-label={`${name} tactile pad`}
    >
      <rect
        x={-PAD_VIEW / 2}
        y={-PAD_VIEW / 2}
        width={PAD_VIEW}
        height={PAD_VIEW}
        rx="8"
        className={finger.contact ? "tactile-outline contact" : "tactile-outline"}
      />
      {geometry.chip_positions_mm.map((position, index) => {
        const dbz = finger.dbz[index] ?? 0;
        const cx = toView(position[0]);
        // Screen y grows downward while the pad frame grows upward.
        const cy = -toView(position[1]);
        const radius = 3 + Math.min(Math.abs(dbz) / DBZ_FULL_SCALE_UT, 1) * 18;
        const shear = finger.shear_xy[index] ?? [0, 0];
        const arrowScale = (PAD_VIEW / 2) / SHEAR_FULL_SCALE_UT;
        return (
          <g key={geometry.chip_names[index] ?? index}>
            <circle
              cx={cx}
              cy={cy}
              r={radius}
              // Hollow when negative, so the sign of dBz stays visible.
              className={dbz < 0 ? "tactile-chip negative" : "tactile-chip"}
            />
            <line
              x1={cx}
              y1={cy}
              x2={cx + shear[0] * arrowScale}
              y2={cy - shear[1] * arrowScale}
              className="tactile-shear"
            />
          </g>
        );
      })}
      {finger.centroid_mm !== null && (
        <g className="tactile-centroid">
          <line
            x1={toView(finger.centroid_mm[0]) - 7}
            y1={-toView(finger.centroid_mm[1])}
            x2={toView(finger.centroid_mm[0]) + 7}
            y2={-toView(finger.centroid_mm[1])}
          />
          <line
            x1={toView(finger.centroid_mm[0])}
            y1={-toView(finger.centroid_mm[1]) - 7}
            x2={toView(finger.centroid_mm[0])}
            y2={-toView(finger.centroid_mm[1]) + 7}
          />
        </g>
      )}
    </svg>
  );
}

export function TactileView({ state }: { state: TactileState }) {
  // One history per finger per signal, kept outside React state so a 60 Hz
  // frame does not allocate three arrays per render.
  const traces = useRef<Map<string, number[]>>(new Map());

  const frame = state.status === "live" ? state.frame : null;
  useEffect(() => {
    if (frame === null) return;
    frame.fingers.forEach((finger, index) => {
      for (const [signal, value] of [
        ["force", finger.force],
        ["shear", finger.shear_magnitude],
        ["vibration", finger.vibration],
      ] as const) {
        const key = `${index}:${signal}`;
        const series = traces.current.get(key) ?? [];
        series.push(value);
        if (series.length > CHART_POINTS) series.shift();
        traces.current.set(key, series);
      }
    });
  }, [frame]);

  if (state.status === "unavailable") {
    return <p className="muted">Tactile sensing unavailable: {state.reason}</p>;
  }
  if (state.status === "connecting" || frame === null) {
    return <p className="muted">Connecting to the tactile sensor…</p>;
  }

  const { geometry } = state;
  const balance = frame.balance;

  return (
    <div className="tactile">
      <div className="tactile-fingers">
        {frame.fingers.map((finger, index) => {
          const name = `Finger ${index + 1}`;
          return (
            <div className="tactile-finger" key={index}>
              <div className="tactile-finger-head">
                <span>{name}</span>
                <span className={finger.contact ? "tactile-flag contact" : "tactile-flag"}>
                  {finger.contact ? "contact" : "clear"}
                </span>
              </div>
              <FingerPad finger={finger} geometry={geometry} name={name} />
              <Sparkline
                values={traces.current.get(`${index}:force`) ?? []}
                max={geometry.contact_threshold_ut * 6}
                label="force"
              />
              <Sparkline
                values={traces.current.get(`${index}:shear`) ?? []}
                max={geometry.contact_threshold_ut * 3}
                label="shear"
              />
              {/* An activity measure that rises on any fast change, including a
                  deliberate press. Not a trained slip detector, so not labelled
                  as one. */}
              <Sparkline
                values={traces.current.get(`${index}:vibration`) ?? []}
                max={12}
                label="vibration"
              />
            </div>
          );
        })}
      </div>

      <div className="tactile-pair">
        <div className="tactile-grip">
          <span className="tactile-chart-label">grip force</span>
          <strong>{frame.grip_force.toFixed(0)}</strong>
          <span className="muted">µT, uncalibrated</span>
        </div>
        {balance === null ? (
          <p className="muted">Balance needs two fingers.</p>
        ) : (
          <div className="tactile-balance">
            <span className="tactile-chart-label">balance</span>
            <div className="tactile-balance-track">
              <div className="tactile-balance-centre" />
              <div
                className="tactile-balance-fill"
                style={{
                  left: balance < 0 ? `${50 + balance * 50}%` : "50%",
                  width: `${Math.min(Math.abs(balance), 1) * 50}%`,
                }}
              />
            </div>
            <span className="muted">{balance.toFixed(2)}</span>
          </div>
        )}
      </div>
    </div>
  );
}
