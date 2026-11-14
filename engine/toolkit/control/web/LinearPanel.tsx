// control/linear's panel: a summary strip (order, stability, margins, dc gain), then Bode,
// pole-zero or step in a black display. Plot2D (shared) draws, with hover readouts.
import { useMemo, useState } from "react";
import type { AnalysisPanelProps } from "@paperboat/host";
import { Plot2D, type PlotSeries } from "@paperboat/shared/plot";

type Result = {
  domain: "s" | "z";
  sample_time: number;
  order: number;
  inputs: string[];
  outputs: string[];
  input: number;
  output: number;
  num: number[];
  den: number[];
  poles: [number, number][];
  zeros: [number, number][];
  stable: boolean;
  stability: "stable" | "marginal" | "unstable";
  dc_gain: number | null;
  bode: { w: number[]; mag_db: number[]; phase_deg: number[] };
  margins: { gm_db: number | null; pm_deg: number | null; w_gc: number | null; w_pc: number | null };
  step: { t: number[]; y: number[]; info: { rise_time?: number | null; overshoot_pct?: number; settling_time?: number | null } };
};

const VIEWS = ["bode", "poles·zeros", "step"] as const;
type View = (typeof VIEWS)[number];

const num = (v: number | null | undefined, unit = "") => (v === null || v === undefined || !Number.isFinite(v) ? "–" : `${+v.toPrecision(3)}${unit}`);
const complex = ([re, im]: [number, number]) => (Math.abs(im) < 1e-9 ? `${+re.toPrecision(3)}` : `${+re.toPrecision(3)}${im < 0 ? "−" : "+"}${+Math.abs(im).toPrecision(3)}j`);
/** A CSS colour token's value for the canvas, which can't read var(). */
const token = (name: string, fallback: string) => getComputedStyle(document.documentElement).getPropertyValue(name).trim() || fallback;
/** The polynomial as text the matrix params take: "1 2 1". */
const coefficients = (p: number[]) => p.map((c) => +c.toPrecision(10)).join(" ");

export function LinearPanel({ result, error, options, setOptions }: AnalysisPanelProps) {
  const [view, setView] = useState<View>("bode");
  const r = result as Result | undefined;
  const colors = useMemo(() => ({ screen: token("--surface-screen", "#0d0d0d"), trace: token("--trace-1", "#ff6b1a"), second: token("--trace-2", "#05a5b3") }), []);
  if (error) return <p className="bad-text">{error}</p>;
  if (!r) return <p className="muted">Linearizing…</p>;
  const m = r.margins;
  const hz = r.domain === "z" ? ` · z, ${num(r.sample_time, " s")}` : "";
  const axes = (x: string, y: string, log = false) => ({ xlabel: { text: x }, ylabel: { text: y }, xscale: log ? ("log" as const) : ("linear" as const), grid: true, background: colors.screen });
  let plots: { series: PlotSeries[]; axes: ReturnType<typeof axes> }[];
  if (view === "bode") {
    plots = [
      { series: [{ x: r.bode.w, y: r.bode.mag_db, color: colors.trace, lineWidth: 1.5 }], axes: axes("ω (rad/s)", "dB", true) },
      { series: [{ x: r.bode.w, y: r.bode.phase_deg, color: colors.trace, lineWidth: 1.5 }], axes: axes("ω (rad/s)", "°", true) },
    ];
  } else if (view === "step") {
    plots = [{ series: [{ x: r.step.t, y: r.step.y, color: colors.trace, lineWidth: 1.5 }], axes: axes("t (s)", r.outputs[r.output] ?? "y") }];
  } else {
    plots = [];
  }
  return (
    <div className="linear-panel">
      {(r.inputs.length > 1 || r.outputs.length > 1) && (
        <div className="linear-pair">
          <select className="well" value={r.input} onChange={(e) => setOptions({ ...options, input: Number(e.target.value) })} aria-label="Input">
            {r.inputs.map((n, i) => (
              <option key={n} value={i}>{n}</option>
            ))}
          </select>
          <span className="muted">→</span>
          <select className="well" value={r.output} onChange={(e) => setOptions({ ...options, output: Number(e.target.value) })} aria-label="Output">
            {r.outputs.map((n, i) => (
              <option key={n} value={i}>{n}</option>
            ))}
          </select>
        </div>
      )}
      <dl className="linear-summary">
        <div>
          <dt>order</dt>
          <dd>
            {r.order} <span className={r.stability === "unstable" ? "bad-text" : ""}>{r.stability}</span>
            {hz}
          </dd>
        </div>
        <div>
          <dt>gain margin</dt>
          <dd>{num(m.gm_db, " dB")}{m.w_pc !== null && <span className="muted"> at {num(m.w_pc, " rad/s")}</span>}</dd>
        </div>
        <div>
          <dt>phase margin</dt>
          <dd>{num(m.pm_deg, "°")}{m.w_gc !== null && <span className="muted"> at {num(m.w_gc, " rad/s")}</span>}</dd>
        </div>
        <div>
          <dt>dc gain</dt>
          <dd>{r.dc_gain === null ? "∞" : num(r.dc_gain)}</dd>
        </div>
      </dl>
      <div className="choice mono linear-views" role="tablist" aria-label="Plot">
        {VIEWS.map((v) => (
          <button key={v} role="tab" aria-selected={view === v} className={view === v ? "on" : ""} onClick={() => setView(v)}>
            {view === v ? `[ ${v} ]` : `  ${v}  `}
          </button>
        ))}
      </div>
      {plots.map((p, i) => (
        <Plot2D key={`${view}-${i}`} className="linear-plot" series={p.series} axes={p.axes} />
      ))}
      {view === "poles·zeros" && <PoleZeroMap poles={r.poles} zeros={r.zeros} discrete={r.domain === "z"} trace={colors.trace} second={colors.second} />}
      {view === "step" && r.step.info.rise_time !== undefined && (
        <p className="muted small">
          rise {num(r.step.info.rise_time, " s")} · overshoot {num(r.step.info.overshoot_pct, " %")} · settling (2 %) {num(r.step.info.settling_time, " s")}
        </p>
      )}
      <dl className="linear-roots mono small">
        <dt>poles</dt>
        <dd>{r.poles.map(complex).join("  ") || "–"}</dd>
        <dt>zeros</dt>
        <dd>{r.zeros.map(complex).join("  ") || "–"}</dd>
      </dl>
      <div className="btns">
        <button className="key small" title="The transfer function as num and den text, for a transfer_fn block or a variable" onClick={() => void navigator.clipboard?.writeText(`num = "${coefficients(r.num)}"\nden = "${coefficients(r.den)}"`)}>
          Copy as tf
        </button>
      </div>
    </div>
  );
}

/**
 * The pole-zero map: × poles, ○ zeros (shape and colour both say which), the imaginary axis
 * dashed, and the unit circle for a discrete system. Square scale, so angles read true.
 */
function PoleZeroMap({ poles, zeros, discrete, trace, second }: { poles: [number, number][]; zeros: [number, number][]; discrete: boolean; trace: string; second: string }) {
  const all = [...poles, ...zeros, [0, 0] as [number, number], ...(discrete ? [[1, 1], [-1, -1]] as [number, number][] : [])];
  const span = Math.max(1e-6, ...all.map(([re, im]) => Math.max(Math.abs(re), Math.abs(im)))) * 1.25;
  const W = 400, H = 220, k = Math.min(W, H) / (2 * span);
  const x = (re: number) => W / 2 + re * k;
  const y = (im: number) => H / 2 - im * k;
  const label = ([re, im]: [number, number]) => (Math.abs(im) < 1e-9 ? `${+re.toPrecision(4)}` : `${+re.toPrecision(4)} ${im < 0 ? "−" : "+"} ${+Math.abs(im).toPrecision(4)}j`);
  return (
    <figure className="pz-map">
      <svg viewBox={`0 0 ${W} ${H}`} role="img" aria-label={`${poles.length} poles, ${zeros.length} zeros`}>
        <line className="pz-axis" x1={0} x2={W} y1={y(0)} y2={y(0)} />
        <line className="pz-axis dashed" x1={x(0)} x2={x(0)} y1={0} y2={H} />
        {discrete && <circle className="pz-axis dashed" cx={x(0)} cy={y(0)} r={k} fill="none" />}
        {zeros.map((z, i) => (
          <circle key={`z${i}`} cx={x(z[0])} cy={y(z[1])} r={5} fill="none" stroke={second} strokeWidth={2}>
            <title>zero {label(z)}</title>
          </circle>
        ))}
        {poles.map((p, i) => (
          <g key={`p${i}`} stroke={trace} strokeWidth={2} strokeLinecap="round">
            <title>pole {label(p)}</title>
            <line x1={x(p[0]) - 5} y1={y(p[1]) - 5} x2={x(p[0]) + 5} y2={y(p[1]) + 5} />
            <line x1={x(p[0]) - 5} y1={y(p[1]) + 5} x2={x(p[0]) + 5} y2={y(p[1]) - 5} />
            <rect x={x(p[0]) - 9} y={y(p[1]) - 9} width={18} height={18} fill="transparent" stroke="none" />
          </g>
        ))}
        <text className="pz-tick" x={W - 4} y={y(0) - 4} textAnchor="end">{`Re ±${+span.toPrecision(2)}`}</text>
      </svg>
      <figcaption>
        <span style={{ color: trace }}>×</span> poles <span style={{ color: second }}>○</span> zeros{discrete ? " · unit circle dashed" : " · imaginary axis dashed"}
      </figcaption>
    </figure>
  );
}
