// The control toolkit's web part: the panel for its linear analysis (paperboat#370, control#14).
import type { ToolkitWebHost } from "@paperboat/host";
import { LinearPanel } from "./LinearPanel";
import "./linear.css";

export function register(host: ToolkitWebHost) {
  host.registerAnalysis("control/linear", LinearPanel);
}
