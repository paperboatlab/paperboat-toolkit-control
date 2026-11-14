#pragma once

// The control blocks' compact plates, in one table: a glyph or a drawn view.icon (the 0..100,
// 5-grid primitives of paperboat-shared icon.ts) and the caption's value. The registration
// helpers (add_designed, add_signal, add_without_feedthrough) and pid apply it, so a block
// gets its plate by its type alone. Blocks not listed keep the full card.
#include <string_view>

#include <nlohmann/json.hpp>

#include "paperboat/toolkit/registry.hpp"

namespace paperboat::toolkit::control {

inline View compact_view(std::string_view type) {
  struct Row { std::string_view type, glyph, value, icon; };
  static constexpr Row rows[] = {
      {"integrator", "∫", "x₀ {initial}", R"()"},
      {"derivative", "", "τ {tau}", R"({"w":1,"h":1,"g":[{"t":"text","x":50,"y":30,"s":"d","size":15,"font":"mono"},{"t":"line","p":[[30,50],[70,50]]},{"t":"text","x":50,"y":70,"s":"dt","size":15,"font":"mono"}]})"},
      {"lag", "", "τ {tau}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,80],[90,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,15],[10,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,25],[90,25]],"stroke":"muted","dash":"dash"},{"t":"line","p":[[10,80],[15,70],[20,60],[25,50],[30,45],[35,40],[40,35],[45,35],[50,30],[55,30],[60,30],[65,30],[70,30],[75,25],[80,25],[85,25],[90,25]]}]})"},
      {"second_order", "", "ζ {zeta}", R"({"w":1,"h":1,"g":[{"t":"text","x":50,"y":50,"s":"ωₙ ζ","size":15,"font":"mono"}]})"},
      {"transfer_fn", "", "", R"({"w":2,"h":1,"g":[{"t":"text","x":50,"y":30,"s":"{num:poly}","size":13,"font":"mono"},{"t":"line","p":[[10,50],[90,50]]},{"t":"text","x":50,"y":70,"s":"{den:poly}","size":13,"font":"mono"}]})"},
      {"pid", "", "{kp} · {ki} · {kd}", R"({"w":1,"h":1,"g":[{"t":"text","x":50,"y":50,"s":"PID","size":15,"font":"mono"}]})"},
      {"delay", "", "{steps} steps", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,75],[90,75]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[15,75],[15,40],[35,40],[35,75]],"stroke":"muted","dash":"dash"},{"t":"line","p":[[50,75],[50,40],[70,40],[70,75]]},{"t":"line","p":[[25,25],[55,25]],"weight":"thin"},{"t":"line","p":[[50,20],[55,25],[50,30]],"weight":"thin"}]})"},
      {"washout", "", "τ {tau}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,80],[90,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,15],[10,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,80],[20,80]]},{"t":"line","p":[[20,20],[25,40],[30,55],[35,65],[40,70],[45,75],[50,75],[55,75],[60,80],[65,80],[70,80],[75,80],[80,80],[85,80],[90,80]]}]})"},
      {"notch", "", "{freq}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,80],[90,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,30],[40,30],[50,75],[60,30],[90,30]]}]})"},
      {"rate_limiter", "", "{rise} / {fall}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,80],[90,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,15],[10,80]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,75],[30,75],[30,25],[90,25]],"stroke":"muted","dash":"dash"},{"t":"line","p":[[10,75],[30,75],[65,25],[90,25]]}]})"},
      {"quantizer", "", "q {interval}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,80],[90,20]],"stroke":"muted","dash":"dash"},{"t":"line","p":[[10,80],[25,80],[25,65],[40,65],[40,50],[55,50],[55,35],[70,35],[70,20],[90,20]]}]})"},
      {"relay", "", "{off_at}…{on_at}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,50],[90,50]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[50,10],[50,90]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,75],[60,75],[60,25],[90,25]]},{"t":"line","p":[[90,25],[40,25],[40,75],[10,75]],"weight":"thin"}]})"},
      {"backlash", "", "gap {width}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,50],[90,50]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[50,10],[50,90]],"stroke":"muted","weight":"thin"},{"t":"poly","p":[[25,80],[45,80],[80,20],[60,20]]}]})"},
      {"friction", "μ", "{coulomb}", R"()"},
      {"moving_average", "", "n {window}", R"({"w":1,"h":1,"g":[{"t":"line","p":[[10,50],[20,35],[30,60],[40,45],[50,70],[60,40],[70,55],[80,30],[90,65]],"stroke":"muted","weight":"thin"},{"t":"line","p":[[10,50],[90,50]]}]})"},
      {"discrete_integrator", "Σ", "T {sample_time}", R"()"},
  };
  for (const auto& r : rows) {
    if (r.type != type) continue;
    return {.compact = true, .glyph = std::string(r.glyph), .value = std::string(r.value),
            .icon = r.icon.empty() ? nlohmann::json(nullptr) : nlohmann::json::parse(r.icon)};
  }
  return {};
}

// Gives `t` its compact plate when the table has one and the block set no look of its own.
inline void apply_compact_view(Template& t) {
  if (t.def.view.empty()) t.def.view = compact_view(t.def.type);
}

}  // namespace paperboat::toolkit::control
