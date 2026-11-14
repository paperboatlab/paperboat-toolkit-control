// control/linear (paperboat#370, control#14): blocks linearized and joined along their wires.

#include <complex>
#include <memory>
#include <string>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <nlohmann/json.hpp>

#include "paperboat/toolkit/analysis.hpp"
#include "paperboat/toolkit/control.hpp"
#include "paperboat/toolkit/registry.hpp"

using namespace paperboat::toolkit;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using json = nlohmann::json;
using C = std::complex<double>;

namespace {

const Registry& registry() {
  static const Registry r = [] {
    Registry out = builtins();
    control::register_control(out);
    return out;
  }();
  return r;
}

void add(AnalysisRequest& req, const std::string& id, const std::string& name, const Values& params) {
  const Template* t = registry().find(name);
  REQUIRE(t != nullptr);
  // As the engine makes them: text params go in the config too.
  Config config = t->config;
  for (const auto& [k, v] : params)
    if (const auto* text = std::get_if<std::string>(&v)) config[k] = *text;
  req.blocks.push_back({.id = id, .def = t->def, .block = t->make(params, config), .params = params, .text = config});
}

json analyse(const AnalysisRequest& req) { return registry().find_analysis("control/linear")->run(req); }

bool has_root(const json& list, C want) {
  for (const auto& r : list)
    if (std::abs(C(r[0], r[1]) - want) < 1e-6) return true;
  return false;
}

C at(const json& p, C s) {
  C v = 0;
  for (double c : p) v = v * s + c;
  return v;
}

// L = PID(kp 2, ki 1) · 1/(s² + s + 1): open from pid.e to plant.y.
AnalysisRequest open_loop() {
  AnalysisRequest req;
  req.dt = 0.01;
  add(req, "pid", "control/pid", {{"kp", 2.0}, {"ki", 1.0}});
  add(req, "plant", "control/second_order", {{"wn", 1.0}, {"zeta", 0.5}});
  req.wires[{"plant", "u"}] = {"pid", "u"};
  req.inputs = {{"pid", "e"}};
  req.outputs = {{"plant", "y"}};
  return req;
}

}  // namespace

TEST_CASE("control/linear joins an open loop: poles, zeros, margins") {
  const json r = analyse(open_loop());
  CHECK(r["order"] == 3);
  CHECK(r["domain"] == "s");
  CHECK(r["inputs"] == json::array({"pid.e"}));
  CHECK(r["outputs"] == json::array({"plant.y"}));
  CHECK(has_root(r["poles"], 0.0));
  CHECK(has_root(r["poles"], C(-0.5, std::sqrt(0.75))));
  CHECK(has_root(r["poles"], C(-0.5, -std::sqrt(0.75))));
  CHECK(has_root(r["zeros"], -0.5));
  CHECK(r["dc_gain"].is_null());  // the integrator: infinite at dc
  CHECK(r["stability"] == "marginal");  // its pole at 0: on the boundary, none outside
  // At the gain crossover |L| is 1 and the phase margin is 180° + ∠L.
  const double wgc = r["margins"]["w_gc"];
  const C L = at(r["num"], C(0, wgc)) / at(r["den"], C(0, wgc));
  CHECK_THAT(std::abs(L), WithinAbs(1.0, 0.02));
  CHECK_THAT(r["margins"]["pm_deg"].get<double>(), WithinAbs(180 + std::arg(L) * 180 / M_PI, 0.5));
}

TEST_CASE("control/linear closes a loop through a std sum") {
  AnalysisRequest req = open_loop();
  add(req, "sum", "math/sum", {{"signs", std::string("+-")}});
  req.wires[{"pid", "e"}] = {"sum", "y"};
  req.wires[{"sum", "b"}] = {"plant", "y"};
  req.inputs = {{"sum", "a"}};
  const json r = analyse(req);
  // s(s² + s + 1) + 2s + 1 = s³ + s² + 3s + 1: stable, and integral action gives a dc gain of 1.
  CHECK(r["stable"] == true);
  CHECK_THAT(r["dc_gain"].get<double>(), WithinAbs(1.0, 1e-9));
  const json& den = r["den"];
  CHECK_THAT(den[1].get<double>(), WithinAbs(1.0, 1e-9));
  CHECK_THAT(den[2].get<double>(), WithinAbs(3.0, 1e-9));
  CHECK_THAT(den[3].get<double>(), WithinAbs(1.0, 1e-9));
  CHECK_THAT(r["step"]["y"].back().get<double>(), WithinAbs(1.0, 0.02));
  CHECK(r["step"]["info"]["overshoot_pct"].get<double>() >= 0);
}

TEST_CASE("control/linear: discrete parts, and what it refuses") {
  AnalysisRequest z;
  z.dt = 0.1;
  add(z, "f", "control/transfer_fn", {{"num", std::string("0.5")}, {"den", std::string("1 -0.5")}, {"domain", std::string("z")}});
  z.inputs = {{"f", "u"}}, z.outputs = {{"f", "y"}};
  const json r = analyse(z);
  CHECK(r["domain"] == "z");
  CHECK(r["sample_time"] == 0.1);
  CHECK(has_root(r["poles"], 0.5));
  CHECK_THAT(r["dc_gain"].get<double>(), WithinAbs(1.0, 1e-12));

  AnalysisRequest bad = open_loop();
  add(bad, "m", "control/median", {});
  CHECK_THROWS_WITH(analyse(bad), ContainsSubstring("m (median) can't be linearized"));

  AnalysisRequest algebraic;
  add(algebraic, "g", "math/gain", {{"k", 1.0}});
  add(algebraic, "s", "math/sum", {{"signs", std::string("+-")}});
  algebraic.wires[{"g", "u"}] = {"s", "y"};
  algebraic.wires[{"s", "b"}] = {"g", "y"};
  algebraic.inputs = {{"s", "a"}}, algebraic.outputs = {{"g", "y"}};
  // g = s − g: 2g = s, solvable (I − DM invertible); with k = −1 it would not be.
  CHECK_NOTHROW(analyse(algebraic));

  AnalysisRequest none = open_loop();
  none.inputs.clear();
  CHECK_THROWS_WITH(analyse(none), ContainsSubstring("no inputs"));
  AnalysisRequest pd = open_loop();
  pd.blocks[0] = {};
  add(pd, "pid2", "control/pid", {{"kd", 1.0}});
  CHECK_THROWS(pd.blocks.back().block->linearize({}));
}
